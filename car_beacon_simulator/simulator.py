#!/usr/bin/env python3
"""Local, deterministic hardware-in-the-loop substitute for the robot firmware.

Run `python3 simulator.py` and open http://127.0.0.1:8765.
No checkout, firmware source, branch, or UF2 is modified.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parent
DEFAULT_REPO = (ROOT.parent if (ROOT.parent / 'getting_there/getting_there.c').is_file()
                else ROOT.parent / 'INF2004_AAI19_Embedded-sys-project')
BUILD = ROOT / '.build'
RUNS = ROOT / 'runs'
LOCK = threading.Lock()
REQUIRED = ['getting_there.c', 'motion.c', 'payload_client.c', 'faults.c']
OPTIONAL = ['wifi_sampler.c', 'line_search.c', 'colour_sensor.c', 'colour_mat.c', 'robot_cues.c', 'target_beacon.c']
ROBOT_BUILDS = ('ap1', 'ap2', 'ap3', 'route', 'route2')
TWO_BEACON_DEFINE = 'ROUTE_BEACON_COUNT=2'
SOURCE_FOLDERS = ['getting_there','ble_transfer','ble_transfer_powerbank',
                  'common', 'picow_access_point','picow_access_point_powerbank',
                  'picow_access_point2','picow_access_point2_powerbank',
                  'picow_access_point3','picow_access_point3_powerbank']

def supported_source(names):
    return (all('getting_there/'+n in names for n in REQUIRED) and
            any(p in names for p in ['ble_transfer/beacon_server.c','ble_transfer_powerbank/beacon_server.c']) and
            any(p in names for p in ['getting_there/imu_mpu9250.c','pico_rssi_tracker/imu_mpu9250.c']))
DEFAULTS = dict(x=-2.0, y=-1.0, heading=0.0, bx=1.0, by=1.0, speed=.76,
                wheelbase=.14, wifi_ref=-50.0, exponent=2.0, ble_radius=.25,
                stop_at=-1.0, horizon=1000.0, corrupt_reply=0,
                mat_radius=.35, sensor_offset=.08, colour_enabled=1, radio_range=5., box_radius=.07)
LIMITS = dict(x=(-50,50),y=(-50,50),heading=(-360,360),bx=(-50,50),by=(-50,50),
              speed=(.05,2),wheelbase=(.05,1),wifi_ref=(-100,-10),exponent=(.5,5),
              ble_radius=(.05,3),stop_at=(-1,1100),horizon=(5,1100),corrupt_reply=(0,1),
              mat_radius=(.01,5),sensor_offset=(0,1),colour_enabled=(0,1),radio_range=(.01,100),box_radius=(0,1))

def command(args, cwd=None, timeout=60):
    p = subprocess.run([str(a) for a in args], cwd=cwd, capture_output=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError(p.stderr.decode(errors='replace')[-16000:] or p.stdout.decode(errors='replace')[-16000:])
    return p.stdout

def git(repo, *args):
    return command(['git', '-C', repo, *args]).decode().strip()

def find_sdk(explicit=None):
    candidates = [Path(p).expanduser() for p in [explicit, os.environ.get('PICO_SDK_PATH')] if p]
    candidates += sorted((Path.home()/'.pico-sdk/sdk').glob('*'), reverse=True)
    candidates += [Path.home()/'pico/pico-sdk']
    for p in candidates:
        if (p/'lib/btstack/src/btstack.h').is_file(): return p.resolve()
    raise RuntimeError('Pico SDK with BTstack was not found. Pass --sdk /path/to/pico-sdk or set PICO_SDK_PATH.')

def validate_scenario(data):
    if not isinstance(data, dict): raise ValueError('scenario must be an object')
    result = DEFAULTS.copy()
    for key, (lo, hi) in LIMITS.items():
        try: value = float(data.get(key, result[key]))
        except (TypeError, ValueError): raise ValueError(f'{key} must be numeric') from None
        if not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{key} must be between {lo} and {hi}.')
        if key in ('corrupt_reply', 'colour_enabled'):
            if value not in (0, 1): raise ValueError(f'{key} must be 0 or 1')
            value = int(value)
        result[key] = value
    beacons = data.get('beacons', [dict(id=1, x=result['bx'], y=result['by'], mat='pink')])
    if not isinstance(beacons, list) or not 1 <= len(beacons) <= 3:
        raise ValueError('beacons must contain 1 to 3 physical beacons')
    result['beacons'] = []
    seen = set()
    for beacon in beacons:
        if not isinstance(beacon, dict): raise ValueError('Each beacon must be an object')
        identity = beacon.get('id')
        if type(identity) is not int or identity not in (1, 2, 3) or identity in seen:
            raise ValueError('Beacon IDs must be unique integers from 1 to 3')
        seen.add(identity)
        position = {}
        for axis in ('x', 'y'):
            try: value = float(beacon[axis])
            except (KeyError, ValueError, TypeError): raise ValueError(f'Beacon {axis} must be numeric') from None
            if not math.isfinite(value) or not -50 <= value <= 50: raise ValueError('Beacon position must be between -50 and 50')
            position[axis] = value
        mat = beacon.get('mat', ('pink','blue','purple')[identity-1])
        if mat not in ('pink','blue','purple'): raise ValueError('Beacon mat must be pink, blue or purple')
        result['beacons'].append(dict(id=identity, **position, mat=mat))
        if identity == 1: result.update(bx=position['x'], by=position['y'])
    # Canonical ordering makes array order irrelevant to radio identity and overlap handling.
    result['beacons'].sort(key=lambda b: b['id'])
    if 'mats' in data:
        # Same placements schema, with a separate ID namespace from radios.
        mats = validate_scenario({'beacons': data['mats']})['beacons']
        result['mats'] = mats
    else:
        result['mats'] = [b.copy() for b in result['beacons']]
    return result

def source_snapshot(repo: Path, ref: str):
    if not repo.is_dir(): raise ValueError(f'Repository folder does not exist: {repo}')
    if ref == 'working-tree':
        # Copy source only; no SDK/build directories, and no mutation of edits.
        paths = sorted(p for folder in SOURCE_FOLDERS for p in (repo/folder).rglob('*')
                       if p.is_file() and p.suffix in ('.c','.h','.txt','.md','.gatt') and 'build' not in p.relative_to(repo).parts)
        paths += sorted((repo/'pico_rssi_tracker').glob('imu_mpu9250.[ch]'))
        digest = hashlib.sha256()
        contents = [(str(p.relative_to(repo)), p.read_bytes()) for p in paths]
        for name, blob in contents: digest.update(name.encode()+b'\0'+blob)
        sha = 'working-' + digest.hexdigest()[:20]
        base_commit = git(repo, 'rev-parse', 'HEAD')
    else:
        if not ref or ref.startswith('-') or len(ref)>200: raise ValueError('Invalid Git reference.')
        sha = git(repo, 'rev-parse', '--verify', ref+'^{commit}')
        base_commit = sha
        names = git(repo, 'ls-tree','-r','--name-only',sha).splitlines()
        contents = [(n, command(['git','-C',repo,'show',sha+':'+n])) for n in names
                    if (n.startswith(tuple(folder+'/' for folder in SOURCE_FOLDERS)) and '/build/' not in n)
                    or n in ('pico_rssi_tracker/imu_mpu9250.c','pico_rssi_tracker/imu_mpu9250.h')]
    names = {name for name,_ in contents}
    missing = [n for n in REQUIRED if 'getting_there/'+n not in names]
    if not any(p in names for p in ['getting_there/imu_mpu9250.c','pico_rssi_tracker/imu_mpu9250.c']):
        missing.append('imu_mpu9250.c')
    if not any(p in names for p in ['ble_transfer/beacon_server.c','ble_transfer_powerbank/beacon_server.c']):
        missing.append('beacon_server.c')
    if missing:
        raise ValueError('This branch does not expose the supported getting_there firmware interface. Missing: '+', '.join(missing)+'. Select main/Alvin_2 or add an adapter; no substitute controller will be run.')
    source = BUILD / sha / 'source'
    for name, blob in contents:
        dest = source/name
        if not dest.is_file() or dest.read_bytes()!=blob:
            dest.parent.mkdir(parents=True,exist_ok=True); dest.write_bytes(blob)
    return source, dict(ref=ref, commit=sha, base_commit=base_commit, repository=str(repo))

def beacon_configs(source):
    """Read each beacon's actual source/identity from its selected CMake target."""
    source = source.resolve()
    result = []
    for identity in (1, 2, 3):
        suffix = '' if identity == 1 else str(identity)
        target = next((source/p/'CMakeLists.txt' for p in
                       [f'picow_access_point{suffix}_powerbank', f'picow_access_point{suffix}']
                       if (source/p/'CMakeLists.txt').is_file()), None)
        if target is None:
            if identity == 1: raise ValueError('AP1 CMake configuration is missing')
            continue  # Older snapshots expose only AP1. Never invent AP2/AP3 firmware.
        cmake = target.read_text()
        path = re.search(r'(\.\./[A-Za-z0-9_/.-]+/beacon_server\.c)', cmake)
        ssid = re.search(r'WIFI_SSID="([^"\n]+)"', cmake)
        beacon_id = re.search(r'BEACON_ID=(\d+)', cmake)
        if not path or not ssid or not beacon_id or int(beacon_id.group(1)) != identity:
            raise ValueError(f'Unsupported AP{identity} CMake configuration')
        beacon_source = (target.parent/path.group(1)).resolve()
        if not beacon_source.is_relative_to(source.resolve()) or not beacon_source.is_file():
            raise ValueError('Beacon source must exist inside the selected snapshot')
        result.append(dict(id=identity, target=str(target.relative_to(source)),
                           source=str(beacon_source.relative_to(source)), ssid=ssid.group(1),
                           buzzer='../common/buzzer.c' in cmake))
    return result


def compile_firmware(repo, ref, sdk, moving=False, build_target='ap1'):
    if build_target not in ROBOT_BUILDS: raise ValueError('Invalid build_target')
    source, info = source_snapshot(repo, ref)
    btstack = sdk/'lib/btstack/src'
    cc = shutil.which(os.environ.get('CC','clang')) or shutil.which('gcc')
    if not cc: raise RuntimeError('A native C compiler is required (clang or gcc).')
    fw = source/'getting_there'
    configs = beacon_configs(source)
    if build_target != 'ap1' and not (fw/'target_beacon.h').is_file():
        raise ValueError('Selected source does not support robot target selection')
    if build_target == 'route' and not any('THREE_BEACON_ROUTE' in p.read_text() for p in fw.glob('*') if p.suffix in ('.c','.h')):
        raise ValueError('Selected source does not implement THREE_BEACON_ROUTE')
    if build_target == 'route2':
        cmake = fw/'CMakeLists.txt'
        if not cmake.is_file() or not re.search(
                r'target_compile_definitions\s*\(\s*getting_there_route2\s+PRIVATE\s+'
                + TWO_BEACON_DEFINE + r'\s*\)', cmake.read_text()):
            raise ValueError('Selected source does not implement the two-beacon route (getting_there_route2)')
    digest = hashlib.sha256()
    digest.update(Path(__file__).read_bytes())
    for p in sorted((ROOT/'native').rglob('*')):
        if p.is_file(): digest.update(p.read_bytes())
    digest.update(str(sdk).encode()); digest.update(str(moving).encode()); digest.update(build_target.encode())
    digest.update((btstack/'btstack_event.h').read_bytes()); digest.update(cc.encode())
    exe = source.parent / ('robot-'+digest.hexdigest()[:16])
    actual_sources=[fw/n for n in REQUIRED+OPTIONAL if (fw/n).exists()]
    imu=fw/'imu_mpu9250.c'
    if not imu.exists(): imu=source/'pico_rssi_tracker/imu_mpu9250.c'
    actual_sources.append(imu)
    actual_sources.append(fw/'report_store.c')
    report_source = fw/'run_report.c'
    if not (fw/'run_report.c').is_file(): raise ValueError('Selected source lacks the run_report storage interface')
    robot_buzzer = (fw/'robot_cues.c').exists()
    if robot_buzzer: actual_sources.append(source/'common/buzzer.c')
    actual_sources.extend(source/c['source'] for c in configs)
    info.update(moving_samples=moving, sdk=str(sdk), compiler=cc, robot_build=build_target,
                adapter_hash=digest.hexdigest(), model_version=3, scalar_reference_id=1, beacons=configs,
                car_source='getting_there/getting_there.c', beacon_source=configs[0]['source'],
                beacon_target=configs[0]['target'], beacon_ssid=configs[0]['ssid'],
                beacon_configs=configs, beacon_mode='branch-firmware',
                actual_sources=sorted({str(p.relative_to(source)) for p in actual_sources+[report_source]}),
                model='differential drive + isolated branch beacon firmware + TCS34725 calibrated concentric mats',
                report_storage='selected run_report.c + report_store.c with RAM-backed physical flash; real truncation, pending and 512 KiB journal limits',
                beacon_idle='boot yields at idle loop; radio and alarm callbacks retained; foreground heartbeat loop omitted',
                geometry='equal radial bands, adjustable assumed mat_radius; wheel-midpoint point body collides with circular boxes',
                radio_model='noiseless log-distance; radio_range is an adjustable illustrative visibility floor, not chair calibration')
    if not exe.exists():
        generated=source.parent/('generated-'+digest.hexdigest()[:16]); generated.mkdir(exist_ok=True)
        command([sys.executable,sdk/'lib/btstack/tool/compile_gatt.py',
                 source/'ble_transfer/file_transfer.gatt',generated/'file_transfer.h'])
        flags = [cc,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror',
                 '-ffunction-sections','-fdata-sections','-DENABLE_BLE',
                 '-DLINE_SEARCH_MOVING_SAMPLES='+str(int(moving)),
                 '-I',ROOT/'native/stubs','-I',fw,'-I',btstack,'-I',source/'ble_transfer',
                 '-I',ROOT/'native','-I',source/'pico_rssi_tracker','-I',source/'common','-I',generated]
        if 'gatt_event_query_complete_get_service_id' in (btstack/'btstack_event.h').read_text():
            flags += ['-DSIM_EXTENDED_GATT=1']
        if 'payload_client_close_threshold(' in (fw/'payload_client.c').read_text(): flags += ['-DSIM_DYNAMIC_BLE_THRESHOLD=1']
        if (fw/'target_beacon.h').exists(): flags += ['-DSIM_TARGET_API=1']
        if (fw/'target_beacon.h').exists() and 'target_beacon_id(' in (fw/'target_beacon.h').read_text():
            flags += ['-DSIM_DYNAMIC_TARGET_API=1']
        if build_target == 'route': flags += ['-DTHREE_BEACON_ROUTE=1']
        elif build_target == 'route2': flags += ['-D'+TWO_BEACON_DEFINE]
        else: flags += ['-DTARGET_BEACON_ID='+build_target[-1]]
        objects = []
        symbols = ['beacon_firmware_main','native_beacon_boot','native_beacon_connect','native_beacon_enable_notifications',
                   'native_beacon_write','native_beacon_take_notification','native_beacon_value_handle',
                   'native_beacon_service_start','native_beacon_service_end','native_beacon_advertisement',
                   'native_beacon_ssid','att_server_register_packet_handler','gap_advertisements_set_params',
                   'gap_advertisements_set_data','gap_advertisements_enable','att_server_request_can_send_now_event',
                   'att_server_notify','cyw43_arch_enable_ap_mode','cyw43_arch_gpio_put','cyw43_wifi_pm',
                   'cyw43_pm_value','powerbank_load_start','buzzer_init','buzzer_play','buzzer_play_looping_tail',
                   'buzzer_stop','buzzer_busy']
        for config in configs:
            identity = config['id']; wrapper = generated/f'beacon{identity}.c'
            # Each inclusion has separate C static storage and uniquely named exports.
            wrapper.write_text(''.join(f'#define {name} sim_ap{identity}_{name}\n' for name in symbols) +
                               f'#define SIM_BEACON_ID {identity}\n' +
                               '#include "beacon_adapter.c"\n' +
                               ('#include "buzzer.c"\n' if config['buzzer'] else ''))
            obj=generated/f'beacon{identity}.o'; objects.append(obj)
            command(flags+['-I',(source/config['source']).parent,'-DBEACON_ID='+str(identity),
                           '-DWIFI_SSID='+json.dumps(config['ssid']),'-c',wrapper,'-o',obj],timeout=90)
        registry=generated/'beacon_registry.c'
        registry.write_text('#include "beacon_adapter.h"\n' +
                            ''.join(f'SIM_DECLARE_BEACON(sim_ap{c["id"]}_);\n' for c in configs) +
                            'const NativeBeacon native_beacons[3] = {\n' +
                            ''.join(f'[{c["id"]-1}] = SIM_BEACON_ENTRY(sim_ap{c["id"]}_),\n' for c in configs) + '};\n')
        cmd = flags+[ROOT/'native'/n for n in ('navigation.c','runtime.c','environment.c','scenario.c','timers.c','ble_peer.c','diagnostics.c','report_adapter.c')]
        cmd += [registry,*objects]
        cmd += [p for p in actual_sources if p.name not in ('getting_there.c','payload_client.c','beacon_server.c')]
        cmd += [btstack/'ad_parser.c',btstack/'btstack_util.c',
                '-Wl,-dead_strip' if sys.platform=='darwin' else '-Wl,--gc-sections','-lm','-o',exe]
        command(cmd, timeout=90)
    return exe, info

def run_simulation(repo, ref, sdk, scenario=None, moving=False, save=True, build_target='ap1'):
    scenario = validate_scenario(scenario if scenario is not None else {})
    with LOCK:
        exe, metadata = compile_firmware(repo, ref, sdk, moving, build_target)
        start=time.monotonic()
        available = {b['id'] for b in metadata['beacons']}
        if any(b['id'] not in available for b in scenario['beacons']):
            raise ValueError('Scenario requests a beacon absent from the selected source CMake configurations')
        raw=command([exe,'--scenario-json',json.dumps(scenario,separators=(',',':'))],timeout=30)
    events=[json.loads(line) for line in raw.decode().splitlines() if line.strip()]
    frames=[e for e in events if e['type']=='frame']
    logs=[e for e in events if e['type']=='log']
    results=[e for e in events if e['type']=='result']
    if not frames or not results: raise RuntimeError('The native firmware process did not produce a complete trace.')
    result=results[-1]; final=frames[-1]
    result.update(distance=final['distance'],closest=final['closest'],path=final['path'],faults=final['faults'],
                  wall_seconds=round(time.monotonic()-start,3),
                  failure_messages=[e['message'].strip() for e in logs if 'FAILED:' in e['message'] or 'FAULT:' in e['message'] or 'LOG ERROR:' in e['message']],
                  beacons=final.get('beacons', []), active_target=final.get('active_target'),
                  completed_mask=final.get('completed_mask', 0))
    identifier=str(time.time_ns())+'-'+uuid.uuid4().hex[:8]
    data=dict(id=identifier,metadata=metadata,scenario=scenario,result=result,frames=frames,logs=logs)
    if save:
        RUNS.mkdir(exist_ok=True)
        (RUNS/(identifier+'.json')).write_text(json.dumps(data,separators=(',',':')))
    return data

def refs(repo):
    output=git(repo,'for-each-ref','--format=%(refname:short)','refs/heads','refs/remotes/origin','refs/tags')
    result=[]
    for ref in output.splitlines():
        if ref.endswith('/HEAD'): continue
        names=set(git(repo,'ls-tree','-r','--name-only',ref,*SOURCE_FOLDERS,'pico_rssi_tracker/imu_mpu9250.c').splitlines())
        supported=supported_source(names)
        beacon=next((p for p in ['ble_transfer_powerbank/beacon_server.c','ble_transfer/beacon_server.c'] if p in names),None)
        result.append(dict(name=ref,supported=supported,commit=git(repo,'rev-parse','--short',ref),
                           car_source='getting_there/getting_there.c' if supported else None,
                           beacon_source=beacon))
    return sorted(result,key=lambda r:(r['name']!='origin/main',not r['supported'],r['name']))

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def send(self, code, data, content_type='application/json'):
        body=json.dumps(data).encode() if content_type=='application/json' else data
        self.send_response(code); self.send_header('Content-Type',content_type)
        self.send_header('Cache-Control','no-store'); self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Content-Length',str(len(body))); self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        try:
            if self.path=='/api/config':
                return self.send(200,dict(repo=str(self.server.repo),sdk=str(self.server.sdk),refs=refs(self.server.repo),defaults=DEFAULTS))
            if self.path=='/api/runs':
                entries=[]
                for p in sorted(RUNS.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:30]:
                    d=json.loads(p.read_text()); entries.append({k:d[k] for k in ['id','metadata','scenario','result']})
                return self.send(200,entries)
            if self.path.startswith('/api/run/'):
                identifier=self.path.split('/')[-1]
                if not identifier or any(c not in '0123456789abcdef-working' for c in identifier): raise ValueError('Invalid run ID')
                p=RUNS/(identifier+'.json')
                if not p.exists(): return self.send(404,dict(error='Run not found'))
                return self.send(200,json.loads(p.read_text()))
            files={'/':'index.html','/app.js':'app.js','/style.css':'style.css'}
            if self.path not in files: return self.send(404,dict(error='Not found'))
            p=ROOT/'dist'/files[self.path]
            mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'}[p.suffix]
            self.send(200,p.read_bytes(),mime)
        except (ValueError,RuntimeError,subprocess.TimeoutExpired) as e: self.send(400,dict(error=str(e)))
    def do_POST(self):
        # No CORS. A custom header prevents another website submitting a run.
        if self.headers.get('X-Simulator')!='1': return self.send(403,dict(error='Local simulator request required'))
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length>65536: raise ValueError('Request too large')
            data=json.loads(self.rfile.read(length) or b'{}')
            if self.path=='/api/build':
                with LOCK:
                    exe, metadata = compile_firmware(self.server.repo, data.get('ref','origin/main'),
                                                     self.server.sdk, bool(data.get('moving')),
                                                     data.get('build_target','ap1'))
                return self.send(200,dict(executable=str(exe),metadata=metadata))
            if self.path=='/api/run':
                ref=data.get('ref','origin/main')
                result=run_simulation(self.server.repo,ref,self.server.sdk,data.get('scenario'),bool(data.get('moving')),
                                      build_target=data.get('build_target','ap1'))
                return self.send(200,result)
            if self.path=='/api/fetch':
                git(self.server.repo,'fetch','origin')
                return self.send(200,dict(refs=refs(self.server.repo)))
            return self.send(404,dict(error='Not found'))
        except (ValueError,RuntimeError,subprocess.TimeoutExpired,json.JSONDecodeError) as e: self.send(400,dict(error=str(e)))
        except Exception as e: self.send(500,dict(error=f'{type(e).__name__}: {e}'))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=DEFAULT_REPO)
    parser.add_argument('--sdk')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--build-only',action='store_true')
    parser.add_argument('--build-target',choices=ROBOT_BUILDS,default='ap1')
    parser.add_argument('--ref')
    parser.add_argument('--moving-samples',action='store_true')
    parser.add_argument('--scenario',type=Path)
    args=parser.parse_args()
    repo=args.repo.expanduser().resolve(); sdk=find_sdk(args.sdk)
    if args.build_only:
        exe, metadata = compile_firmware(repo,args.ref or 'origin/main',sdk,args.moving_samples,args.build_target)
        print(json.dumps(dict(executable=str(exe),metadata=metadata),indent=2)); return
    if args.headless:
        setup=json.loads(args.scenario.read_text()) if args.scenario else {}
        data=run_simulation(repo,args.ref or setup.get('ref','origin/main'),sdk,setup.get('scenario',setup),
                            args.moving_samples or bool(setup.get('moving',False)),
                            build_target=setup.get('build_target',args.build_target))
        print(json.dumps(dict(id=data['id'],metadata=data['metadata'],result=data['result']),indent=2)); return
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    server.repo=repo; server.sdk=sdk
    print(f'Beacon Lab: http://127.0.0.1:{args.port}\nFirmware repository: {repo}',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()

if __name__=='__main__': main()
