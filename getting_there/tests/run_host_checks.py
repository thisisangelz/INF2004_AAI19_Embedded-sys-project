"""Compile and run tests against the actual navigation and BLE client sources."""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--build-only', action='store_true', help='Compile/link fixtures; never execute them')
parser.add_argument('--output', type=Path, help='Separate generated output directory')
parser.add_argument('--sdk', required=True, type=Path)
args = parser.parse_args()
tests = Path(__file__).resolve().parent
repo = tests.parents[1]
output = args.output or (tests.parent / 'build' / 'host_checks')
output.mkdir(parents=True, exist_ok=True)
btstack = args.sdk / 'lib' / 'btstack' / 'src'
# BTstack added service_id and connection_id to GATT events in newer SDKs.
# Keep hand-built test events aligned with the selected SDK, not a guessed version.
event_header = (btstack / 'btstack_event.h').read_text()
extended_events = 'gatt_event_query_complete_get_service_id' in event_header
common = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
          '-fdata-sections', '-I', str(tests / 'stubs'),
          '-I', str(tests.parent)]
dead_strip = '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections'
for name in ['navigation', 'route', 'route2', 'payload_client', 'payload_client_route', 'report', 'motion', 'flash', 'colour']:
    executable = output / f'test_{name}.exe'
    source = {'route2': 'route', 'payload_client_route': 'payload_client'}.get(name, name)
    command = common + [str(tests / f'test_{source}.c'), str(tests.parent / 'faults.c'),
                        str(tests.parent / 'target_beacon.c')]
    if name in ('route', 'payload_client_route'):
        command += ['-DTHREE_BEACON_ROUTE=1']
    if name == 'route2':
        command += ['-DROUTE_BEACON_COUNT=2']
    if name == 'flash':
        command += ['-DPICO_FLASH_SIZE_BYTES=2097152', '-DPICO_FLASH_BANK_TOTAL_SIZE=8192',
                    '-DPICO_FLASH_BANK_STORAGE_OFFSET=1564672']
    if source in ('navigation', 'route'):
        command += ['-DLINE_SEARCH_MOVING_SAMPLES=1', str(tests.parent / 'line_search.c'),
                    str(tests.parent / 'wifi_sampler.c'), str(tests.parent / 'colour_mat.c')]
    if name == 'colour':
        command += [str(tests.parent / 'colour_sensor.c'), str(tests.parent / 'colour_mat.c')]
    if name in ('payload_client', 'payload_client_route'):
        command += ['-DENABLE_BLE', '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                    str(btstack / 'ad_parser.c'), str(btstack / 'btstack_util.c')]
        if extended_events:
            command += ['-DTEST_EXTENDED_GATT_EVENTS=1']
    command += [dead_strip, '-lm', '-o', str(executable)]
    subprocess.run(command, check=True)
    if not args.build_only:
        subprocess.run([str(executable)], check=True)

for filter_name in ('KALMAN', 'TRIMMED_MEAN'):
    executable = output / f'test_wifi_filter_{filter_name.lower()}.exe'
    subprocess.run(common + [f'-DWIFI_RSSI_FILTER=WIFI_RSSI_FILTER_{filter_name}',
                             str(tests / 'test_wifi_filter.c'), str(tests.parent / 'wifi_sampler.c'),
                             dead_strip, '-o', str(executable)], check=True)
    if not args.build_only:
        subprocess.run([str(executable)], check=True)

# Each robot target build (getting_there, robot_ap2, robot_ap3) must look for
# the Wi-Fi SSID and BLE beacon ID that the matching beacon firmware uses.
import re
for beacon, route in ((1, 0), (2, 0), (3, 0), (1, 3), (1, 2)):
    folder = repo / f"picow_access_point{'' if beacon == 1 else beacon}_powerbank"
    cmake = (folder / 'CMakeLists.txt').read_text()
    ssid = re.search(r'WIFI_SSID="([^"]+)"', cmake).group(1)
    beacon_id = int(re.search(r'BEACON_ID=(\d+)', cmake).group(1))
    executable = output / f'test_target_{beacon}_route{route}.exe'
    subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(tests.parent), '-I', str(tests / 'stubs'),
                    f'-DROUTE_BEACON_COUNT={route}',
                    f'-DTARGET_BEACON_ID={beacon}', f'-DEXPECTED_ID={beacon_id}',
                    f'-DEXPECTED_SSID="{ssid}"', str(tests / 'test_target.c'), str(tests.parent / 'target_beacon.c'), '-o', str(executable)],
                   check=True)
    if not args.build_only:
        subprocess.run([str(executable)], check=True)

if args.build_only:
    print(f"Compiled/linked all host fixtures into {output}; no tests executed.")
