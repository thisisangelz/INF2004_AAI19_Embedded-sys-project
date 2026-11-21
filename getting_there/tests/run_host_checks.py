"""Compile and run tests against the actual navigation and BLE client sources."""
import argparse
from pathlib import Path
import re
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--sdk', required=True, type=Path)
parser.add_argument('--sanitize', nargs='?', const='address,undefined',
                    choices=['address,undefined', 'undefined'],
                    help='Enable sanitizers; use undefined where the ASan runtime is unavailable')
args = parser.parse_args()
args.sdk = args.sdk.resolve()
tests = Path(__file__).resolve().parent
repo = tests.parents[1]
output = tests.parent / 'build' / ('host_checks_sanitized' if args.sanitize else 'host_checks')
output.mkdir(parents=True, exist_ok=True)
btstack = args.sdk / 'lib' / 'btstack' / 'src'
if not (btstack / 'btstack_event.h').is_file():
    parser.error('--sdk must point to a Pico SDK checkout with its lib/btstack submodule initialized')
# BTstack added service_id and connection_id to GATT events in newer SDKs.
# Keep hand-built test events aligned with the selected SDK, not a guessed version.
event_header = (btstack / 'btstack_event.h').read_text()
extended_events = 'gatt_event_query_complete_get_service_id' in event_header
common = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
          '-fdata-sections', '-O2', '-I', str(tests / 'stubs'),
          '-I', str(tests.parent)]
if args.sanitize:
    common += ['-g', f'-fsanitize={args.sanitize}', '-fno-sanitize-recover=all',
               '-fno-omit-frame-pointer']
for name in ['navigation', 'diagnostic', 'payload_client', 'report', 'motion', 'flash', 'colour', 'colour_scan', 'buzzer', 'beacon_log']:
    executable = output / f'test_{name}.exe'
    source_name = 'navigation' if name == 'diagnostic' else name
    command = common + [str(tests / f'test_{source_name}.c'), str(tests.parent / 'faults.c')]
    if name == 'flash':
        command += ['-DPICO_FLASH_SIZE_BYTES=2097152', '-DPICO_FLASH_BANK_TOTAL_SIZE=8192',
                    '-DPICO_FLASH_BANK_STORAGE_OFFSET=1564672']
    if name in ('navigation', 'diagnostic'):
        command += [f'-DLINE_SEARCH_MOVING_SAMPLES={0 if name == "diagnostic" else 1}', str(tests.parent / 'line_search.c'),
                    str(tests.parent / 'wifi_sampler.c'), str(tests.parent / 'colour_mat.c'),
                    str(tests.parent / 'colour_scan.c'),
                    str(tests.parent / 'target_beacon.c')]
        if name == 'diagnostic':
            command += ['-DCOLOUR_SCAN_TEST=1']
    if name == 'colour':
        command += [str(tests.parent / 'colour_sensor.c'), str(tests.parent / 'colour_mat.c')]
    if name == 'colour_scan':
        command += [str(tests.parent / 'colour_scan.c'), str(tests.parent / 'colour_mat.c'),
                    str(tests.parent / 'line_search.c')]
    if name == 'payload_client':
        command += ['-DENABLE_BLE', '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                    str(tests.parent / 'target_beacon.c'),
                    str(btstack / 'ad_parser.c'), str(btstack / 'btstack_util.c')]
        if extended_events:
            command += ['-DTEST_EXTENDED_GATT_EVENTS=1']
    command += ['-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
                '-lm', '-o', str(executable)]
    subprocess.run(command, check=True, timeout=120)
    subprocess.run([str(executable)], check=True, timeout=120)

# Each robot target build (getting_there, robot_ap2, robot_ap3) must look for
# the Wi-Fi SSID and BLE beacon ID that the matching beacon firmware uses.
for beacon in (1, 2, 3):
    folder = repo / f"picow_access_point{'' if beacon == 1 else beacon}_powerbank"
    cmake = (folder / 'CMakeLists.txt').read_text()
    ssid = re.search(r'WIFI_SSID="([^"]+)"', cmake).group(1)
    beacon_id = int(re.search(r'BEACON_ID=(\d+)', cmake).group(1))
    executable = output / f'test_target_{beacon}.exe'
    subprocess.run(common + [
                    f'-DTARGET_BEACON_ID={beacon}', f'-DEXPECTED_ID={beacon_id}',
                    f'-DEXPECTED_SSID="{ssid}"', str(tests / 'test_target.c'),
                    str(tests.parent / 'target_beacon.c'), '-o', str(executable)],
                   check=True, timeout=120)
    subprocess.run([str(executable)], check=True, timeout=120)

# Exercise the actual beacon parser with its generated GATT handles. No ARM
# build is needed to run the server tests, including each compiled beacon ID.
generated = output / 'generated'
generated.mkdir(exist_ok=True)
subprocess.run([sys.executable, str(btstack.parent / 'tool' / 'compile_gatt.py'),
                str(repo / 'ble_transfer' / 'file_transfer.gatt'),
                str(generated / 'file_transfer.h')], check=True, timeout=120)
for beacon in (1, 2, 3):
    executable = output / f'test_beacon_server_{beacon}.exe'
    command = common + ['-DENABLE_BLE', f'-DBEACON_ID={beacon}',
                        '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                        '-I', str(repo / 'common'), '-I', str(generated),
                        str(tests / 'test_beacon_server.c'),
                        str(repo / 'ble_transfer_powerbank' / 'beacon_log.c'),
                        str(btstack / 'btstack_util.c'),
                        '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
                        '-o', str(executable)]
    subprocess.run(command, check=True, timeout=120)
    subprocess.run([str(executable)], check=True, timeout=120)
