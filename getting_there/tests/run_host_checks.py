"""Compile and run tests against the actual navigation and BLE client sources."""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--sdk', required=True, type=Path)
args = parser.parse_args()
tests = Path(__file__).resolve().parent
repo = tests.parents[1]
output = tests.parent / 'build' / 'host_checks'
output.mkdir(parents=True, exist_ok=True)
btstack = args.sdk / 'lib' / 'btstack' / 'src'
# BTstack added service_id and connection_id to GATT events in newer SDKs.
# Keep hand-built test events aligned with the selected SDK, not a guessed version.
event_header = (btstack / 'btstack_event.h').read_text()
extended_events = 'gatt_event_query_complete_get_service_id' in event_header
common = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
          '-fdata-sections', '-I', str(tests / 'stubs'),
          '-I', str(tests.parent)]
for name in ['navigation', 'payload_client', 'report', 'motion', 'flash', 'colour']:
    executable = output / f'test_{name}.exe'
    command = common + [str(tests / f'test_{name}.c'), str(tests.parent / 'faults.c')]
    if name == 'flash':
        command += ['-DPICO_FLASH_SIZE_BYTES=2097152', '-DPICO_FLASH_BANK_TOTAL_SIZE=8192',
                    '-DPICO_FLASH_BANK_STORAGE_OFFSET=1564672']
    if name == 'navigation':
        command += ['-DLINE_SEARCH_MOVING_SAMPLES=1', str(tests.parent / 'line_search.c'),
                    str(tests.parent / 'wifi_sampler.c'), str(tests.parent / 'colour_mat.c')]
    if name == 'colour':
        command += [str(tests.parent / 'colour_sensor.c'), str(tests.parent / 'colour_mat.c')]
    if name == 'payload_client':
        command += ['-DENABLE_BLE', '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                    str(btstack / 'ad_parser.c'), str(btstack / 'btstack_util.c')]
        if extended_events:
            command += ['-DTEST_EXTENDED_GATT_EVENTS=1']
    command += ['-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
                '-lm', '-o', str(executable)]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)

# Each robot target build (getting_there, robot_ap2, robot_ap3) must look for
# the Wi-Fi SSID and BLE beacon ID that the matching beacon firmware uses.
import re
for beacon in (1, 2, 3):
    folder = repo / f"picow_access_point{'' if beacon == 1 else beacon}_powerbank"
    cmake = (folder / 'CMakeLists.txt').read_text()
    ssid = re.search(r'WIFI_SSID="([^"]+)"', cmake).group(1)
    beacon_id = int(re.search(r'BEACON_ID=(\d+)', cmake).group(1))
    executable = output / f'test_target_{beacon}.exe'
    subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(tests.parent),
                    f'-DTARGET_BEACON_ID={beacon}', f'-DEXPECTED_ID={beacon_id}',
                    f'-DEXPECTED_SSID="{ssid}"', str(tests / 'test_target.c'), '-o', str(executable)],
                   check=True)
    subprocess.run([str(executable)], check=True)
