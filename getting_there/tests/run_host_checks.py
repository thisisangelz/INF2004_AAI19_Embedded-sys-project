"""Compile and run tests against the actual navigation and BLE client sources."""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--sdk', required=True, type=Path)
args = parser.parse_args()
tests = Path(__file__).resolve().parent
repo = tests.parents[1]
output = tests.parent / 'build' / 'host_checks'
output.mkdir(parents=True, exist_ok=True)
btstack = args.sdk / 'lib' / 'btstack' / 'src'
common = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
          '-fdata-sections', '-I', str(tests / 'stubs')]
for name in ['navigation', 'payload_client']:
    executable = output / f'test_{name}.exe'
    command = common + [str(tests / f'test_{name}.c')]
    if name == 'payload_client':
        command += ['-DENABLE_BLE', '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                    str(btstack / 'ad_parser.c'), str(btstack / 'btstack_util.c')]
    command += ['-Wl,--gc-sections', '-o', str(executable)]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)
