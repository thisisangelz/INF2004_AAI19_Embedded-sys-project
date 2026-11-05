"""Compile and run tests against the actual navigation and BLE client sources."""
import argparse
import os
from pathlib import Path
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
parser.add_argument('--sdk', required=True, type=Path)
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--coverage', action='store_true')
parser.add_argument('--test', action='append', choices=['navigation', 'payload_client'],
                    help='run only the selected test; may be repeated')
args = parser.parse_args()
tests = Path(__file__).resolve().parent
repo = tests.parents[1]
output = args.output_dir or tests.parent / 'build' / 'host_checks'
output.mkdir(parents=True, exist_ok=True)
btstack = args.sdk / 'lib' / 'btstack' / 'src'
common = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-ffunction-sections',
          '-fdata-sections', '-I', str(tests / 'stubs')]
if args.coverage:
    common += ['--coverage', '-O0', '-g']
selected = args.test or ['navigation', 'payload_client']


def run_test_executable(executable):
    for attempt in range(4):
        try:
            subprocess.run([str(executable)], check=True)
            return
        except OSError as exc:
            if getattr(exc, 'winerror', None) != 4551 or attempt == 3:
                raise
            delay = 2 * (attempt + 1)
            print(f'[RETRY] Application Control inspection; waiting {delay}s', flush=True)
            time.sleep(delay)


for name in selected:
    suffix = '.exe' if os.name == 'nt' else ''
    executable = output / f'test_{name}{suffix}'
    command = common + [str(tests / f'test_{name}.c'), str(tests.parent / 'faults.c')]
    if name == 'payload_client':
        command += ['-DENABLE_BLE', '-I', str(btstack), '-I', str(repo / 'ble_transfer'),
                    str(btstack / 'ad_parser.c'), str(btstack / 'btstack_util.c')]
    command += ['-Wl,--gc-sections', '-o', str(executable)]
    print(f'[BUILD] {name}', flush=True)
    subprocess.run(command, check=True)
    print(f'[RUN] {name}', flush=True)
    run_test_executable(executable)
    print(f'[PASS] {name}', flush=True)
