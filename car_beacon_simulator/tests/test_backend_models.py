"""Adapter-only checks. Never compile, link, boot or execute real firmware here."""
import importlib.util
import io
from types import SimpleNamespace
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('backend_simulator', ROOT/'simulator.py')
s = importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)


class ScenarioTests(unittest.TestCase):
    def test_legacy_fields_and_model_defaults(self):
        scene = s.validate_scenario(dict(bx=2, by=-3))
        self.assertEqual(scene['beacons'], [dict(id=1,x=2,y=-3,mat='pink')])
        self.assertEqual(scene['mats'], scene['beacons'])
        self.assertEqual([scene[k] for k in ('mat_radius','sensor_offset','box_radius','radio_range','colour_enabled')], [.35,.08,.07,5,1])

    def test_mats_have_independent_identity_and_geometry(self):
        scene = s.validate_scenario(dict(beacons=[dict(id=3,x=4,y=2,mat='pink')],
                                        mats=[dict(id=1,x=0,y=1,mat='purple'),dict(id=3,x=0,y=0,mat='blue')]))
        self.assertEqual(scene['beacons'][0]['id'],3)
        self.assertEqual([b['id'] for b in scene['mats']],[1,3])
        self.assertEqual(scene['mats'][1]['mat'],'blue')
        scene = s.validate_scenario(dict(beacons=[dict(id=2,x=3,y=1),dict(id=1,x=0,y=0)]))
        self.assertEqual((scene['bx'],scene['by']),(0,0))
        self.assertEqual(scene['beacons'][1]['mat'],'blue')

    def test_rejects_invalid_arrays_values_and_fractional_switches(self):
        for bad in [[],[dict(id=0,x=0,y=0)],[dict(id=True,x=0,y=0)],
                    [dict(id=1,x=0,y=0)]*2,[dict(id=1,x=0,y=0,mat='none')],
                    [dict(id=1,x=float('nan'),y=0)]]:
            for field in ('beacons','mats'):
                with self.subTest(field=field,bad=bad), self.assertRaises(ValueError): s.validate_scenario({field:bad})
        for bad in [dict(colour_enabled=.5),dict(corrupt_reply=.5),dict(radio_range=0),dict(box_radius=-1),dict(mat_radius=float('inf'))]:
            with self.assertRaises(ValueError): s.validate_scenario(bad)

    def test_cmake_configs_use_selected_source_not_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)
            (source/'ble_transfer').mkdir(); (source/'ble_transfer/beacon_server.c').write_text('// mock')
            for identity in (1,2,3):
                folder=source/('picow_access_point'+('' if identity==1 else str(identity))+'_powerbank')
                folder.mkdir()
                (folder/'CMakeLists.txt').write_text(f'../ble_transfer/beacon_server.c BEACON_ID={identity} WIFI_SSID="source radio {identity}"')
            configs=s.beacon_configs(source)
            self.assertEqual([b['id'] for b in configs],[1,2,3])
            self.assertEqual(configs[2]['ssid'],'source radio 3')
            (source/'picow_access_point3_powerbank/CMakeLists.txt').write_text('../ble_transfer/beacon_server.c BEACON_ID=1 WIFI_SSID="wrong"')
            with self.assertRaises(ValueError): s.beacon_configs(source)

    def test_run_interface_passes_json_with_build_target_using_mocks(self):
        frame=dict(type='frame',distance=1,closest=1,path=0,faults=0)
        result=dict(type='result',success=False)
        raw='\n'.join(json.dumps(e) for e in (frame,result)).encode()
        with mock.patch.object(s,'compile_firmware',return_value=(Path('/mock/exe'),dict(beacons=[dict(id=2)]))) as compile_mock, \
             mock.patch.object(s,'command',return_value=raw) as command_mock:
            data=s.run_simulation(Path('/mock/repo'),'mock-ref',Path('/mock/sdk'),
                                  dict(beacons=[dict(id=2,x=0,y=1)]),save=False,build_target='ap3')
            self.assertFalse(data['result']['success'])
            self.assertEqual(compile_mock.call_args.args[-1],'ap3')
            args=command_mock.call_args.args[0]
            self.assertEqual(args[1],'--scenario-json')
            self.assertEqual(json.loads(args[2])['beacons'][0]['id'],2)

    def test_build_api_does_not_execute_firmware(self):
        request=json.dumps(dict(ref='source-ref',build_target='route')).encode()
        handler=object.__new__(s.Handler)
        handler.path='/api/build'
        handler.headers={'X-Simulator':'1','Content-Length':str(len(request))}
        handler.rfile=io.BytesIO(request)
        handler.server=SimpleNamespace(repo=Path('/mock/repo'),sdk=Path('/mock/sdk'))
        handler.send=mock.Mock()
        with mock.patch.object(s,'compile_firmware',return_value=(Path('/mock/exe'),{'robot_build':'route'})) as build, \
             mock.patch.object(s,'run_simulation') as run:
            handler.do_POST()
            build.assert_called_once_with(Path('/mock/repo'),'source-ref',Path('/mock/sdk'),False,'route')
            run.assert_not_called()
            self.assertEqual(handler.send.call_args.args[0],200)
            self.assertEqual(handler.send.call_args.args[1]['metadata']['robot_build'],'route')

    def test_source_snapshot_copies_common_and_all_beacon_folders(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo=Path(tmp)/'repo'; repo.mkdir()
            for folder in s.SOURCE_FOLDERS:
                (repo/folder).mkdir()
                (repo/folder/'CMakeLists.txt').write_text('selected configuration')
            for name in s.REQUIRED+['imu_mpu9250.c']:
                (repo/'getting_there'/name).write_text('// mock source')
            (repo/'ble_transfer/beacon_server.c').write_text('// mock beacon')
            (repo/'common/buzzer.c').write_text('// mock buzzer')
            with mock.patch.object(s,'BUILD',Path(tmp)/'snapshot'),mock.patch.object(s,'git',return_value='mock-base'):
                source,metadata=s.source_snapshot(repo,'working-tree')
            for folder in s.SOURCE_FOLDERS:
                self.assertTrue((source/folder/'CMakeLists.txt').is_file(),folder)
            self.assertEqual((source/'common/buzzer.c').read_text(),'// mock buzzer')
            self.assertEqual(metadata['base_commit'],'mock-base')


class NativeModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc=shutil.which('clang') or shutil.which('gcc')
        if not cc: raise unittest.SkipTest('No C compiler available')
        cls.tmp=tempfile.TemporaryDirectory(prefix='backend-models-')
        cls.exe=Path(cls.tmp.name)/'mock-models'
        flags=[cc,'-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'tests/backend_stubs'),
               '-I',str(ROOT/'native/stubs'),'-I',str(ROOT/'native'),'-DTHREE_BEACON_ROUTE=1']
        runtime=Path(cls.tmp.name)/'runtime.o'
        subprocess.run(flags+['-Dmain=mocked_native_entry','-c',str(ROOT/'native/runtime.c'),'-o',str(runtime)],check=True,capture_output=True)
        sources=[ROOT/'native'/n for n in ('environment.c','scenario.c','timers.c','diagnostics.c')]
        subprocess.run(flags+[str(ROOT/'tests/backend_models.c'),str(runtime),*[str(p) for p in sources],'-lm','-o',str(cls.exe)],check=True,capture_output=True)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_adapter_world_radio_i2c_alarm_collision_and_markers(self):
        output=subprocess.run([self.exe],check=True,capture_output=True,text=True).stdout
        frame=[json.loads(line) for line in output.splitlines() if json.loads(line)['type']=='frame'][-1]
        self.assertEqual({b['id'] for b in frame['beacons']},{1,2,3})
        self.assertEqual(frame['completed_mask'],7)
        self.assertEqual(frame['active_target'],2)
        self.assertTrue(frame['collision'])
        self.assertEqual(frame['beacons'][1]['payload_events'],1)

    def test_direct_console_errors_are_structured_and_do_not_enter_report(self):
        # No report implementation is linked into this model binary.
        output=subprocess.run([self.exe,'console'],check=True,capture_output=True,text=True).stdout
        event=json.loads(output)
        self.assertEqual(event['type'],'log')
        self.assertEqual(event['source'],'firmware-console')
        self.assertEqual(event['message'],'LOG ERROR: saved report incomplete; 2048 pages\n')

    def test_native_and_python_json_validation(self):
        valid=[{},dict(bx=0,by=2),dict(colour_enabled=False),
               dict(beacons=[dict(id=3,x=-1,y=0)],mats=[dict(id=1,x=0,y=0),dict(id=2,x=1,y=1)])]
        invalid=[dict(beacons=[]),dict(mats=[]),dict(beacons=[dict(id=1,x=0,y=0)]*2),dict(radio_range=0),dict(colour_enabled=.5)]
        for scene in valid+invalid:
            expected=scene in valid
            result=subprocess.run([self.exe,'validate',json.dumps(scene)],capture_output=True)
            self.assertEqual(result.returncode==0,expected,scene)
        for malformed in ('{','{"beacons":[}', '{"x":NaN}', '{"x":1.}', '{"x":0} trailing'):
            self.assertNotEqual(subprocess.run([self.exe,'validate',malformed],capture_output=True).returncode,0)


class TransportSpyTests(unittest.TestCase):
    def test_identity_dispatch_visibility_and_cancel_completion(self):
        cc=shutil.which('clang') or shutil.which('gcc')
        if not cc: self.skipTest('No C compiler available')
        try: sdk=s.find_sdk()
        except RuntimeError: self.skipTest('BTstack headers not available')
        with tempfile.TemporaryDirectory(prefix='backend-transport-') as tmp:
            exe=Path(tmp)/'transport-spies'
            args=[cc,'-std=c11','-Wall','-Wextra','-Werror','-I',ROOT/'tests/backend_stubs',
                  '-I',ROOT/'native/stubs','-I',ROOT/'native','-I',sdk/'lib/btstack/src','-DSIM_DYNAMIC_BLE_THRESHOLD=1',
                  ROOT/'tests/backend_transport.c',ROOT/'native/ble_peer.c',
                  ROOT/'native/scenario.c',ROOT/'native/environment.c','-lm','-o',exe]
            compiled=subprocess.run([str(a) for a in args],capture_output=True,text=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            ran=subprocess.run([exe],capture_output=True,text=True)
            self.assertEqual(ran.returncode,0,ran.stderr)


if __name__=='__main__': unittest.main(verbosity=2)
