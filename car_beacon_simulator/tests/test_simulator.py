"""Behavioural checks of the simulator against real native firmware execution.

These check the harness, not a rewritten navigation algorithm. Some navigation
scenarios intentionally fail: failure must remain visible, never be corrected
by a simulator that secretly steers towards the known beacon coordinates.
"""
import math
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import simulator as s

class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk=s.find_sdk()
        cls.commit=s.git(s.DEFAULT_REPO,'rev-parse','origin/main')
        cls.baseline=s.run_simulation(s.DEFAULT_REPO,cls.commit,cls.sdk,
                                      dict(x=0,y=0,bx=1,by=0),save=False)

    def run_scene(self,**kw):
        return s.run_simulation(s.DEFAULT_REPO,self.commit,self.sdk,kw,save=False)

    def test_actual_firmware_reaches_and_verifies_both_payloads(self):
        d=self.baseline; f=d['frames'][-1]
        self.assertTrue(d['result']['success'])
        self.assertTrue(f['peer_verified'] and f['peer_complete'] and f['payload_verified'])
        self.assertEqual((f['rx_bytes'],f['tx_bytes']),(39,26))
        self.assertEqual(f['faults'],0)
        self.assertEqual((f['left'],f['right']),(0,0))
        self.assertTrue(any(f['buzzer'] for f in d['frames']))
        # Proximity comes from the selected firmware, not a fixed 25 cm target.
        # Radio reports are rounded to integer dBm before firmware filtering.
        self.assertGreaterEqual(f['ble'],f['ble_threshold']-.5-1e-6)
        # v18 also plays startup, calibration and in-range cues. Those are
        # telemetry, not additional payloads or evidence of route completion.
        self.assertTrue(any('COMPLETION BEEPS:' in entry['message'] for entry in d['logs']))

    def test_identical_inputs_produce_identical_execution(self):
        again=self.run_scene(x=0,y=0,bx=1,by=0)
        self.assertEqual(self.baseline['frames'],again['frames'])
        self.assertEqual(self.baseline['logs'],again['logs'])

    def test_noiseless_signal_tracks_distance_and_is_not_goal_directed(self):
        for a,b in zip(self.baseline['frames'],self.baseline['frames'][1:]):
            if b['distance']<a['distance']-1e-6:
                self.assertGreaterEqual(b['wifi'],a['wifi'])
                self.assertGreaterEqual(b['ble'],a['ble'])
            if abs(b['distance']-a['distance'])<1e-10:
                self.assertEqual(b['wifi'],a['wifi'])
        far=self.run_scene(x=-2,y=-1,bx=1,by=1)
        initial=[]
        for entry in far['logs']:
            if 'DIRECTION SEARCH' in entry['message']: break
            initial.append(entry['message'])
        self.assertTrue(any('continue through inconclusive readings' in m for m in initial))
        points=[int(m.group(1)) for text in initial
                for m in [re.search(r'FOLLOW_POINT,powered_ms=(\d+)',text)] if m]
        self.assertTrue(points)
        self.assertGreater(max(points),2400)
        # Do not prescribe a successful route: preserve whichever result the
        # actual firmware reports after its own backtracking/limit decisions.
        self.assertEqual(far['result']['success'],any(
            entry['message'].strip()=='RESULT: PAYLOAD CONFIRMED' for entry in far['logs']))

    def test_close_start_transfers_without_moving(self):
        d=self.run_scene(x=0,y=0,bx=.15,by=0)
        self.assertTrue(d['result']['success'])
        self.assertEqual(d['result']['path'],0)
        self.assertTrue(all(f['left']==0 and f['right']==0 for f in d['frames']))

    def test_stop_button_stops_real_firmware_and_latches(self):
        d=self.run_scene(x=0,y=0,bx=1,by=0,stop_at=15)
        self.assertFalse(d['result']['success'])
        self.assertTrue(any('STOP_LATCHED' in l['message'] for l in d['logs']))
        f=d['frames'][-1]
        self.assertEqual((f['left'],f['right']),(0,0))
        self.assertLess(f['t'],15.1)

    def test_invalid_reply_does_not_falsely_pass(self):
        d=self.run_scene(x=0,y=0,bx=.15,by=0,corrupt_reply=1)
        self.assertFalse(d['result']['success'])
        self.assertTrue(any('CRC or length failed' in l['message'] for l in d['logs']))
        self.assertFalse(d['frames'][-1]['payload_verified'])
        self.assertEqual(d['result']['path'],0)
        self.assertFalse(any(f['buzzer'] for f in d['frames']))

    def test_rotating_entire_world_preserves_result(self):
        d=self.run_scene(x=0,y=0,heading=90,bx=0,by=1)
        self.assertTrue(d['result']['success'])
        self.assertAlmostEqual(self.baseline['result']['distance'],d['result']['distance'],places=5)
        self.assertAlmostEqual(self.baseline['result']['path'],d['result']['path'],places=5)

    def test_side_beacon_requires_real_turns_and_returns(self):
        d=self.run_scene(x=0,y=0,bx=1,by=1)
        self.assertTrue(d['result']['success'])
        self.assertTrue(any(f['left']*f['right']<0 for f in d['frames']))
        self.assertTrue(any(f['left']+f['right']<-.01 for f in d['frames']))
        self.assertTrue(any('DIRECTION SEARCH' in l['message'] for l in d['logs']))

    def test_world_coordinates_never_leak_into_controller(self):
        # Translation of the entire world leaves every sensor reading and decision unchanged.
        d=self.run_scene(x=4,y=-3,bx=5,by=-3)
        self.assertEqual(self.baseline['logs'],d['logs'])
        self.assertEqual(self.baseline['result']['t'],d['result']['t'])

    def test_another_branch_executes_its_own_algorithm(self):
        ref='origin/Alvin'
        d=s.run_simulation(s.DEFAULT_REPO,ref,self.sdk,dict(x=0,y=0,bx=1,by=0),save=False)
        self.assertTrue(d['result']['success'])
        self.assertNotEqual(d['metadata']['commit'],self.commit)
        self.assertGreater(d['result']['path'],self.baseline['result']['path']*2)

    def test_saved_working_tree_edits_are_actually_compiled(self):
        # Introduce an unmistakable change in a temporary repository. The host
        # must execute it, rather than always running a cached main algorithm.
        source,_=s.source_snapshot(s.DEFAULT_REPO,self.commit)
        import shutil
        with tempfile.TemporaryDirectory(prefix='beacon-sim-test-') as tmp:
            repo=Path(tmp)
            for folder in s.SOURCE_FOLDERS:
                if (source/folder).exists():
                    shutil.copytree(source/folder,repo/folder,ignore=shutil.ignore_patterns('build'))
            subprocess.run(['git','init','-q',repo],check=True)
            subprocess.run(['git','-C',repo,'add','.'],check=True)
            subprocess.run(['git','-C',repo,'-c','user.name=Simulator test','-c','user.email=simulator@example.invalid',
                            'commit','-qm','Test fixture'],check=True)
            nav=repo/'getting_there/getting_there.c'
            original=nav.read_text()
            self.assertIn('static bool approach_beacon(void)\n{',original)
            nav.write_text(original.replace('static bool approach_beacon(void)\n{',
                'static bool approach_beacon(void)\n{\n    report_printf("SIM TEST: injected stop\\n"); return false;',1))
            payload=repo/'getting_there/payload_client.h'
            if '#define AP1_CLOSE_RSSI_DBM' not in payload.read_text():
                payload=repo/'getting_there/payload_client.c'
            payload_original=payload.read_text()
            modified,count=re.subn(r'(#define (?:AP1_CLOSE_RSSI_DBM|CLOSE_RSSI_DBM))\s+\(-?\d+\)',
                                   r'\1 (-30)',payload_original,count=1)
            self.assertEqual(count,1,'The test must change the real threshold definition')
            payload.write_text(modified)
            d=s.run_simulation(repo,'working-tree',self.sdk,dict(x=0,y=0,bx=1,by=0),save=False)
            self.assertFalse(d['result']['success'])
            self.assertEqual(d['result']['path'],0)
            self.assertTrue(any('SIM TEST: injected stop' in l['message'] for l in d['logs']))
            self.assertEqual(d['frames'][0]['ble_threshold'],-30)
            self.assertEqual(d['frames'][0]['ble'],self.baseline['frames'][0]['ble'],
                             'Branch threshold changes must not change the environment signal')
            # Now edit only the beacon, and let the original car run normally.
            nav.write_text(original)
            payload.write_text(payload_original)
            beacon=repo/self.baseline['metadata']['beacon_source']
            self.assertIn('Reply file from beacon AP%d',beacon.read_text())
            beacon.write_text(beacon.read_text().replace('Reply file from beacon AP%d',
                                                        'Changed branch beacon AP%d'))
            d=s.run_simulation(repo,'working-tree',self.sdk,dict(x=0,y=0,bx=.15,by=0),save=False)
            self.assertTrue(d['result']['success'])
            self.assertEqual(d['metadata']['beacon_mode'],'branch-firmware')
            self.assertTrue(any('AP1 payload: Changed branch beacon AP1' in l['message'] for l in d['logs']))
            self.assertEqual(d['frames'][-1]['tx_bytes'],len('Changed branch beacon AP1'))

    def test_two_decimal_positions_and_environment_values(self):
        scene=dict(x=-1.23,y=2.34,heading=12.34,bx=-1.13,by=2.34,
                   speed=.77,wheelbase=.15,wifi_ref=-50.12,exponent=2.01,ble_radius=.27,stop_at=-1)
        d=self.run_scene(**scene)
        for key,value in scene.items(): self.assertEqual(d['scenario'][key],value)
        self.assertTrue(d['result']['success'])
        self.assertEqual((d['frames'][0]['x'],d['frames'][0]['y']),(-1.23,2.34))
        # The browser validation increment must match drag's two-decimal output.
        from html.parser import HTMLParser
        class Inputs(HTMLParser):
            def handle_starttag(self,tag,attrs):
                a=dict(attrs)
                if tag=='input' and a.get('type')=='number':
                    self.inputs.append(a)
        parser=Inputs(); parser.inputs=[]
        parser.feed((s.ROOT/'dist/index.html').read_text())
        self.assertGreaterEqual(len(parser.inputs),11)
        for a in parser.inputs: self.assertEqual(a.get('step'),'0.01',a.get('id'))

    def test_invalid_parameters_are_rejected(self):
        for bad in [dict(x=float('nan')),dict(speed=-1),dict(wheelbase=0),dict(ble_radius=0)]:
            with self.assertRaises(ValueError): s.validate_scenario(bad)

if __name__=='__main__': unittest.main(verbosity=2)
