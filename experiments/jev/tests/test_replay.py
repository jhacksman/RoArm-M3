from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from jev_replay.core import ContractError, Observation, Pending, ShadowGate, build_request
from jev_replay.__main__ import replay, reject_constant, unique_object

FIXTURE = Path(__file__).resolve().parents[1] / 'examples/synthetic-replay.json'


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads(FIXTURE.read_text())
        case = self.doc['cases'][0]
        self.data = case['submitted']
        self.obs = Observation.parse(self.data)
        self.pending = Pending.create('request-1', self.obs, 1020, 1200, 300)
        self.response = case['attempts'][0]['response']

    def reason(self, current=None, response=None, now=1100, pending=None):
        return ShadowGate().evaluate(pending or self.pending, current or self.obs,
                    self.response if response is None else response, now).reason

    def test_fixture(self):
        results = replay(self.doc)
        self.assertEqual(len(results), 7)
        self.assertTrue(all(item['matches_expectation'] for item in results))

    def test_request_filters_infeasible_actions(self):
        self.data['facts']['tracking_valid'] = False
        pending = Pending.create('r',Observation.parse(self.data),1020,1200,300)
        request = build_request(pending)
        self.assertEqual(set(request['questions']['next_action']['criteria']), {'observe'})
        self.assertEqual(request['state']['mode'], 'offline_shadow_only')

    def test_snapshot_is_immutable(self):
        self.data['candidates'][0]['requires'].append('new_fact')
        self.data['facts']['tracking_valid'] = False
        self.assertEqual(self.reason(), 'validated')

    def test_duplicate_and_epoch_reuse(self):
        gate = ShadowGate()
        self.assertEqual(gate.evaluate(self.pending,self.obs,self.response,1100).reason,'validated')
        self.assertEqual(gate.evaluate(self.pending,self.obs,self.response,1110).reason,'duplicate_request')
        second = replace(self.pending,request_id='request-2')
        self.assertEqual(gate.evaluate(second,self.obs,self.response,1120).reason,'epoch_already_used')

    def test_rejected_response_consumes_request(self):
        gate = ShadowGate()
        self.assertEqual(gate.evaluate(self.pending,self.obs,{},1100).status,'rejected')
        self.assertEqual(gate.evaluate(self.pending,self.obs,self.response,1110).reason,'duplicate_request')

    def test_context_and_state_changes(self):
        for change, reason in [({'arm':'other'},'context_changed'),({'session':'other'},'context_changed'),
                ({'epoch':'other'},'epoch_changed'),({'calibration':'other'},'calibration_changed'),
                ({'task':'other'},'state_changed'),({'labels':(('tracking','lost'),)},'state_changed')]:
            with self.subTest(change=change):
                self.assertEqual(self.reason(current=replace(self.obs,**change)),reason)

    def test_candidate_same_id_changed_parameters(self):
        for field,value in [('target','B'),('lease_ms',10000),('skill','grip')]:
            candidates=(replace(self.obs.candidates[0],**{field:value}),self.obs.candidates[1])
            self.assertEqual(self.reason(current=replace(self.obs,candidates=candidates)),
                             'candidate_no_longer_feasible')

    def test_time_boundaries(self):
        self.assertEqual(self.reason(now=1200),'deadline_expired')
        self.assertEqual(self.reason(now=1010),'response_before_request')
        self.assertEqual(self.reason(current=replace(self.obs,captured_ms=1101)),'stale_live_observation')
        self.assertEqual(self.reason(current=replace(self.obs,captured_ms=999)),'capture_regressed')
        pending=replace(self.pending,deadline_ms=2000)
        self.assertEqual(self.reason(pending=pending,now=1301),'stale_live_observation')
        self.assertEqual(self.reason(pending=pending,now=1301,current=replace(self.obs,captured_ms=1300)),
                         'stale_request_observation')
        gate=ShadowGate(); gate.evaluate(self.pending,self.obs,self.response,1100)
        self.assertEqual(gate.evaluate(self.pending,self.obs,self.response,1099).reason,'clock_regressed')

    def test_invalid_probabilities(self):
        for value in [float('nan'),float('inf'),-0.1,1.1,10**1000,True,'0.75',None]:
            response=deepcopy(self.response)
            response['answers']['next_action']['probabilities']['align-A']=value
            self.assertEqual(self.reason(response=response),'invalid_probability')

    def test_response_shape(self):
        for value in [None,[],{}, {'model':'x','answers':{}}, {'model':'x','answers':[]}]:
            self.assertEqual(ShadowGate().evaluate(self.pending,self.obs,value,1100).status,'rejected')
        for change,reason in [({'choice':'unknown'},'unknown_choice'),({'choice':'observe'},'choice_not_maximum'),
                ({'probabilities':{'align-A':1}},'invalid_distribution_keys'),
                ({'confidence':False},'invalid_probability')]:
            response=deepcopy(self.response); response['answers']['next_action'].update(change)
            self.assertEqual(self.reason(response=response),reason)

    def test_invalid_input_contracts(self):
        for modify in [lambda d:d.update(captured_ms=True),lambda d:d.update(candidates=[]),
                lambda d:d['candidates'].append(deepcopy(d['candidates'][0])),
                lambda d:d['facts'].update(tracking_valid='true'),
                lambda d:d['candidates'][0].update(lease_ms=0),
                lambda d:d['candidates'][0].update(requires=['missing'])]:
            data=deepcopy(self.data); modify(data)
            with self.assertRaises(ContractError): Observation.parse(data)
        for issued,deadline,age in [(999,1200,300),(1020,1020,300),(1020,1200,0)]:
            with self.assertRaises(ContractError): Pending.create('r',self.obs,issued,deadline,age)

    def test_strict_json_and_synthetic_only(self):
        for raw in ['{"a":1,"a":2}','{"a":NaN}']:
            with self.assertRaises(ContractError):
                json.loads(raw,parse_constant=reject_constant,object_pairs_hook=unique_object)
        self.doc['data_kind']='real'
        with self.assertRaises(ContractError): replay(self.doc)

    def test_cli_exit_codes(self):
        def run(path):
            return subprocess.run([sys.executable,'-m','jev_replay',str(path)],capture_output=True,text=True)
        result=run(FIXTURE)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(len(result.stdout.splitlines()),7)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'fixture.json'
            self.doc['cases'][0]['attempts'][0]['expected_reason']='wrong'
            path.write_text(json.dumps(self.doc))
            self.assertEqual(run(path).returncode,1)
            path.write_text('{"private":"SENSITIVE_TEST_MARKER"}')
            result=run(path)
            self.assertEqual(result.returncode,2)
            self.assertNotIn('SENSITIVE_TEST_MARKER',result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
