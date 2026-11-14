"""Replay synthetic fixtures locally, with no network or motor interface."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from .core import ContractError, Observation, Pending, ShadowGate, require


def reject_constant(_):
    raise ContractError('nonfinite_json')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate_json_key')
        result[key] = value
    return result


def replay(document):
    require(isinstance(document, dict) and type(document.get('schema_version')) is int
            and document['schema_version'] == 1, 'unsupported_schema')
    require(document.get('data_kind') == 'synthetic', 'synthetic_only')
    require(isinstance(document.get('cases'), list), 'invalid_cases')
    reports = []
    for case in document['cases']:
        require(isinstance(case, dict), 'invalid_case')
        pending = Pending.create(case['request_id'], Observation.parse(case['submitted']),
                                 case['issued_ms'], case['deadline_ms'], case['max_age_ms'])
        gate = ShadowGate()
        require(isinstance(case['attempts'], list) and case['attempts'], 'invalid_attempts')
        for attempt in case['attempts']:
            verdict = gate.evaluate(pending, Observation.parse(attempt['current']),
                                    attempt['response'], attempt['now_ms'])
            require(isinstance(attempt['expected_reason'], str), 'invalid_expectation')
            reports.append({'case':case['request_id'], **asdict(verdict),
                            'matches_expectation':verdict.reason == attempt['expected_reason']})
    require(bool(reports), 'empty_replay')
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixture', type=Path)
    args = parser.parse_args()
    try:
        document = json.loads(args.fixture.read_text(), parse_constant=reject_constant,
                              object_pairs_hook=unique_object)
        reports = replay(document)
    except (OSError, ValueError, TypeError, KeyError):
        print('Invalid or unreadable synthetic replay fixture.', file=sys.stderr)
        return 2
    for report in reports:
        print(json.dumps(report, allow_nan=False))
    return 0 if all(r['matches_expectation'] for r in reports) else 1


if __name__ == '__main__':
    sys.exit(main())
