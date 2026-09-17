"""Validate a Jev Choice against immutable snapshots in a synthetic replay.

All timestamps are integer milliseconds in one host/session monotonic domain.
This module grants no motor authority and implements no physical safety system.
"""
from dataclasses import dataclass
import math


class ContractError(ValueError):
    """Malformed replay input or response (message contains no input values)."""


def require(condition, reason):
    if not condition:
        raise ContractError(reason)


def text(value):
    require(isinstance(value, str) and bool(value.strip()), 'invalid_text')
    return value


def millis(value):
    require(type(value) is int and value >= 0, 'invalid_time')
    return value


def probability(value):
    require(type(value) in (int, float), 'invalid_probability')
    require(0 <= value <= 1 and math.isfinite(value), 'invalid_probability')
    return value


def fields(value, expected):
    require(isinstance(value, dict) and set(value) == set(expected.split()),
            'invalid_fields')


@dataclass(frozen=True)
class Candidate:
    id: str
    skill: str
    target: str
    description: str
    requires: tuple
    lease_ms: int

    @classmethod
    def parse(cls, data):
        fields(data, 'id skill target description requires lease_ms')
        require(isinstance(data['requires'], list) and data['requires'],
                'missing_preconditions')
        checks = tuple(text(item) for item in data['requires'])
        require(len(checks) == len(set(checks)), 'duplicate_preconditions')
        lease = millis(data['lease_ms'])
        require(lease > 0, 'invalid_lease')
        return cls(*(text(data[key]) for key in ('id', 'skill', 'target', 'description')),
                   checks, lease)


@dataclass(frozen=True)
class Observation:
    session: str
    arm: str
    epoch: str
    calibration: str
    captured_ms: int
    task: str
    labels: tuple
    facts: tuple
    candidates: tuple

    @classmethod
    def parse(cls, data):
        fields(data, 'session arm epoch calibration captured_ms task labels facts candidates')
        require(isinstance(data['labels'], dict), 'invalid_labels')
        labels = tuple(sorted((text(k), text(v)) for k, v in data['labels'].items()))
        require(isinstance(data['facts'], dict), 'invalid_facts')
        facts = []
        for key, value in data['facts'].items():
            require(type(value) is bool, 'invalid_fact')
            facts.append((text(key), value))
        require(isinstance(data['candidates'], list), 'invalid_candidates')
        candidates = tuple(Candidate.parse(item) for item in data['candidates'])
        require(candidates and len({c.id for c in candidates}) == len(candidates),
                'empty_or_duplicate_candidates')
        for candidate in candidates:
            require(all(key in data['facts'] for key in candidate.requires),
                    'unknown_precondition')
        return cls(*(text(data[key]) for key in ('session', 'arm', 'epoch', 'calibration')),
                   millis(data['captured_ms']), text(data['task']), labels,
                   tuple(sorted(facts)), candidates)

    def feasible(self):
        facts = dict(self.facts)
        return tuple(c for c in self.candidates if all(facts[key] for key in c.requires))


@dataclass(frozen=True)
class Pending:
    request_id: str
    observation: Observation
    issued_ms: int
    deadline_ms: int
    max_age_ms: int
    candidates: tuple

    @classmethod
    def create(cls, request_id, observation, issued_ms, deadline_ms, max_age_ms):
        text(request_id)
        for value in (issued_ms, deadline_ms, max_age_ms):
            millis(value)
        require(max_age_ms > 0 and deadline_ms > issued_ms, 'invalid_window')
        require(0 <= issued_ms - observation.captured_ms <= max_age_ms,
                'invalid_capture_age')
        candidates = observation.feasible()
        require(bool(candidates), 'no_feasible_candidates')
        return cls(request_id, observation, issued_ms, deadline_ms, max_age_ms, candidates)


def build_request(pending, model='jev-latest'):
    """Build the documented Choice shape without sending it or reading secrets."""
    obs = pending.observation
    return {
        'model': text(model),
        'state': {
            'mode': 'offline_shadow_only', 'task': obs.task,
            'observation_epoch': obs.epoch, 'arm': obs.arm,
            'labels': dict(obs.labels), 'facts': dict(obs.facts),
            'candidate_actions': [
                {'id': c.id, 'skill': c.skill, 'target': c.target,
                 'meaning': c.description, 'lease_ms': c.lease_ms}
                for c in pending.candidates],
        },
        'questions': {'next_action': {
            'type': 'choice',
            'instructions': 'Choose one available candidate that progresses the task. '
                            'Use the observations and candidate meanings. Prefer re-observation '
                            'when evidence is insufficient. This is shadow evaluation only.',
            'criteria': {c.id: c.description for c in pending.candidates},
        }},
    }


def parse_choice(response, candidates):
    require(isinstance(response, dict), 'invalid_response')
    text(response.get('model'))
    answers = response.get('answers')
    require(isinstance(answers, dict), 'invalid_answers')
    answer = answers.get('next_action')
    require(isinstance(answer, dict) and answer.get('type') == 'choice', 'invalid_answer')
    choice = text(answer.get('choice'))
    ids = {c.id for c in candidates}
    require(choice in ids, 'unknown_choice')
    distribution = answer.get('probabilities')
    require(isinstance(distribution, dict) and set(distribution) == ids,
            'invalid_distribution_keys')
    values = [probability(v) for v in distribution.values()]
    require(math.isclose(sum(values), 1.0, rel_tol=0, abs_tol=1e-6),
            'invalid_distribution_sum')
    require(distribution[choice] == max(values), 'choice_not_maximum')
    probability(answer.get('confidence'))
    return choice


@dataclass(frozen=True)
class Verdict:
    status: str
    reason: str
    candidate_id: str | None = None


class ShadowGate:
    """Single-process, sequential, in-memory replay gate; never an executor.

    Each response attempt consumes its request ID, including rejections. Each
    accepted epoch consumes that arm/session revision to prevent duplicate actions.
    Production persistence, concurrency and physical command ownership are absent.
    """
    def __init__(self):
        self.attempted = set()
        self.accepted_epochs = set()
        self.last_now = None

    def evaluate(self, pending, current, response, now_ms):
        try:
            millis(now_ms)
            require(self.last_now is None or now_ms >= self.last_now, 'clock_regressed')
            self.last_now = now_ms
            require(pending.request_id not in self.attempted, 'duplicate_request')
            self.attempted.add(pending.request_id)
            old = pending.observation
            require(now_ms >= pending.issued_ms, 'response_before_request')
            require(now_ms < pending.deadline_ms, 'deadline_expired')
            require((current.session, current.arm) == (old.session, old.arm), 'context_changed')
            require(current.calibration == old.calibration, 'calibration_changed')
            require(current.epoch == old.epoch, 'epoch_changed')
            require(current.task == old.task and current.labels == old.labels, 'state_changed')
            require(0 <= now_ms - current.captured_ms <= pending.max_age_ms,
                    'stale_live_observation')
            require(current.captured_ms >= old.captured_ms, 'capture_regressed')
            require(now_ms - old.captured_ms <= pending.max_age_ms, 'stale_request_observation')
            epoch = (old.session, old.arm, old.epoch)
            require(epoch not in self.accepted_epochs, 'epoch_already_used')
            choice = parse_choice(response, pending.candidates)
            selected = next(c for c in pending.candidates if c.id == choice)
            require(selected in current.feasible(), 'candidate_no_longer_feasible')
            self.accepted_epochs.add(epoch)
            return Verdict('accepted_for_shadow', 'validated', choice)
        except ContractError as error:
            return Verdict('rejected', str(error))
