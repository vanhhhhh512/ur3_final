"""Bounded HTTP skill planning; the response is untrusted domain data."""
import json
import os
import urllib.error
import urllib.request
from .contracts import Rejected, VerifiedPlan


def strict_document(text):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise Rejected(f'Duplicate JSON field: {key}')
            result[key] = value
        return result
    try:
        return json.loads(text, object_pairs_hook=unique, parse_constant=lambda x: (_ for _ in ()).throw(Rejected('Non-finite JSON number')))
    except (ValueError, TypeError) as error:
        raise Rejected('Planner response is not a single strict JSON document') from error


class SkillModel:
    def __init__(self, layout, endpoint=None, model=None, token=None):
        self.layout = layout
        self.endpoint = (endpoint or os.getenv('NINEROUTER_BASE_URL', 'http://127.0.0.1:11434/v1')).rstrip('/')
        self.model = model or os.getenv('NINEROUTER_MODEL', 'qwen2.5:3b')
        self.token = token or os.getenv('NINEROUTER_API_KEY', 'local-ollama')
        self.last_attempts = []

    def propose(self, request_text, goal, observation):
        occupant = observation.occupants[goal.destination]
        blocker = occupant if occupant not in (None, goal.item) else None
        schema = {'steps': [{'op': 'take', 'item': 'blue_cube'}, {'op': 'stage', 'item': 'blue_cube'},
                            {'op': 'take', 'item': 'red_cube'}, {'op': 'deposit', 'item': 'red_cube', 'zone': 'zone_b'}, {'op': 'park'}]}
        if blocker is None:
            schema = {'steps': [{'op': 'take', 'item': 'green_cube'},
                                {'op': 'deposit', 'item': 'green_cube', 'zone': 'zone_c'}, {'op': 'park'}]}
        system = ('You select symbolic skills for a one-block-at-a-time robot. Return exactly one JSON object with key steps. '
                  'take requires empty hand; stage moves ONLY the destination blocker to a free table slot selected by software; '
                  'deposit places ONLY the requested item in the requested empty zone; park returns home with an empty hand. '
                  'If the destination contains another item, first take and stage that blocker, then take and deposit the requested item. '
                  'Read destination_occupant and blocker_to_stage in the user message. '
                  'Never stage the requested item. A blocker must be taken and staged before taking the requested item. '
                  'If blocker_to_stage is null, omit stage and take/deposit only the requested item. '
                  'Do not provide coordinates, joint commands, trajectories, extra fields, markdown or prose. '
                  'Finish every task with park. Example for an ' + ('occupied' if blocker else 'empty') +
                  ' destination (adapt item and zone to the actual goal): ' + json.dumps(schema))
        body = {'model': self.model, 'temperature': 0, 'stream': False, 'response_format': {'type': 'json_object'},
                'messages': [{'role': 'system', 'content': system}, {'role': 'user', 'content': json.dumps({'request': request_text, 'goal': {'item': goal.item, 'zone': goal.destination}, 'destination_occupant': occupant, 'blocker_to_stage': blocker, 'scene': observation.context()})}]}
        self.last_attempts = []
        for attempt in range(3):
            message = urllib.request.Request(self.endpoint + '/chat/completions', data=json.dumps(body).encode(),
                                             headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + self.token})
            try:
                with urllib.request.urlopen(message, timeout=120) as response:
                    raw = response.read(1048577)
                if len(raw) > 1048576:
                    raise Rejected('Planner reply exceeds size limit')
                reply = strict_document(raw.decode())
                text = reply['choices'][0]['message']['content']
                if not isinstance(text, str):
                    raise TypeError('Missing planner text')
            except (OSError, UnicodeError, KeyError, IndexError, TypeError) as error:
                raise Rejected('Language-model transport or response envelope failed') from error
            try:
                plan = VerifiedPlan.check(strict_document(text), observation, goal, self.layout)
            except Rejected as error:
                self.last_attempts.append({'text': text, 'accepted': False, 'error': str(error)})
                if attempt == 2:
                    raise
                body['messages'] += [{'role': 'assistant', 'content': text},
                                     {'role': 'user', 'content': 'Validator rejected this plan: ' + str(error) +
                                      '. No motion was executed. Revise the entire JSON plan for the original goal. '
                                      'blocker_to_stage=' + json.dumps(blocker) + '; stage is forbidden when it is null.'}]
            else:
                self.last_attempts.append({'text': text, 'accepted': True})
                return plan, text
