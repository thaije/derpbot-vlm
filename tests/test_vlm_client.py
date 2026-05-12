import pytest
from agent.vlm_client import _parse_response, VLMResult


class TestParseResponse:
    def test_valid_json(self):
        text = '{"action": "forward", "reasoning": "I see a clear path", "target_visible": true}'
        result = _parse_response(text)
        assert result is not None
        assert result.action == "forward"
        assert result.reasoning == "I see a clear path"
        assert result.target_visible is True

    def test_json_with_surrounding_text(self):
        text = 'Here is my response: {"action": "left", "reasoning": "turning", "target_visible": false} and that is it.'
        result = _parse_response(text)
        assert result is not None
        assert result.action == "left"

    def test_invalid_action_defaults_to_stop(self):
        text = '{"action": "jump", "reasoning": "test", "target_visible": false}'
        result = _parse_response(text)
        assert result is not None
        assert result.action == "stop"

    def test_missing_action_defaults_to_stop(self):
        text = '{"reasoning": "nothing here"}'
        result = _parse_response(text)
        assert result is not None
        assert result.action == "stop"

    def test_all_valid_actions(self):
        for action in ["forward", "backward", "left", "right", "stop"]:
            text = f'{{"action": "{action}", "reasoning": "test", "target_visible": true}}'
            result = _parse_response(text)
            assert result.action == action

    def test_case_insensitive_action(self):
        text = '{"action": "FORWARD", "reasoning": "test", "target_visible": true}'
        result = _parse_response(text)
        assert result.action == "forward"

    def test_no_json_returns_none(self):
        text = "I think we should go forward"
        result = _parse_response(text)
        assert result is None

    def test_invalid_json_returns_none(self):
        text = "{not valid json}"
        result = _parse_response(text)
        assert result is None

    def test_target_visible_defaults_false(self):
        text = '{"action": "forward", "reasoning": "moving"}'
        result = _parse_response(text)
        assert result is not None
        assert result.target_visible is False