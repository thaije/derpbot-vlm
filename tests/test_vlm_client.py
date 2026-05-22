import pytest
from agent.vlm_client import NavigationAction, VLMResult, _parse_vlm_response


class TestNavigationAction:
    def test_valid_json_all_actions(self):
        for action in ["forward", "backward", "left", "right", "stop"]:
            raw = f'{{"action": "{action}", "reasoning": "test", "target_visible": false}}'
            parsed = NavigationAction.model_validate_json(raw)
            assert parsed.action == action
            assert parsed.target_visible is False

    def test_target_visible_true(self):
        raw = '{"action": "forward", "reasoning": "drill visible", "target_visible": true}'
        parsed = NavigationAction.model_validate_json(raw)
        assert parsed.target_visible is True
        assert parsed.action == "forward"

    def test_target_visible_false(self):
        raw = '{"action": "left", "reasoning": "no drill visible", "target_visible": false}'
        parsed = NavigationAction.model_validate_json(raw)
        assert parsed.target_visible is False

    def test_action_must_be_valid_literal(self):
        with pytest.raises(Exception):
            NavigationAction.model_validate_json('{"action": "jump", "reasoning": "test", "target_visible": false}')

    def test_missing_field_rejected(self):
        with pytest.raises(Exception):
            NavigationAction.model_validate_json('{"action": "forward"}')

    def test_extra_fields_ignored(self):
        raw = '{"action": "forward", "reasoning": "test", "target_visible": false, "extra": "ignored"}'
        parsed = NavigationAction.model_validate_json(raw)
        assert parsed.action == "forward"


class TestVLMResult:
    def test_vlm_result_fields(self):
        result = VLMResult(action="forward", reasoning="clear path", target_visible=False)
        assert result.action == "forward"
        assert result.reasoning == "clear path"
        assert result.target_visible is False

    def test_vlm_result_from_parsed(self):
        raw = '{"action": "right", "reasoning": "wall ahead", "target_visible": false}'
        parsed = NavigationAction.model_validate_json(raw)
        result = VLMResult(action=parsed.action, reasoning=parsed.reasoning, target_visible=parsed.target_visible)
        assert result.action == "right"
        assert result.target_visible is False


class TestParseVLMResponse:
    def test_clean_json(self):
        result = _parse_vlm_response('{"action": "forward", "reasoning": "clear view", "target_visible": true}')
        assert result.target_visible is True
        assert result.action == "forward"

    def test_json_in_code_fence(self):
        result = _parse_vlm_response('```json\n{"action": "forward", "reasoning": "no target", "target_visible": false}\n```')
        assert result.target_visible is False

    def test_json_embedded_in_text(self):
        result = _parse_vlm_response('I analyzed the image. {"target_visible": true, "reasoning": "fire extinguisher on wall", "action": "forward"} The target is present.')
        assert result.target_visible is True
        assert "fire extinguisher" in result.reasoning

    def test_partial_json_missing_action(self):
        result = _parse_vlm_response('{"target_visible": false, "reasoning": "nothing here"}')
        assert result.target_visible is False
        assert result.action == "forward"

    def test_free_text_visible(self):
        result = _parse_vlm_response("I can see the target fire extinguisher on the wall to the left.")
        assert result.target_visible is True

    def test_free_text_not_visible(self):
        result = _parse_vlm_response("The fire extinguisher is not visible in this image.")
        assert result.target_visible is False

    def test_free_text_no_target(self):
        result = _parse_vlm_response("No target object can be seen in the current frame.")
        assert result.target_visible is False

    def test_empty_returns_none(self):
        assert _parse_vlm_response("") is None

    def test_unparseable_defaults_not_visible(self):
        result = _parse_vlm_response("The image shows a corridor with walls and a door.")
        assert result.target_visible is False