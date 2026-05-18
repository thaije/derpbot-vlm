import pytest
from agent.vlm_client import NavigationAction, VLMResult


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