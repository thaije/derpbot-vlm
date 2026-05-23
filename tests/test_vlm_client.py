import pytest
from agent.vlm_client import NavigationDecision, VLMResult, _parse_vlm_response


class TestNavigationDecision:
    def test_valid_minimal(self):
        raw = '{"target_visible": false, "heading": "center", "drive_distance_m": 1.0, "reason": "open path"}'
        parsed = NavigationDecision.model_validate_json(raw)
        assert parsed.target_visible is False
        assert parsed.heading == "center"
        assert parsed.drive_distance_m == 1.0
        assert parsed.target_bbox is None

    def test_with_bbox(self):
        raw = '{"target_visible": true, "target_bbox": [10, 20, 100, 200], "heading": "left", "drive_distance_m": 0.5, "reason": "target on left"}'
        parsed = NavigationDecision.model_validate_json(raw)
        assert parsed.target_visible is True
        assert parsed.target_bbox == [10, 20, 100, 200]
        assert parsed.heading == "left"

    def test_distance_bounds(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "center", "drive_distance_m": 3.0, "reason": "x"}')
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "center", "drive_distance_m": -0.1, "reason": "x"}')

    def test_heading_must_be_literal(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json(
                '{"target_visible": false, "heading": "forward", "drive_distance_m": 1.0, "reason": "x"}')

    def test_missing_field_rejected(self):
        with pytest.raises(Exception):
            NavigationDecision.model_validate_json('{"target_visible": false, "heading": "center"}')


class TestVLMResult:
    def test_fields(self):
        r = VLMResult(target_visible=True, heading="right", drive_distance_m=1.2,
                      target_bbox=[1, 2, 3, 4], reason="ok")
        assert r.target_visible is True
        assert r.heading == "right"
        assert r.drive_distance_m == 1.2
        assert r.target_bbox == [1, 2, 3, 4]


class TestParseVLMResponse:
    def test_clean_json(self):
        r = _parse_vlm_response(
            '{"target_visible": true, "target_bbox": [5,6,7,8], "heading": "center", "drive_distance_m": 1.5, "reason": "ok"}')
        assert r.target_visible is True
        assert r.heading == "center"
        assert r.drive_distance_m == 1.5
        assert r.target_bbox == [5, 6, 7, 8]

    def test_json_in_code_fence(self):
        raw = '```json\n{"target_visible": false, "heading": "left", "drive_distance_m": 0.5, "reason": "wall"}\n```'
        r = _parse_vlm_response(raw)
        assert r.target_visible is False
        assert r.heading == "left"
        assert r.drive_distance_m == 0.5

    def test_distance_clamp_above(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "center", "drive_distance_m": 5.0, "reason": "x"}')
        assert r.drive_distance_m == 2.0

    def test_distance_clamp_below(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "center", "drive_distance_m": -2.0, "reason": "x"}')
        assert r.drive_distance_m == 0.0

    def test_heading_coercion_uppercase(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "LEFT", "drive_distance_m": 0.8, "reason": "x"}')
        assert r.heading == "left"

    def test_heading_unknown_defaults_center(self):
        r = _parse_vlm_response(
            '{"target_visible": false, "heading": "forward", "drive_distance_m": 0.8, "reason": "x"}')
        assert r.heading == "center"

    def test_free_text_visible(self):
        r = _parse_vlm_response("I can see the target fire extinguisher on the wall to the left, turn left.")
        assert r.target_visible is True
        assert r.heading == "left"

    def test_free_text_not_visible(self):
        r = _parse_vlm_response("The fire extinguisher is not visible in this image. Go right.")
        assert r.target_visible is False
        assert r.heading == "right"

    def test_empty_returns_none(self):
        assert _parse_vlm_response("") is None

    def test_unparseable_defaults_safe(self):
        r = _parse_vlm_response("The image shows a corridor with walls and a door.")
        assert r.target_visible is False
        assert r.heading == "center"
        assert r.drive_distance_m == 0.5
        assert r.target_bbox is None
