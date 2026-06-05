import base64
import io
import json
import logging
import re
from dataclasses import dataclass
from typing import Literal, Optional

from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)

MAX_IMAGE_DIM = 768
JPEG_QUALITY = 90

SYSTEM_PROMPT = (
    "You are steering a robot through an indoor environment using a camera view."
    "\nFor every image, you must decide TWO things in one JSON response:"
    "\n  1. DETECT — is the target object visible?"
    "\n     Scan the WHOLE image carefully — including the floor, corners, edges,"
    "\n     along walls, and partly-occluded areas behind other objects."
    "\n     Targets may appear SMALL (covering only a few percent of the image),"
    "\n     LOW-CONTRAST (similar colour/material to the background), or PARTLY"
    "\n     HIDDEN. The target name is a descriptive label, often underscored;"
    "\n     accept any object that plausibly fits the description — synonyms,"
    "\n     variants, partial views and side-on views all count."
    "\n     Set target_visible=true if you see ANY object that could match."
    "\n     If true, you MUST also fill target_bbox=[x1,y1,x2,y2] tightly around"
    "\n     the object (top-left = (0,0))."
    "\n     NEVER report target_visible=true without a bounding box."
    "\n  2. NAVIGATE — pick the next heading and how far to drive there."
    "\n     heading ∈ {\"left\" (~30° left), \"center\" (straight), \"right\" (~30° right)}."
    "\n     drive_distance_m ∈ [0.0, 2.0]. 0.0 = stop and rescan."
    "\nGuidelines:"
    "\n  - When the target is visible, drive toward it. Pick the heading that points at it; pick a"
    "\n    distance close to how far away it appears."
    "\n  - When the target is NOT visible, pick a heading that leads into open, unexplored space."
    "\n  - Avoid walls/obstacles. Use shorter distances (0.3-0.8 m) in cluttered or uncertain scenes;"
    "\n    longer (1.0-2.0 m) when the path ahead is clearly open."
    "\n  - If you are facing a wall and no good option, choose left or right with a small distance."
    "\nReply with valid JSON ONLY, matching this schema:"
    "\n{\"target_visible\": bool, \"target_bbox\": [x1,y1,x2,y2] or null,"
    " \"heading\": \"left\"|\"center\"|\"right\", \"drive_distance_m\": float, \"reason\": str}"
)


class NavigationDecision(BaseModel):
    target_visible: bool
    target_bbox: Optional[list[int]] = None
    heading: Literal["left", "center", "right"]
    drive_distance_m: float = Field(ge=0.0, le=2.0)
    reason: str


NAV_DECISION_SCHEMA = NavigationDecision.model_json_schema()


# ── Verifier (skeptical second call, #10) ───────────────────────────────────
# Given a tightly cropped image of a candidate region, decide whether it
# really shows the target. Deliberately framed in the OPPOSITE direction from
# the detection prompt (which rewards aggressive "see anything that fits"
# behaviour). The verifier is asked to enumerate evidence both for AND against
# so the model commits to a calibrated judgement instead of agreeing reflexively.

VERIFIER_SYSTEM_PROMPT = (
    "You are a strict visual verifier. A previous perception step flagged the"
    " attached image crop as containing a specific target object. Your job is"
    " to confirm or reject that claim."
    "\nDefault to REJECT. Most regions in indoor scenes are NOT the target —"
    " walls, floor seams, brick edges, cables and shadows all create"
    " confusing shapes. Only confirm when the crop clearly shows the named"
    " target."
    "\nFor every crop, you must:"
    "\n  - List 1-3 visual features that MATCH the target (shape, colour,"
    "\n    typical mounting/placement, distinctive parts)."
    "\n  - List 1-3 features that DO NOT match or look wrong."
    "\n  - Decide confirmed=true ONLY if matches clearly outweigh mismatches"
    "\n    AND the object is recognisable, not just a vaguely similar shape."
    "\nReply with valid JSON ONLY, matching this schema:"
    '\n{"confirmed": bool, "matches": [str, ...], "mismatches": [str, ...],'
    ' "reason": str}'
)


class VerificationDecision(BaseModel):
    confirmed: bool
    matches: list[str] = []
    mismatches: list[str] = []
    reason: str = ""


VERIFY_DECISION_SCHEMA = VerificationDecision.model_json_schema()


@dataclass
class VerifyResult:
    confirmed: bool
    matches: list[str]
    mismatches: list[str]
    reason: str


def _parse_verify_response(raw: str) -> Optional[VerifyResult]:
    """Tolerant parse of a verifier reply. Strict JSON → fenced → embedded →
    heuristic last resort. Mirrors `_parse_vlm_response`."""
    if not raw:
        return None

    def _from_dict(d: dict) -> VerifyResult:
        return VerifyResult(
            confirmed=bool(d.get("confirmed", False)),
            matches=list(d.get("matches") or []),
            mismatches=list(d.get("mismatches") or []),
            reason=str(d.get("reason", raw[:200])),
        )

    s = raw.strip()
    if s.startswith("{"):
        try:
            return _from_dict(json.loads(s))
        except (json.JSONDecodeError, ValueError):
            pass

    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", raw, re.DOTALL | re.IGNORECASE)
    if fence:
        try:
            return _from_dict(json.loads(fence.group(1)))
        except (json.JSONDecodeError, ValueError):
            pass

    obj = re.search(r"\{[^{}]*\"confirmed\"\s*:\s*(?:true|false)[^{}]*\}", raw, re.IGNORECASE)
    if obj:
        try:
            return _from_dict(json.loads(obj.group(0)))
        except (json.JSONDecodeError, ValueError):
            pass

    text = raw.lower()
    confirmed = bool(re.search(r"confirmed\s*[:=]\s*true", text)) or bool(
        re.search(r"\b(yes|confirmed|correct)\b.*\btarget\b", text)
    )
    logger.warning("Verifier response unstructured, heuristic parse: %s", raw[:200])
    return VerifyResult(
        confirmed=confirmed,
        matches=[],
        mismatches=[],
        reason=raw[:200],
    )


@dataclass
class VLMResult:
    target_visible: bool
    heading: str
    drive_distance_m: float
    target_bbox: Optional[list[int]]
    reason: str
    # Dimensions of the image actually sent to the VLM (after resize). Needed
    # so callers can rescale target_bbox into other frames (e.g. depth image).
    image_width: int = 0
    image_height: int = 0


_HEADING_TOKENS = {"left", "center", "right"}


def _clamp_distance(v) -> float:
    try:
        d = float(v)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, min(2.0, d))


def _coerce_heading(v) -> str:
    s = str(v or "").strip().lower()
    if s in _HEADING_TOKENS:
        return s
    if s in {"l", "ccw", "anticlockwise"}:
        return "left"
    if s in {"r", "cw", "clockwise"}:
        return "right"
    return "center"


def _coerce_bbox(v) -> Optional[list[int]]:
    if v is None:
        return None
    if not isinstance(v, (list, tuple)) or len(v) != 4:
        return None
    try:
        return [int(x) for x in v]
    except (TypeError, ValueError):
        return None


def _result_from_dict(d: dict, raw: str) -> VLMResult:
    return VLMResult(
        target_visible=bool(d.get("target_visible", False)),
        heading=_coerce_heading(d.get("heading", "center")),
        drive_distance_m=_clamp_distance(d.get("drive_distance_m", 0.0)),
        target_bbox=_coerce_bbox(d.get("target_bbox")),
        reason=str(d.get("reason", d.get("reasoning", raw[:200]))),
    )


def _parse_vlm_response(raw: str) -> Optional[VLMResult]:
    if not raw:
        return None

    s = raw.strip()
    if s.startswith("{"):
        try:
            return _result_from_dict(json.loads(s), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", raw, re.DOTALL | re.IGNORECASE)
    if fence:
        try:
            return _result_from_dict(json.loads(fence.group(1)), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    obj = re.search(r"\{[^{}]*\"target_visible\"\s*:\s*(?:true|false)[^{}]*\}", raw, re.IGNORECASE)
    if obj:
        try:
            return _result_from_dict(json.loads(obj.group(0)), raw)
        except (json.JSONDecodeError, ValueError):
            pass

    text = raw.lower()
    visible = bool(re.search(r"target_visible\s*[:=]\s*true", text)) or bool(
        re.search(r"i\s+(?:can\s+)?see\s+(?:a\s+|the\s+)?(?:fire\s+extinguisher|drink|drill|pipe|suitcase|target)", text)
    )
    heading = "center"
    m = re.search(r"heading\s*[:=]\s*\"?(left|center|right)\"?", text)
    if m:
        heading = m.group(1)
    elif re.search(r"\bturn\s+left|go\s+left\b", text):
        heading = "left"
    elif re.search(r"\bturn\s+right|go\s+right\b", text):
        heading = "right"

    dist = 0.5
    m = re.search(r"(?:drive_distance_m|distance|drive)\s*[:=]\s*([0-9]*\.?[0-9]+)", text)
    if m:
        dist = _clamp_distance(m.group(1))

    logger.warning("VLM response unstructured, heuristic parse: %s", raw[:200])
    return VLMResult(
        target_visible=visible,
        heading=heading,
        drive_distance_m=dist,
        target_bbox=None,
        reason=raw[:200],
    )


class VLMClient:
    def __init__(self, config: dict):
        model_cfg = config["model"]
        self.model_name = model_cfg["name"]
        self.backend = model_cfg.get("backend", "ollama")
        self.is_cloud = self.backend == "ollama-cloud"

        inf_cfg = config["inference"]
        self.max_retries = inf_cfg.get("max_retries", 3)
        self.timeout = inf_cfg.get("timeout_s", 30.0)
        self._client = None

    def start(self, ready_timeout: float = 300.0):
        from ollama import Client
        self._client = Client()

        if self.is_cloud:
            logger.info("Cloud model %s — skipping local pre-load", self.model_name)
            try:
                self._client.chat(
                    model=self.model_name,
                    messages=[{"role": "user", "content": "ping"}],
                    keep_alive=0,
                )
                logger.info("Cloud model %s reachable", self.model_name)
            except Exception as e:
                logger.warning("Cloud model reachability check: %s", e)
            return

        logger.info("Pre-loading Ollama model %s...", self.model_name)
        import time
        t0 = time.time()
        try:
            self._client.chat(
                model=self.model_name,
                messages=[{"role": "user", "content": "ping"}],
                keep_alive=-1,
            )
        except Exception as e:
            logger.warning("Model pre-load attempt: %s (model may still be downloading)", e)
        logger.info("Ollama model %s ready (%.1fs)", self.model_name, time.time() - t0)

    def stop(self):
        if self._client:
            try:
                self._client.chat(model=self.model_name, messages=[], keep_alive=0)
            except Exception:
                pass
        logger.info("Ollama model unloaded")

    def query(self, image, prompt: str, verbose: bool = False) -> Optional[VLMResult]:
        if self._client is None:
            raise RuntimeError("VLM client not started")

        w, h = image.size
        if max(w, h) > MAX_IMAGE_DIM:
            scale = MAX_IMAGE_DIM / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)))
        sent_w, sent_h = image.size

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=JPEG_QUALITY)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        if verbose:
            # Full prompt I/O for the debug harness (#13). Production callers
            # leave verbose=False and keep the terse one-line summary below.
            logger.info("VLM REQUEST · system prompt:\n%s", SYSTEM_PROMPT)
            logger.info("VLM REQUEST · user prompt:\n%s", prompt)
            logger.info("VLM REQUEST · image sent %dx%d JPEG q%d",
                        sent_w, sent_h, JPEG_QUALITY)

        for attempt in range(1, self.max_retries + 1):
            try:
                keep_alive = 0 if self.is_cloud else -1
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": prompt, "images": [img_b64]},
                    ],
                    format=NAV_DECISION_SCHEMA,
                    options={"temperature": 0.3},
                    stream=False,
                    keep_alive=keep_alive,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("VLM returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue
                if verbose:
                    logger.info("VLM RAW RESPONSE:\n%s", raw)

                result = _parse_vlm_response(raw)
                if result is not None:
                    result.image_width = sent_w
                    result.image_height = sent_h
                    bbox_str = (
                        f"[{result.target_bbox[0]},{result.target_bbox[1]},"
                        f"{result.target_bbox[2]},{result.target_bbox[3]}]"
                        if result.target_bbox else "None"
                    )
                    logger.info("VLM: vis=%s hdg=%s dist=%.2f bbox=%s img=%dx%d | %s",
                                result.target_visible, result.heading,
                                result.drive_distance_m, bbox_str,
                                sent_w, sent_h, result.reason[:100])
                    if result.target_visible and result.target_bbox is None:
                        logger.warning("VLM raw (visible w/o bbox): %.500s", raw)
                    return result
                logger.warning("VLM response unparseable (attempt %d/%d): %.200s", attempt, self.max_retries, raw)

            except Exception as e:
                logger.error("VLM query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d VLM attempts failed, defaulting to stop", self.max_retries)
        return VLMResult(
            target_visible=False,
            heading="center",
            drive_distance_m=0.0,
            target_bbox=None,
            reason="VLM query failed after retries",
            image_width=sent_w,
            image_height=sent_h,
        )

    def verify_candidate(self, crop, target_name: str,
                         verbose: bool = False) -> Optional[VerifyResult]:
        """Skeptical second call on a candidate crop (#10).

        Sends ``crop`` (a PIL Image — pre-cropped + upscaled by the caller)
        with a verifier prompt. Returns the verifier's judgement, or None on
        repeated failure.
        """
        if self._client is None:
            raise RuntimeError("VLM client not started")

        w, h = crop.size
        if max(w, h) > MAX_IMAGE_DIM:
            scale = MAX_IMAGE_DIM / max(w, h)
            crop = crop.resize((int(w * scale), int(h * scale)))
        sent_w, sent_h = crop.size

        buf = io.BytesIO()
        crop.save(buf, format="JPEG", quality=JPEG_QUALITY)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        target_natural = (target_name or "the target").replace("_", " ")
        user_prompt = (
            f"Target: {target_name} (natural language: \"{target_natural}\")\n"
            "This crop was flagged as containing the target. Confirm or reject.\n"
            "Be strict; reject vaguely similar shapes."
        )

        if verbose:
            logger.info("VERIFIER REQUEST · system prompt:\n%s", VERIFIER_SYSTEM_PROMPT)
            logger.info("VERIFIER REQUEST · user prompt:\n%s", user_prompt)
            logger.info("VERIFIER REQUEST · crop sent %dx%d JPEG q%d",
                        sent_w, sent_h, JPEG_QUALITY)

        for attempt in range(1, self.max_retries + 1):
            try:
                keep_alive = 0 if self.is_cloud else -1
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": VERIFIER_SYSTEM_PROMPT},
                        {"role": "user", "content": user_prompt, "images": [img_b64]},
                    ],
                    format=VERIFY_DECISION_SCHEMA,
                    options={"temperature": 0.1},  # lower temp → more deterministic verdicts
                    stream=False,
                    keep_alive=keep_alive,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("Verifier returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue
                if verbose:
                    logger.info("VERIFIER RAW RESPONSE:\n%s", raw)

                result = _parse_verify_response(raw)
                if result is not None:
                    logger.info(
                        "VERIFY: confirmed=%s crop=%dx%d matches=%d mismatches=%d | %s",
                        result.confirmed, sent_w, sent_h,
                        len(result.matches), len(result.mismatches),
                        result.reason[:120],
                    )
                    return result
                logger.warning("Verifier response unparseable (attempt %d/%d): %.200s", attempt, self.max_retries, raw)

            except Exception as e:
                logger.error("Verifier query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d verifier attempts failed; defaulting to REJECT (safe)", self.max_retries)
        return VerifyResult(confirmed=False, matches=[], mismatches=[], reason="verifier failed")
