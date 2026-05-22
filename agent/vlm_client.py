import base64
import io
import json
import logging
import re
from dataclasses import dataclass
from typing import Literal, Optional

from pydantic import BaseModel

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = (
    "You detect objects in robot camera images."
    "\n\nYour ONLY job is to determine if the target object is visible in the image."
    "\n\nRules:"
    "\n- target_visible MUST be false unless you can CLEARLY see the named target object."
    "\n- Look carefully at ALL objects in the image: on floors, walls, shelves, corners, tables."
    "\n- The target may be small, partially visible, or in the background."
    "\n- When in doubt, set target_visible=false."
    "\n- action should always be 'forward' (navigation is handled separately)."
    "\n- If target_visible is true, explain what you see in reasoning."
    "\n\nYou MUST respond with valid JSON matching this schema:"
    "\n{\"action\": \"forward\", \"reasoning\": \"...\", \"target_visible\": true/false}"
)


class NavigationAction(BaseModel):
    action: Literal["forward", "backward", "left", "right", "stop"]
    reasoning: str
    target_visible: bool


NAV_ACTION_SCHEMA = NavigationAction.model_json_schema()


@dataclass
class VLMResult:
    action: str
    reasoning: str
    target_visible: bool


def _parse_vlm_response(raw: str) -> Optional[VLMResult]:
    if not raw:
        return None

    json_pattern = r'\{[^{}]*"target_visible"\s*:\s*(?:true|false)[^{}]*\}'
    matches = re.findall(json_pattern, raw, re.IGNORECASE)
    for candidate in matches:
        try:
            data = json.loads(candidate)
            return VLMResult(
                action=data.get("action", "forward"),
                reasoning=data.get("reasoning", ""),
                target_visible=bool(data.get("target_visible", False)),
            )
        except (json.JSONDecodeError, ValueError):
            continue

    if raw.strip().startswith("{"):
        try:
            data = json.loads(raw.strip())
            return VLMResult(
                action=data.get("action", "forward"),
                reasoning=data.get("reasoning", ""),
                target_visible=bool(data.get("target_visible", False)),
            )
        except (json.JSONDecodeError, ValueError):
            pass

    visible_patterns = [
        r"target_visible\s*:\s*true",
        r"i\s+can\s+see\s+the\s+target",
        r"the\s+target\s+is\s+visible",
        r"i\s+see\s+(?:a\s+)?(?:fire\s+extinguisher|drink\s+can|drill|pipe|suitcase)",
    ]
    not_visible_patterns = [
        r"target_visible\s*:\s*false",
        r"not\s+visible",
        r"no\s+target",
        r"cannot\s+(?:see|find)",
        r"no\s+(?:fire\s+extinguisher|drink|drill|pipe|suitcase)",
        r"do\s+not\s+see",
        r"is\s+not\s+(?:present|in\s+(?:the\s+)?(?:image|view|frame|picture))",
    ]

    text_lower = raw.lower()
    for pattern in visible_patterns:
        if re.search(pattern, text_lower):
            return VLMResult(action="forward", reasoning=raw[:200], target_visible=True)

    for pattern in not_visible_patterns:
        if re.search(pattern, text_lower):
            return VLMResult(action="forward", reasoning=raw[:200], target_visible=False)

    logger.warning("Could not parse VLM response, treating as not visible: %s", raw[:200])
    return VLMResult(action="forward", reasoning=raw[:200], target_visible=False)


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

    def query(self, image, prompt: str) -> Optional[VLMResult]:
        if self._client is None:
            raise RuntimeError("VLM client not started")

        max_dim = 384
        w, h = image.size
        if max(w, h) > max_dim:
            scale = max_dim / max(w, h)
            image = image.resize((int(w * scale), int(h * scale)))

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=70)
        img_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        for attempt in range(1, self.max_retries + 1):
            try:
                keep_alive = 0 if self.is_cloud else -1
                response = self._client.chat(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": prompt, "images": [img_b64]},
                    ],
                    format=NAV_ACTION_SCHEMA,
                    options={"temperature": 0.3},
                    stream=False,
                    keep_alive=keep_alive,
                )
                raw = response.message.content
                if not raw:
                    logger.warning("VLM returned empty (attempt %d/%d)", attempt, self.max_retries)
                    continue

                result = _parse_vlm_response(raw)
                if result is not None:
                    logger.info("VLM parsed: vis=%s | %s", result.target_visible, result.reasoning[:100])
                    return result
                logger.warning("VLM response unparseable (attempt %d/%d): %.200s", attempt, self.max_retries, raw)

            except Exception as e:
                logger.error("VLM query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d VLM attempts failed, defaulting to stop", self.max_retries)
        return VLMResult(action="stop", reasoning="VLM query failed after retries", target_visible=False)