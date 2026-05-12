import json
import logging
import re
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)

VALID_ACTIONS = {"forward", "backward", "left", "right", "stop"}

SYSTEM_PROMPT = (
    "You are a robot navigation agent. "
    "You receive a camera image and the last few actions you took. "
    "You must decide the next action to navigate toward your mission target. "
    'Respond with JSON: {"action": "forward|backward|left|right|stop", '
    '"reasoning": "brief explanation", "target_visible": true|false}. '
    "Only output valid JSON, no other text."
)


@dataclass
class VLMResult:
    action: str
    reasoning: str
    target_visible: bool


def _parse_response(text: str) -> Optional[VLMResult]:
    json_match = re.search(r"\{[^{}]+\}", text)
    if not json_match:
        logger.warning("No JSON object found in VLM response: %s", text[:200])
        return None

    try:
        data = json.loads(json_match.group())
    except json.JSONDecodeError:
        logger.warning("Failed to parse JSON from VLM: %s", json_match.group()[:200])
        return None

    action = data.get("action", "stop").lower().strip()
    if action not in VALID_ACTIONS:
        logger.warning("Invalid action '%s', defaulting to stop", action)
        action = "stop"

    reasoning = str(data.get("reasoning", ""))
    target_visible = bool(data.get("target_visible", False))

    return VLMResult(action=action, reasoning=reasoning, target_visible=target_visible)


def _inference_worker(model_name: str, device: str, dtype_str: str, request_queue, result_queue):
    import multiprocessing
    import torch
    import transformers
    from PIL import Image

    dtype = getattr(torch, dtype_str, torch.float16)

    logger.info("Loading model %s on %s (%s)...", model_name, device, dtype_str)
    processor = transformers.AutoProcessor.from_pretrained(model_name, trust_remote_code=True)
    model = transformers.AutoModelForCausalLM.from_pretrained(
        model_name, trust_remote_code=True, torch_dtype=dtype
    ).to(device)
    model.eval()
    logger.info("Model loaded successfully")

    while True:
        item = request_queue.get()
        if item is None:
            break

        image, prompt = item
        try:
            messages = [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"<|image_1|>\n{prompt}"},
            ]
            prompt_text = processor.tokenizer.apply_chat_template(
                messages, tokenize=False, add_generation_prompt=True
            )
            inputs = processor(prompt_text, [image], return_tensors="pt").to(device)

            with torch.no_grad():
                output_ids = model.generate(
                    **inputs,
                    max_new_tokens=256,
                    temperature=0.3,
                    do_sample=True,
                )
            generated_ids = output_ids[:, inputs.input_ids.shape[1]:]
            response = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
            result_queue.put(response)
        except Exception as e:
            logger.error("Inference error: %s", e)
            result_queue.put(None)


class VLMClient:
    def __init__(self, config: dict):
        self.config = config
        model_cfg = config["model"]
        self.model_name = model_cfg["name"]
        self.device = model_cfg.get("device", "cuda")
        self.dtype_str = model_cfg.get("torch_dtype", "float16")

        inf_cfg = config["inference"]
        self.max_retries = inf_cfg.get("max_retries", 3)
        self.timeout = inf_cfg.get("timeout_s", 10.0)

        self._request_queue: Optional[multiprocessing.Queue] = None
        self._result_queue: Optional[multiprocessing.Queue] = None
        self._process: Optional[multiprocessing.Process] = None

    def start(self):
        import multiprocessing
        ctx = multiprocessing.get_context("spawn")
        self._request_queue = ctx.Queue(maxsize=1)
        self._result_queue = ctx.Queue(maxsize=1)
        self._process = ctx.Process(
            target=_inference_worker,
            args=(self.model_name, self.device, self.dtype_str, self._request_queue, self._result_queue),
            daemon=True,
        )
        self._process.start()
        logger.info("VLM subprocess started (pid=%d)", self._process.pid)

    def stop(self):
        if self._process and self._process.is_alive():
            self._request_queue.put(None)
            self._process.join(timeout=5)
            if self._process.is_alive():
                self._process.terminate()
            logger.info("VLM subprocess stopped")

    def query(self, image, prompt: str) -> Optional[VLMResult]:
        if self._request_queue is None:
            raise RuntimeError("VLM client not started")

        for attempt in range(1, self.max_retries + 1):
            try:
                self._request_queue.put((image, prompt), timeout=self.timeout)
                raw = self._result_queue.get(timeout=self.timeout)
                if raw is None:
                    logger.warning("VLM returned None (attempt %d/%d)", attempt, self.max_retries)
                    continue

                result = _parse_response(raw)
                if result is not None:
                    return result
                logger.warning("Failed to parse VLM response (attempt %d/%d)", attempt, self.max_retries)

            except Exception as e:
                logger.error("VLM query error (attempt %d/%d): %s", attempt, self.max_retries, e)

        logger.error("All %d VLM attempts failed, defaulting to stop", self.max_retries)
        return VLMResult(action="stop", reasoning="VLM query failed after retries", target_visible=False)