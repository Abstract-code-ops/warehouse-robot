#!/usr/bin/env python3
"""
llm_gateway_node.py
─────────────────────────────────────────────────────────────────────────────
Converts natural-language task requests into structured JSON using an LLM.

Topics:
  SUB  /task_request_raw     std_msgs/String  — raw operator text
  PUB  /task_request_parsed  std_msgs/String  — structured JSON output

Supported backends (set env var):
  CEREBRAS_API_KEY   → uses cerebras/llama-3.3-70b  (fastest, recommended)
  ANTHROPIC_API_KEY  → uses anthropic/claude-sonnet-4-20250514
  OPENAI_API_KEY     → uses openai/gpt-4o-mini
  Neither            → MOCK mode (rule-based matching for demos without key)

  All LLM calls go through litellm — any litellm-supported model string works.
  Override with env var:  LLM_MODEL=groq/llama3-70b-8192

Parsed output schema:
  { "intent": "fetch" | "clarify",
    "product_id": "SKU-XXXX",
        "product_ids": ["SKU-XXXX", "SKU-YYYY"],  // optional for multi-pick
    "quantity": 1,
    "priority": false,
    "raw_input": "...",
    "question": "..."  (only on clarify) }
"""

import json
import os
import re
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from cognitive_amr.warehouse_constants import PRODUCTS

# ── Build product catalogue string for system prompt ─────────────────────
def _build_catalogue() -> str:
    lines = ['Product catalogue (product_id → name):']
    for sku, info in PRODUCTS.items():
        lines.append(f"  {sku}: {info['name']}")
    return '\n'.join(lines)


SYSTEM_PROMPT = f"""You are a warehouse task parser for an autonomous mobile robot.

{_build_catalogue()}

Your job: parse the operator's text and output ONLY valid JSON, no extra text.

Output schema:
{{
  "intent": "fetch" | "clarify",
  "product_id": "SKU-XXXX",    // null if unclear
    "product_ids": ["SKU-XXXX", "SKU-YYYY"], // optional for multi-item requests
  "quantity": 1,                // integer, default 1
  "priority": false,            // true if operator uses words like urgent/rush/asap
  "raw_input": "...",           // copy of original input
  "question": "..."             // only present when intent=clarify
}}

Rules:
- Match product names fuzzily (ignore case, partial match, synonyms).
- If the input clearly refers to one product, set intent=fetch.
- If the input clearly refers to multiple products, set intent=fetch and provide product_ids.
- If ambiguous (could match 2+ products), set intent=clarify and ask.
- If no product name is recognisable at all, set intent=clarify.
- Priority keywords: urgent, rush, asap, priority, express.
- Never output anything outside the JSON object.
"""

MAX_TOKENS = 200


class LLMGatewayNode(Node):

    def __init__(self):
        super().__init__('llm_gateway_node')

        self._backend = self._detect_backend()
        self.get_logger().info(f"LLM backend: {self._backend}")

        self._sub = self.create_subscription(
            String, '/task_request_raw', self._on_raw, 10)
        self._pub = self.create_publisher(
            String, '/task_request_parsed', 10)

    # ── Backend detection ─────────────────────────────────────────────────

    def _detect_backend(self) -> str:
        # Use 'or' so an empty-string LLM_MODEL falls through to the default
        model_override = os.environ.get('LLM_MODEL', '').strip()
        if os.environ.get('CEREBRAS_API_KEY'):
            return model_override or 'cerebras/llama3.1-8b'
        if os.environ.get('ANTHROPIC_API_KEY'):
            return model_override or 'anthropic/claude-sonnet-4-20250514'
        if os.environ.get('OPENAI_API_KEY'):
            return model_override or 'openai/gpt-4o-mini'
        return 'mock'

    # ── Inbound raw request ───────────────────────────────────────────────

    def _on_raw(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return
        self.get_logger().info(f"LLM request: '{raw}'")
        # Parse in background thread to avoid blocking ROS spin
        threading.Thread(target=self._parse_and_publish,
                         args=(raw,), daemon=True).start()

    def _parse_and_publish(self, raw: str):
        try:
            result = self._parse(raw)
        except Exception as e:
            self.get_logger().error(
                f"LLM parse error ({self._backend}): {e} — falling back to mock")
            result = self._parse_mock(raw)

        self.get_logger().info(f"LLM result: {json.dumps(result)}")
        self._pub.publish(String(data=json.dumps(result)))

    # ── Parse dispatch ────────────────────────────────────────────────────

    def _parse(self, raw: str) -> dict:
        if self._backend == 'mock':
            return self._parse_mock(raw)
        return self._parse_litellm(raw)

    # ── LiteLLM backend (covers Cerebras, Anthropic, OpenAI, Groq, …) ────

    def _parse_litellm(self, raw: str) -> dict:
        import litellm   # type: ignore
        response = litellm.completion(
            model=self._backend,
            max_tokens=MAX_TOKENS,
            messages=[
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user',   'content': raw},
            ]
        )
        text = response.choices[0].message.content.strip()
        return self._extract_json(text, raw)

    # ── Mock backend (no API key needed) ──────────────────────────────────

    def _parse_mock(self, raw: str) -> dict:
        """Rule-based matcher for demos without an LLM API key."""
        raw_lower = raw.lower()

        sku_tokens = re.findall(r'sku-?\d{4}', raw_lower)
        normalized_skus = []
        for token in sku_tokens:
            digits = token.replace('sku', '').replace('-', '')
            normalized_skus.append(f'SKU-{digits}')
        normalized_skus = [s for s in normalized_skus if s in PRODUCTS]
        normalized_skus = list(dict.fromkeys(normalized_skus))

        priority = any(kw in raw_lower for kw in
                       ('urgent', 'rush', 'asap', 'priority', 'express'))

        if len(normalized_skus) > 1:
            return {
                'intent': 'fetch',
                'product_id': normalized_skus[0],
                'product_ids': normalized_skus,
                'quantity': 1,
                'priority': priority,
                'raw_input': raw,
            }

        matches = []
        for sku, info in PRODUCTS.items():
            name_lower = info['name'].lower()
            # Check each word in the product name
            words = [w for w in name_lower.split() if len(w) > 3]
            if any(w in raw_lower for w in words):
                matches.append(sku)
            # Direct SKU match
            if sku.lower() in raw_lower:
                matches = [sku]
                break

        if len(matches) == 1:
            return {'intent': 'fetch', 'product_id': matches[0],
                    'quantity': 1, 'priority': priority, 'raw_input': raw}

        if len(matches) > 1:
            names = ', '.join(
                f"{m} ({PRODUCTS[m]['name']})" for m in matches[:4])
            return {'intent': 'clarify',
                    'question': f"Did you mean: {names}?",
                    'raw_input': raw, 'priority': priority}

        return {'intent': 'clarify',
                'question': "Product not recognised. Please specify a SKU "
                            "or product name from the catalogue.",
                'raw_input': raw, 'priority': priority}

    # ── JSON extractor ────────────────────────────────────────────────────

    @staticmethod
    def _extract_json(text: str, raw: str) -> dict:
        match = re.search(r'\{.*\}', text, re.DOTALL)
        if match:
            try:
                d = json.loads(match.group())
                d.setdefault('raw_input', raw)
                return d
            except json.JSONDecodeError:
                pass
        return {'intent': 'clarify',
                'question': f"Could not parse LLM response: {text[:80]}",
                'raw_input': raw}


def main(args=None):
    rclpy.init(args=args)
    node = LLMGatewayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
