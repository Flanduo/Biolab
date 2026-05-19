#!/usr/bin/env python3
"""Protocol parser compatibility exports."""

try:
    # Preferred: shared SDK implementation
    from openarm_sdk.exo import ExoProtocolParser
except Exception:  # pragma: no cover - fallback for legacy runtime
    from .exo_protocol_parser import ExoProtocolParser

__all__ = ["ExoProtocolParser"]