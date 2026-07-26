"""Stable public scoring API extracted from the legacy tool."""

from .compat import (
    aggregate_validation_metrics,
    candidate_accepted,
    evidence_compatible,
    feedforward_evidence_passed,
    fit_feedforward,
    score_samples,
    telemetry_abort_reason,
)

__all__ = [
    "aggregate_validation_metrics", "candidate_accepted", "evidence_compatible",
    "feedforward_evidence_passed", "fit_feedforward", "score_samples",
    "telemetry_abort_reason",
]
