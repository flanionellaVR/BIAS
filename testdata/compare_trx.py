#!/usr/bin/env python3
"""Compare a FlyTrack trajectory file against a reference trajectory.

Used by the FlyTrack smoke test (``ctest -R flytrack_smoke``), and handy on its
own after a manual ``test_gui --batch`` run. Only the standard library is used.

The check passes when both files hold the same number of frames, the frame
numbers agree, and every frame's ``x``, ``y``, ``a``, ``b`` and ``theta`` lie
within the tolerance of the reference. All other fields (``timestamp``, the
wing fields, and anything newer builds add) are ignored. ``theta`` is compared
modulo 2*pi, so -pi and +pi count as equal.

The expected frame count comes from the reference file, not from the video:
the image grabber skips a few startup frames, so the trajectory is shorter than
the video.

Exit status: 0 if the trajectories match, 1 if they differ or a file cannot be
read, 2 for a command-line error.

Usage
-----
    python testdata/compare_trx.py trx.json testdata/trx_reference.json --tolerance 0.01
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

FIELDS = ("x", "y", "a", "b", "theta")
ANGLE_FIELDS = frozenset({"theta"})
FRAME_FIELD = "frame"

# FlyTrack writes doubles with QString::arg, which renders non-finite values as
# nan, inf and -inf. Those are not JSON, so map them to the constants that
# Python's json module accepts. Only value positions (after ':', ',' or '[')
# are rewritten, never text inside keys.
_NON_FINITE_VALUE = re.compile(r"(?<=[:,\[])(\s*)(-?)(nan|inf)\b", re.IGNORECASE)


class TrxError(Exception):
    """A trajectory file is missing, unreadable or not in the FlyTrack format."""


@dataclass(frozen=True)
class FieldDiff:
    field: str
    produced: Any
    reference: Any
    difference: float


@dataclass(frozen=True)
class FrameMismatch:
    index: int
    frame: Any
    diffs: tuple[FieldDiff, ...]


@dataclass(frozen=True)
class Comparison:
    produced_count: int
    reference_count: int
    mismatches: list[FrameMismatch]
    max_difference: dict[str, float]

    @property
    def ok(self) -> bool:
        return self.produced_count == self.reference_count and not self.mismatches


def _to_json_constant(match: re.Match[str]) -> str:
    space, sign, word = match.groups()
    if word.lower() == "nan":
        return f"{space}NaN"
    return f"{space}{sign}Infinity"


def load_track(path: Path) -> list[dict[str, Any]]:
    """Return the list of per-frame entries from a FlyTrack trajectory file."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as err:
        raise TrxError(f"{path}: cannot read file ({err.strerror})") from err

    try:
        data = json.loads(_NON_FINITE_VALUE.sub(_to_json_constant, text))
    except json.JSONDecodeError as err:
        raise TrxError(
            f"{path}: not valid JSON ({err}). A truncated file usually means "
            "BIAS did not stop capture cleanly, so the trajectory was never closed."
        ) from err

    track = data.get("track") if isinstance(data, dict) else None
    if not isinstance(track, list):
        raise TrxError(f'{path}: no "track" list at the top level')

    for index, entry in enumerate(track):
        if not isinstance(entry, dict):
            raise TrxError(f"{path}: track entry {index} is not a JSON object")
        missing = [field for field in FIELDS if field not in entry]
        if missing:
            raise TrxError(f"{path}: track entry {index} has no {', '.join(missing)}")
        for field in FIELDS:
            value = entry[field]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TrxError(
                    f"{path}: track entry {index} field {field} is not a number"
                )
    return track


def field_difference(field: str, produced: float, reference: float) -> float:
    """Return |produced - reference|, wrapped to [0, pi] for angle fields.

    Two NaNs count as equal (no detection in both runs), NaN against a number
    as an infinite difference.
    """
    produced_nan = math.isnan(produced)
    reference_nan = math.isnan(reference)
    if produced_nan or reference_nan:
        return 0.0 if produced_nan and reference_nan else math.inf
    if produced == reference:
        return 0.0
    difference = produced - reference
    if not math.isfinite(difference):
        return math.inf
    if field in ANGLE_FIELDS:
        difference = math.remainder(difference, math.tau)
    return abs(difference)


def compare_tracks(
    produced: Sequence[dict[str, Any]],
    reference: Sequence[dict[str, Any]],
    tolerances: dict[str, float],
) -> Comparison:
    """Compare the frames both tracks have in common, field by field."""
    mismatches: list[FrameMismatch] = []
    max_difference = dict.fromkeys(FIELDS, 0.0)

    # Only the common prefix is compared; a length difference is reported
    # separately from the frame count.
    for index in range(min(len(produced), len(reference))):
        prod = produced[index]
        ref = reference[index]
        diffs: list[FieldDiff] = []

        prod_frame = prod.get(FRAME_FIELD)
        ref_frame = ref.get(FRAME_FIELD)
        if prod_frame != ref_frame:
            diffs.append(FieldDiff(FRAME_FIELD, prod_frame, ref_frame, math.inf))

        for field in FIELDS:
            difference = field_difference(field, prod[field], ref[field])
            max_difference[field] = max(max_difference[field], difference)
            if difference > tolerances[field]:
                diffs.append(FieldDiff(field, prod[field], ref[field], difference))

        if diffs:
            mismatches.append(FrameMismatch(index, ref_frame, tuple(diffs)))

    return Comparison(len(produced), len(reference), mismatches, max_difference)


def _format_number(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def format_report(
    comparison: Comparison,
    produced_path: Path,
    reference_path: Path,
    tolerances: dict[str, float],
    max_report: int,
) -> str:
    """Build the human-readable summary printed by the command line tool."""
    status = "OK" if comparison.ok else "MISMATCH"
    tolerance_text = ", ".join(
        f"{field} {tolerances[field]:g}{' rad' if field in ANGLE_FIELDS else ''}"
        for field in FIELDS
    )
    max_text = ", ".join(
        f"{field} {_format_number(comparison.max_difference[field])}"
        for field in FIELDS
    )
    lines = [
        f"compare_trx: {status}",
        f"  produced:  {produced_path} ({comparison.produced_count} frames)",
        f"  reference: {reference_path} ({comparison.reference_count} frames)",
        f"  tolerance: {tolerance_text}",
        f"  largest difference: {max_text}",
    ]

    if comparison.produced_count != comparison.reference_count:
        lines.append(
            f"  frame count differs: produced {comparison.produced_count}, "
            f"reference {comparison.reference_count}"
        )

    compared = min(comparison.produced_count, comparison.reference_count)
    if comparison.mismatches:
        shown = comparison.mismatches[:max_report]
        lines.append(
            f"  {len(comparison.mismatches)} of {compared} compared frames differ; "
            f"first {len(shown)}:"
        )
        for mismatch in shown:
            details = "; ".join(
                f"{diff.field} {_format_number(diff.produced)} vs "
                f"{_format_number(diff.reference)}"
                + (
                    ""
                    if diff.field == FRAME_FIELD
                    else f" (diff {diff.difference:.3g})"
                )
                for diff in mismatch.diffs
            )
            lines.append(
                f"    entry {mismatch.index} (frame {mismatch.frame}): {details}"
            )
    elif comparison.ok:
        lines.append(f"  all {compared} frames match")

    return "\n".join(lines)


def _non_negative_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None
    if not value >= 0.0:
        raise argparse.ArgumentTypeError(f"must be >= 0: {text!r}")
    return value


def _positive_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {text!r}") from None
    if value < 1:
        raise argparse.ArgumentTypeError(f"must be >= 1: {text!r}")
    return value


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a FlyTrack trajectory (trx.json) against a reference: frame "
            "count, frame numbers and per-frame x, y, a, b, theta."
        ),
    )
    parser.add_argument("produced", type=Path, help="trajectory file to check")
    parser.add_argument("reference", type=Path, help="known-good reference trajectory")
    parser.add_argument(
        "--tolerance",
        type=_non_negative_float,
        required=True,
        help="largest allowed absolute difference per field (pixels for x, y, a, b; "
        "radians for theta unless --theta-tolerance is given)",
    )
    parser.add_argument(
        "--theta-tolerance",
        type=_non_negative_float,
        help="largest allowed absolute difference for theta in radians "
        "(default: --tolerance)",
    )
    parser.add_argument(
        "--max-report",
        type=_positive_int,
        default=10,
        help="number of differing frames to list (default: %(default)s)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    tolerances = dict.fromkeys(FIELDS, args.tolerance)
    if args.theta_tolerance is not None:
        tolerances["theta"] = args.theta_tolerance

    try:
        produced = load_track(args.produced)
        reference = load_track(args.reference)
    except TrxError as err:
        print(f"compare_trx: ERROR: {err}", file=sys.stderr)
        return 1

    comparison = compare_tracks(produced, reference, tolerances)
    print(
        format_report(
            comparison, args.produced, args.reference, tolerances, args.max_report
        )
    )
    return 0 if comparison.ok else 1


if __name__ == "__main__":
    sys.exit(main())
