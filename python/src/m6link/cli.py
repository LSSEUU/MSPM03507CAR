"""Compatible ``m6_tune.py`` CLI backed by :class:`m6link.Session`."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

from . import compat
from .dashboard import run_dashboard
from .errors import M6Error
from .monitor import run_monitor
from .recording import (
    Recorder,
    convert_dashboard_recording,
    load_recording,
    score_loaded_recording,
)
from .scoring import candidate_accepted, evidence_compatible
from .session import BAUD_RATE, DEFAULT_FIRMWARE_PREFIX, PARAMETER_SCHEMA, Session
from .registry import TelemetryStream


def _create_parser() -> argparse.ArgumentParser:
    parser = compat._create_parser()
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="explicitly remain in V1 text mode; V2 is required by default",
    )
    subparsers = next(
        action
        for action in parser._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    score = subparsers.choices["score"]
    score.add_argument(
        "--mode",
        dest="score_mode",
        choices=("wheel", "ground", "line"),
        help="override a recording's inferred test mode",
    )
    score.add_argument(
        "--speed",
        dest="score_speed",
        type=float,
        help="override a recording's inferred requested speed in mm/s",
    )
    adapter = subparsers.add_parser(
        "adapt-dashboard",
        help="convert one dashboard recording directory into score CSV/JSON",
    )
    adapter.add_argument("recording", type=Path)
    adapter.add_argument("output", type=Path)
    adapter.add_argument("--mode", choices=("wheel", "ground", "line"))
    adapter.add_argument("--speed", type=float)
    monitor = subparsers.add_parser(
        "monitor", help="display read-only V2 telemetry in real time"
    )
    monitor.add_argument("--refresh-hz", type=float, default=10.0)
    monitor.add_argument(
        "--trend-seconds",
        type=float,
        default=6.0,
        help="rolling trend window; zero disables trends",
    )
    monitor.add_argument("--duration", type=float)
    monitor.add_argument(
        "--jsonl", action="store_true", help="write snapshots as JSON Lines"
    )
    monitor.add_argument(
        "--output", type=Path, help="also save snapshots as JSON Lines"
    )
    dashboard = subparsers.add_parser(
        "dashboard", help="open the live telemetry, control, and parameter dashboard"
    )
    dashboard.add_argument(
        "--http-port", type=int, default=8765, help="local dashboard port (default: 8765)"
    )
    dashboard.add_argument(
        "--no-open", action="store_true", help="do not open the default browser"
    )
    dashboard.add_argument("--duration", type=float)
    dashboard.add_argument(
        "--recordings-dir",
        type=Path,
        default=Path("recordings"),
        help="dashboard recording library directory (default: ./recordings)",
    )
    return parser


def _json(value: Any) -> str:
    if hasattr(value, "__dataclass_fields__"):
        from dataclasses import asdict
        value = asdict(value)
    return json.dumps(value, ensure_ascii=False, sort_keys=True, default=str)


def _legacy_main(session: Session, identity: Any, args: argparse.Namespace) -> int:
    client = compat.M6Client.from_session(session, response_timeout=args.timeout)
    identity_map = identity.as_legacy_dict()
    if args.command == "probe":
        print(_json(identity_map) if args.json else "port={port} firmware={firmware} schema={schema} revision={revision} source={source} generation={generation}".format(**identity_map))
    elif args.command == "get":
        print(compat._response_json(client.get(args.key)))
    elif args.command == "set":
        print(compat._response_json(client.set(args.key, args.value)))
    elif args.command in ("apply", "revert", "load", "defaults", "save"):
        print(compat._response_json(getattr(client, args.command)()))
    elif args.command == "log":
        print(compat._response_json(client.log(args.mode, args.period_ms)))
    elif args.command == "record":
        result = compat._run_record_command(client, identity_map, args)
        print(result.json_path)
    elif args.command == "tune-wheel":
        result = compat._run_tune_wheel(client, identity_map, args)
        print(json.dumps(result.__dict__, indent=2, ensure_ascii=False, allow_nan=False))
        return 3 if result.aborted else 0
    return 0


def _v2_main(session: Session, identity: Any, args: argparse.Namespace) -> int:
    if args.command == "probe":
        payload = identity.as_legacy_dict()
        payload.update(
            wire_version=identity.wire_version,
            registry_revision=identity.registry_revision,
            boot_id=identity.boot_id,
            capabilities=f"{int(identity.enabled_capabilities):016x}",
        )
        print(_json(payload) if args.json else "port={port} firmware={firmware} schema={schema} revision={revision} source={source} generation={generation}".format(**payload))
    elif args.command == "get":
        if args.key == "meta.firmware":
            value = identity.firmware_id
        elif args.key == "meta.schema":
            value = identity.parameter_schema
        elif args.key == "meta.revision":
            value = identity.active_revision
        elif args.key == "meta.generation":
            value = identity.generation
        elif args.key == "meta.source":
            value = getattr(identity.source, "name", identity.source).lower()
        else:
            value = session.get_parameter(args.key)
        print(_json(value))
    elif args.command == "set":
        print(_json(session.stage_parameter(args.key, args.value)))
    elif args.command == "apply":
        print(_json(session.apply_parameters()))
    elif args.command == "revert":
        print(_json(session.revert_parameters()))
    elif args.command == "load":
        print(_json(session.load_parameters()))
    elif args.command == "defaults":
        print(_json(session.default_parameters()))
    elif args.command == "save":
        print(_json(session.save_parameters()))
    elif args.command == "log":
        entries = {} if args.mode == "off" else {
            TelemetryStream.CORE: args.period_ms,
            TelemetryStream.WHEEL: args.period_ms,
        }
        print(_json(session.subscribe(entries)))
    elif args.command == "record":
        parameters = {
            key: value.active for key, value in session.get_parameters().values.items()
        }
        result = Recorder(session).record(
            args.output,
            duration_s=args.duration,
            period_ms=args.period,
            test_mode=args.mode,
            requested_speed=args.speed,
            identity=identity,
            parameters=parameters,
        )
        print(result.json_path)
    elif args.command == "monitor":
        return run_monitor(
            session,
            identity,
            refresh_hz=args.refresh_hz,
            trend_seconds=args.trend_seconds,
            duration_s=args.duration,
            jsonl=args.jsonl,
            output=args.output,
        )
    elif args.command == "dashboard":
        return run_dashboard(
            session,
            identity,
            port_name=identity.port or args.port or "",
            http_port=args.http_port,
            duration_s=args.duration,
            open_browser=not args.no_open,
            recordings_dir=args.recordings_dir,
        )
    elif args.command == "tune-wheel":
        raise M6Error(
            "V2 tune-wheel requires an explicit human-controlled Arm/Confirm client; "
            "this CLI will not execute it automatically. Pass --legacy only for the "
            "retained interactive V1 workflow"
        )
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = _create_parser()
    args = parser.parse_args(argv)
    if args.command in ("monitor", "dashboard") and args.legacy:
        parser.error(f"{args.command} requires the V2 link; remove --legacy")
    if args.command == "score":
        try:
            samples, metadata = load_recording(
                args.recording,
                test_mode=args.score_mode,
                requested_speed=args.score_speed,
            )
            metrics = score_loaded_recording(samples, metadata)
            output: dict[str, Any] = {"metrics": metrics}
            if args.baseline:
                baseline_samples, baseline_metadata = load_recording(
                    args.baseline,
                    test_mode=args.score_mode,
                    requested_speed=args.score_speed,
                )
                baseline_metrics = score_loaded_recording(
                    baseline_samples, baseline_metadata
                )
                comparable = evidence_compatible(baseline_metadata, metadata)
                accepted, improvement = candidate_accepted(
                    baseline_metrics, metrics
                ) if comparable else (False, -1.0)
                output.update(comparable=comparable, accepted=accepted, improvement=improvement)
            print(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False))
            return 0
        except Exception as exc:
            print(f"M6 error: {exc}", file=sys.stderr)
            return 2
    if args.command == "adapt-dashboard":
        try:
            result = convert_dashboard_recording(
                args.recording,
                args.output,
                test_mode=args.mode,
                requested_speed=args.speed,
            )
            print(json.dumps({
                "csv": str(result.csv_path),
                "json": str(result.json_path),
                "metrics": result.summary["metrics"],
            }, indent=2, ensure_ascii=False, allow_nan=False))
            return 3 if result.summary["metrics"].get("hard_failures") else 0
        except Exception as exc:
            print(f"M6 error: {exc}", file=sys.stderr)
            return 2
    session = Session(
        port=args.port,
        baudrate=args.baud,
        legacy=args.legacy,
        timeout=args.timeout,
        firmware_prefix=args.firmware_prefix,
        firmware_id=args.firmware_id,
        parameter_schema=args.schema,
    )
    try:
        identity = session.connect()
        return _legacy_main(session, identity, args) if args.legacy else _v2_main(session, identity, args)
    except (M6Error, compat.M6Error, ValueError, OSError, KeyError) as exc:
        print(f"M6 error: {exc}", file=sys.stderr)
        return 2
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
