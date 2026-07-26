"""Compatible ``m6_tune.py`` CLI backed by :class:`m6link.Session`."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

from . import compat
from .errors import M6Error
from .recording import Recorder, load_recording
from .scoring import candidate_accepted, evidence_compatible, score_samples
from .session import BAUD_RATE, DEFAULT_FIRMWARE_PREFIX, PARAMETER_SCHEMA, Session
from .registry import TelemetryStream


def _create_parser() -> argparse.ArgumentParser:
    parser = compat._create_parser()
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="explicitly remain in V1 text mode; V2 is required by default",
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
    if args.command == "score":
        try:
            samples, metadata = load_recording(args.recording)
            metrics = score_samples(
                samples,
                requested_speed=float(metadata["requested_speed"]),
                mode=str(metadata["test_mode"]),
                expected_period_ms=int(metadata.get("period_ms", 20)),
            )
            output: dict[str, Any] = {"metrics": metrics}
            if args.baseline:
                _, baseline_metadata = load_recording(args.baseline)
                comparable = evidence_compatible(baseline_metadata, metadata)
                accepted, improvement = candidate_accepted(baseline_metadata["metrics"], metrics) if comparable else (False, -1.0)
                output.update(comparable=comparable, accepted=accepted, improvement=improvement)
            print(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False))
            return 0
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
