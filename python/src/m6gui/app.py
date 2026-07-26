"""Command-line entry point for the M6 Tk desktop application."""

from __future__ import annotations

import argparse
from typing import Any, Sequence

from .controller import Controller


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="M6 V2 desktop control GUI")
    result.add_argument(
        "--fake",
        action="store_true",
        help="use deterministic fake Session; no serial device is opened",
    )
    return result


def session_factory(*, fake: bool) -> Any:
    if fake:
        from .fake import FakeSession

        return FakeSession
    try:
        from m6link import Session
    except ImportError as exc:
        raise SystemExit(
            "m6link is not importable. Install/use the repository package or run with --fake."
        ) from exc
    return Session


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    factory = session_factory(fake=args.fake)
    controller = Controller(factory)
    from .view import run_gui

    run_gui(controller, fake=args.fake)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
