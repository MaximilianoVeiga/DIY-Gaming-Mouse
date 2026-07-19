#!/usr/bin/env python3
"""Apply or check clang-format on firmware sources (safe on Windows)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FILES = [
    (ROOT / "3360_Mouse_pico.ino", "3360_Mouse_pico.cpp"),
    (ROOT / "relmouse_16.h", "relmouse_16.cpp"),
    (ROOT / "bios" / "spi_recorder" / "spi_recorder.ino", "spi_recorder.cpp"),
]


def main() -> int:
    check_only = "--check" in sys.argv
    ok = True
    nl = b"\n"
    for path, assume in FILES:
        data = path.read_bytes().replace(b"\r\n", nl).replace(b"\r", nl)
        result = subprocess.run(
            ["clang-format", f"--assume-filename={assume}"],
            input=data,
            capture_output=True,
            check=True,
        )
        out = result.stdout
        if not out.endswith(nl):
            out += nl
        rel = path.relative_to(ROOT)
        if check_only:
            if data != out:
                print(f"needs format: {rel}")
                ok = False
            else:
                print(f"ok: {rel}")
        else:
            path.write_bytes(out)
            print(f"formatted: {rel} ({out.count(nl)} lines)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
