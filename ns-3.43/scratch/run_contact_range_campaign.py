#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import datetime as dt
import shlex
import subprocess
import time
from pathlib import Path


DEFAULT_RANGES = tuple(20.0 + index * (130.0 / 4.0) for index in range(5))
DEFAULT_RANGES_CSV = ",".join(f"{value:.1f}" for value in DEFAULT_RANGES)
MANIFEST_FIELDS = [
    "contact_range",
    "rng_run",
    "run_label",
    "flowmon_file",
    "summary_file",
    "timeseries_file",
    "log_file",
    "duration_s",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ejecuta una campana de level1ToLevel2ContactRange y guarda los artefactos por corrida."
    )
    parser.add_argument(
        "--ranges",
        default=DEFAULT_RANGES_CSV,
        help="Lista separada por comas de rangos de contacto en metros",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=10,
        help="Cantidad de repeticiones (rngRun empieza en 1)",
    )
    parser.add_argument(
        "--results-dir",
        help="Directorio de salida. Si no se indica, se usa results/contact_range_campaign/<timestamp>",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Omitir la compilacion previa del scratch/adhoc",
    )
    return parser.parse_args()


def parse_ranges(raw_value: str) -> list[float]:
    values = []
    for chunk in raw_value.split(","):
        item = chunk.strip()
        if not item:
            continue
        values.append(float(item))
    return values or list(DEFAULT_RANGES)


def make_range_slug(value: float) -> str:
    return f"{value:.1f}".replace(".", "_")


def write_manifest_header(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()


def append_manifest_row(path: Path, row: dict[str, object]) -> None:
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writerow(row)


def ensure_output_dirs(root: Path) -> dict[str, Path]:
    dirs = {
        "root": root,
        "raw": root / "raw",
        "logs": root / "logs",
    }
    for directory in dirs.values():
        directory.mkdir(parents=True, exist_ok=True)
    return dirs


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    ranges = parse_ranges(args.ranges)
    if args.runs < 1:
        raise ValueError("runs debe ser >= 1")

    if args.results_dir:
        results_root = Path(args.results_dir).expanduser().resolve()
    else:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        results_root = repo_root / "results" / "contact_range_campaign" / timestamp

    dirs = ensure_output_dirs(results_root)
    manifest_path = results_root / "manifest.csv"
    write_manifest_header(manifest_path)

    if not args.skip_build:
        build_cmd = ["./ns3", "build", "scratch/adhoc"]
        print(f"Compilando: {shlex.join(build_cmd)}")
        subprocess.run(build_cmd, cwd=repo_root, check=True)

    print("=== INICIANDO CAMPANA DE RANGO DE CONTACTO ===")
    print(f"Resultados en: {results_root}")

    for contact_range in ranges:
        range_slug = make_range_slug(contact_range)
        for run in range(1, args.runs + 1):
            label = f"range_{range_slug}_run_{run}"
            flowmon_path = dirs["raw"] / f"flowmon_{label}.xml"
            summary_path = dirs["raw"] / f"summary_{label}.csv"
            timeseries_path = dirs["raw"] / f"timeseries_{label}.csv"
            log_path = dirs["logs"] / f"{label}.log"

            command = [
                "./ns3",
                "run",
                "scratch/adhoc",
                "--",
                f"--level1ToLevel2ContactRange={contact_range:.1f}",
                f"--rngRun={run}",
                f"--metricsRunLabel={label}",
                f"--flowmonFile={flowmon_path}",
                f"--summaryCsvFile={summary_path}",
                f"--timeseriesCsvFile={timeseries_path}",
            ]

            print(f"Ejecutando {label}: {shlex.join(command)}")
            started = time.monotonic()
            with log_path.open("w", encoding="utf-8") as log_handle:
                log_handle.write(shlex.join(command) + "\n\n")
                completed = subprocess.run(
                    command,
                    cwd=repo_root,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            duration = time.monotonic() - started
            if completed.returncode != 0:
                raise RuntimeError(f"La corrida {label} fallo. Ver log: {log_path}")

            append_manifest_row(
                manifest_path,
                {
                    "contact_range": f"{contact_range:.1f}",
                    "rng_run": run,
                    "run_label": label,
                    "flowmon_file": str(flowmon_path),
                    "summary_file": str(summary_path),
                    "timeseries_file": str(timeseries_path),
                    "log_file": str(log_path),
                    "duration_s": f"{duration:.3f}",
                },
            )

    print("=== CAMPANA TERMINADA ===")
    print(f"Listo. Revisa: {results_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
