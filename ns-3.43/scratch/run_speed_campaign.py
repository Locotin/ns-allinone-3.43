#!/usr/bin/env python3
"""
run_speed_campaign.py
======================
Script de campaña paramétrica que estudia el efecto de la velocidad de movimiento
de los nodos sobre el protocolo SCF en la simulación MANET de agricultura de precisión.

Propósito
---------
Ejecuta múltiples corridas de ``scratch/adhoc`` variando el parámetro
``--followerSpeedMax`` (velocidad máxima de los nodos seguidores en m/s) para
analizar cómo la movilidad afecta la frecuencia y duración de los contactos entre
nodos, y en consecuencia las métricas de red (PDR, delay E2E, AoI, cobertura).

Parámetro barrido
-----------------
- ``followerSpeedMax``: velocidad máxima en m/s de los nodos que siguen a los
  líderes de cada clúster (modelo Random Waypoint).
  Valores predeterminados: 5.0, 10.0, 15.0 m/s.

Estructura de salida
--------------------
``results/speed_campaign/<timestamp>/``
├── manifest.csv
├── raw/
│   ├── flowmon_<label>.xml
│   ├── summary_<label>.csv
│   └── timeseries_<label>.csv
└── logs/
    └── <label>.log

Uso típico
----------
    python3 scratch/run_speed_campaign.py
    python3 scratch/run_speed_campaign.py --speeds 2.0,8.0,14.0 --runs 5
    python3 scratch/run_speed_campaign.py --skip-build

El script se ejecuta desde la raíz del repositorio ns-3.43.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import shlex
import subprocess
import time
from pathlib import Path


DEFAULT_SPEEDS = (5.0, 10.0, 15.0)
MANIFEST_FIELDS = [
    "follower_speed_max",
    "rng_run",
    "run_label",
    "flowmon_file",
    "summary_file",
    "timeseries_file",
    "log_file",
    "duration_s",
]


def parse_args() -> argparse.Namespace:
    """
    Parsea los argumentos de línea de comandos.

    Retorna
    -------
    argparse.Namespace con los campos:
      - speeds (str): lista CSV de velocidades máximas en m/s.
      - runs (int): número de semillas aleatorias (rngRun 1..runs) por velocidad.
      - results_dir (str | None): directorio de salida; None = auto-timestamp.
      - skip_build (bool): si True, omite la compilación previa.
    """
    parser = argparse.ArgumentParser(
        description="Ejecuta una campana de followerSpeedMax y guarda los artefactos por corrida."
    )
    parser.add_argument(
        "--speeds",
        default="5.0,10.0,15.0",
        help="Lista separada por comas de velocidades maximas en m/s",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=10,
        help="Cantidad de repeticiones (rngRun empieza en 1)",
    )
    parser.add_argument(
        "--results-dir",
        help="Directorio de salida. Si no se indica, se usa results/speed_campaign/<timestamp>",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Omitir la compilacion previa del scratch/adhoc",
    )
    return parser.parse_args()


def parse_speeds(raw_value: str) -> list[float]:
    """Convierte una cadena CSV de velocidades (ej. ``"5.0,10.0,15.0"``) en lista de floats."""
    values = []
    for chunk in raw_value.split(","):
        item = chunk.strip()
        if not item:
            continue
        values.append(float(item))
    return values or list(DEFAULT_SPEEDS)


def make_speed_slug(value: float) -> str:
    """Convierte un float de velocidad a slug seguro para nombres de archivo (ej. ``10.0`` → ``"10_0"``)."""
    return f"{value:.1f}".replace(".", "_")


def write_manifest_header(path: Path) -> None:
    """Crea el archivo manifest.csv con la cabecera de columnas definida en MANIFEST_FIELDS."""
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()


def append_manifest_row(path: Path, row: dict[str, object]) -> None:
    """Agrega una fila al manifest.csv con los metadatos de una corrida completada."""
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=MANIFEST_FIELDS)
        writer.writerow(row)


def ensure_output_dirs(root: Path) -> dict[str, Path]:
    """
    Crea la estructura de directorios de salida si no existe.

    Retorna un diccionario con claves ``"root"``, ``"raw"`` y ``"logs"``
    apuntando a sus respectivos ``Path``.
    """
    dirs = {
        "root": root,
        "raw": root / "raw",
        "logs": root / "logs",
    }
    for directory in dirs.values():
        directory.mkdir(parents=True, exist_ok=True)
    return dirs


def main() -> int:
    """
    Punto de entrada principal de la campaña de velocidad.

    Flujo de ejecución
    ------------------
    1. Parsea argumentos y calcula el directorio de resultados.
    2. Compila ``scratch/adhoc`` con ``./ns3 build`` (salvo ``--skip-build``).
    3. Para cada (follower_speed × rng_run) ejecuta una corrida de la simulación
       pasando los parámetros via CLI y redirigiendo stdout/stderr al log.
    4. Registra cada corrida en manifest.csv en cuanto termina.
    5. Lanza un error si alguna corrida devuelve código de salida ≠ 0.

    Retorna 0 si todas las corridas terminan con éxito.
    """
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    speeds = parse_speeds(args.speeds)
    if args.runs < 1:
        raise ValueError("runs debe ser >= 1")

    if args.results_dir:
        results_root = Path(args.results_dir).expanduser().resolve()
    else:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        results_root = repo_root / "results" / "speed_campaign" / timestamp

    dirs = ensure_output_dirs(results_root)
    manifest_path = results_root / "manifest.csv"
    write_manifest_header(manifest_path)

    if not args.skip_build:
        build_cmd = ["./ns3", "build", "scratch/adhoc"]
        print(f"Compilando: {shlex.join(build_cmd)}")
        subprocess.run(build_cmd, cwd=repo_root, check=True)

    print("=== INICIANDO CAMPANA DE VELOCIDAD ===")
    print(f"Resultados en: {results_root}")

    for follower_speed in speeds:
        speed_slug = make_speed_slug(follower_speed)
        for run in range(1, args.runs + 1):
            label = f"speed_{speed_slug}_run_{run}"
            flowmon_path = dirs["raw"] / f"flowmon_{label}.xml"
            summary_path = dirs["raw"] / f"summary_{label}.csv"
            timeseries_path = dirs["raw"] / f"timeseries_{label}.csv"
            log_path = dirs["logs"] / f"{label}.log"

            command = [
                "./ns3",
                "run",
                "scratch/adhoc",
                "--",
                f"--followerSpeedMax={follower_speed:.1f}",
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
                    "follower_speed_max": f"{follower_speed:.1f}",
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
