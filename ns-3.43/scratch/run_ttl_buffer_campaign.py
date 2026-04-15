#!/usr/bin/env python3
"""
run_ttl_buffer_campaign.py
===========================
Script de campaña paramétrica 2-D que estudia la interacción entre el tiempo de vida
(TTL) de las muestras y la capacidad del buffer SCF en la simulación MANET de
agricultura de precisión.

Propósito
---------
Ejecuta múltiples corridas de ``scratch/adhoc`` variando simultáneamente dos
parámetros del protocolo Store-Carry-Forward (SCF):

- ``--sampleTtl``: tiempo de vida en segundos de cada paquete de datos en el
  buffer SCF. Los paquetes que superan este tiempo son descartados (TTL drop).
- ``--bufferCapacityPackets``: número máximo de paquetes que puede almacenar
  cada nodo relay en su buffer SCF.

El cruce factorial de ambos parámetros genera una matriz de experimentos que
permite observar el compromiso entre frescura de datos (TTL bajo) y retención
de muestras (buffer grande).

Parámetros barridos
-------------------
- TTLs predeterminados: 60 s, 120 s, 240 s
- Buffers predeterminados: 64, 128, 256 paquetes

Estructura de salida
--------------------
``results/ttl_buffer_campaign/<timestamp>/``
├── manifest.csv
├── raw/
│   ├── flowmon_<label>.xml
│   ├── summary_<label>.csv
│   └── timeseries_<label>.csv
└── logs/
    └── <label>.log

Uso típico
----------
    python3 scratch/run_ttl_buffer_campaign.py
    python3 scratch/run_ttl_buffer_campaign.py --ttls 60,180 --buffers 32,64,128 --runs 5
    python3 scratch/run_ttl_buffer_campaign.py --skip-build

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


DEFAULT_TTLS = (60.0, 120.0, 240.0)
DEFAULT_BUFFERS = (64, 128, 256)
MANIFEST_FIELDS = [
    "sample_ttl_s",
    "buffer_capacity_packets",
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
      - ttls (str): lista CSV de TTLs de muestra en segundos.
      - buffers (str): lista CSV de capacidades de buffer en paquetes.
      - runs (int): número de semillas aleatorias (rngRun 1..runs) por combinación.
      - results_dir (str | None): directorio de salida; None = auto-timestamp.
      - skip_build (bool): si True, omite la compilación previa.
    """
    parser = argparse.ArgumentParser(
        description="Ejecuta una campana de sampleTtl y bufferCapacityPackets y guarda los artefactos por corrida."
    )
    parser.add_argument(
        "--ttls",
        default="60.0,120.0,240.0",
        help="Lista separada por comas de TTLs de muestra en segundos",
    )
    parser.add_argument(
        "--buffers",
        default="64,128,256",
        help="Lista separada por comas de capacidades de buffer en paquetes",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=10,
        help="Cantidad de repeticiones (rngRun empieza en 1)",
    )
    parser.add_argument(
        "--results-dir",
        help="Directorio de salida. Si no se indica, se usa results/ttl_buffer_campaign/<timestamp>",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Omitir la compilacion previa del scratch/adhoc",
    )
    return parser.parse_args()


def parse_ttls(raw_value: str) -> list[float]:
    """Convierte una cadena CSV de TTLs (ej. ``"60.0,120.0,240.0"``) en lista de floats."""
    values = []
    for chunk in raw_value.split(","):
        item = chunk.strip()
        if not item:
            continue
        values.append(float(item))
    return values or list(DEFAULT_TTLS)


def parse_buffers(raw_value: str) -> list[int]:
    """Convierte una cadena CSV de capacidades (ej. ``"64,128,256"``) en lista de enteros."""
    values = []
    for chunk in raw_value.split(","):
        item = chunk.strip()
        if not item:
            continue
        values.append(int(item))
    return values or list(DEFAULT_BUFFERS)


def make_ttl_slug(value: float) -> str:
    """Convierte un float de TTL a slug seguro para nombres de archivo (ej. ``120.0`` → ``"120_0"``)."""
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
    Punto de entrada principal de la campaña TTL × Buffer.

    Flujo de ejecución
    ------------------
    1. Parsea argumentos y valida que TTLs > 0 y buffers ≥ 1.
    2. Compila ``scratch/adhoc`` con ``./ns3 build`` (salvo ``--skip-build``).
    3. Para cada (sample_ttl × buffer_capacity × rng_run) ejecuta una corrida
       pasando los parámetros via CLI y redirigiendo stdout/stderr al log.
    4. Registra cada corrida en manifest.csv en cuanto termina.
    5. Lanza un error si alguna corrida devuelve código de salida ≠ 0.

    Retorna 0 si todas las corridas terminan con éxito.
    """
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    ttls = parse_ttls(args.ttls)
    buffers = parse_buffers(args.buffers)

    if args.runs < 1:
        raise ValueError("runs debe ser >= 1")
    if any(ttl <= 0.0 for ttl in ttls):
        raise ValueError("todos los TTL deben ser > 0")
    if any(buffer < 1 for buffer in buffers):
        raise ValueError("todos los buffers deben ser >= 1")

    if args.results_dir:
        results_root = Path(args.results_dir).expanduser().resolve()
    else:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        results_root = repo_root / "results" / "ttl_buffer_campaign" / timestamp

    dirs = ensure_output_dirs(results_root)
    manifest_path = results_root / "manifest.csv"
    write_manifest_header(manifest_path)

    if not args.skip_build:
        build_cmd = ["./ns3", "build", "scratch/adhoc"]
        print(f"Compilando: {shlex.join(build_cmd)}")
        subprocess.run(build_cmd, cwd=repo_root, check=True)

    print("=== INICIANDO CAMPANA DE TTL Y BUFFER ===")
    print(f"Resultados en: {results_root}")

    for sample_ttl in ttls:
        ttl_slug = make_ttl_slug(sample_ttl)
        for buffer_capacity in buffers:
            for run in range(1, args.runs + 1):
                label = f"ttl_{ttl_slug}_buf_{buffer_capacity}_run_{run}"
                flowmon_path = dirs["raw"] / f"flowmon_{label}.xml"
                summary_path = dirs["raw"] / f"summary_{label}.csv"
                timeseries_path = dirs["raw"] / f"timeseries_{label}.csv"
                log_path = dirs["logs"] / f"{label}.log"

                command = [
                    "./ns3",
                    "run",
                    "scratch/adhoc",
                    "--",
                    f"--sampleTtl={sample_ttl:.1f}",
                    f"--bufferCapacityPackets={buffer_capacity}",
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
                        "sample_ttl_s": f"{sample_ttl:.1f}",
                        "buffer_capacity_packets": buffer_capacity,
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
