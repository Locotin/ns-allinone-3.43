#!/usr/bin/env python3
"""
plot_ttl_buffer_campaign.py
============================
Script de análisis y visualización para la campaña factorial TTL × Buffer.

Lee los artefactos generados por ``run_ttl_buffer_campaign.py``, consolida las
métricas de todas las corridas y produce un dashboard HTML+SVG autocontenido
con heatmaps bidimensionales y series temporales de casos representativos.

Diferencias respecto a las otras campañas
-----------------------------------------
Esta campaña varía **dos** parámetros simultáneamente (TTL y buffer), por lo que:
  - La agregación es bidimensional: ``aggregate_summary_2d`` y ``aggregate_timeseries_2d``
    agrupan por la clave compuesta ``(sample_ttl_s, buffer_capacity_packets)``.
  - La visualización principal usa heatmaps en lugar de scatter/barras.
  - Para las series temporales se seleccionan automáticamente hasta 3 "casos
    representativos" (mejor cobertura, referencia y menor cobertura) usando
    ``select_cases`` y ``build_unique_case_rows``.

Salidas generadas
-----------------
Dentro de ``results/ttl_buffer_campaign/<timestamp>/``:

``tables/``
    - ``summary_all_runs.csv``
    - ``summary_aggregated_by_ttl_buffer.csv``
    - ``timeseries_mean_selected_cases.csv``

``plots/``
    - ``heatmaps_dashboard.svg``
        9 heatmaps en cuadrícula 3×3 (cobertura, delay, AoI, drops TTL por capa,
        ocupación de buffer por capa).
    - ``timeseries_dashboard.svg``
        2 paneles de series temporales para los casos seleccionados.
    - ``index.html``
        Dashboard HTML completo.

Uso
---
    python3 scratch/plot_ttl_buffer_campaign.py
    python3 scratch/plot_ttl_buffer_campaign.py results/ttl_buffer_campaign/20240101_120000
"""

from __future__ import annotations

import argparse
import csv
import html
import math
import statistics
from collections import defaultdict
from pathlib import Path

from campaign_plot_utils import (
    build_list_html,
    DEFAULT_CASE_COLORS,
    draw_heatmap_panel,
    draw_line_panel,
    ensure_output_dirs,
    format_float,
    format_metric_value,
    is_float,
    read_manifest,
    read_single_row_csv,
    render_dashboard_index_html,
    resolve_artifact_path,
    resolve_results_dir,
    svg_rect,
    svg_text,
    write_csv,
)


CAMPAIGN_NAME = "ttl_buffer_campaign"
TTL_FIELD = "sample_ttl_s"
BUFFER_FIELD = "buffer_capacity_packets"


def parse_args() -> argparse.Namespace:
    """Parsea el argumento opcional ``results_dir`` (directorio de la campaña a visualizar)."""
    parser = argparse.ArgumentParser(
        description="Consolida y visualiza una carpeta de ttl_buffer_campaign."
    )
    parser.add_argument(
        "results_dir",
        nargs="?",
        help="Carpeta results/ttl_buffer_campaign/<timestamp>. Si se omite, usa la mas reciente.",
    )
    return parser.parse_args()


def ttl_label(ttl: float) -> str:
    """Formatea un TTL en segundos como etiqueta compacta (ej. ``120.0`` → ``"120 s"``)."""
    return f"{ttl:.0f} s" if math.isclose(ttl, round(ttl)) else f"{ttl:.1f} s"


def buffer_label(buffer_capacity: int) -> str:
    """Convierte una capacidad de buffer a cadena de texto (ej. ``128`` → ``"128"``)."""
    return str(buffer_capacity)


def case_label(ttl: float, buffer_capacity: int) -> str:
    """Etiqueta combinada de un caso TTL+Buffer (ej. ``"TTL 120 s / Buf 128"``)."""
    return f"TTL {ttl_label(ttl)} / Buf {buffer_capacity}"


def merge_summary_rows_2d(
    results_root: Path,
    manifest_rows: list[dict[str, str]],
) -> tuple[list[dict[str, str]], list[str], list[float], list[int]]:
    """
    Une el manifest con los archivos ``summary_<label>.csv`` para la campaña 2D.

    Versión especializada de ``merge_summary_rows`` que extrae y retorna
    dos listas de valores únicos: ``ttl_values`` (floats) y ``buffer_values`` (ints).

    Retorna
    -------
    Tupla de cuatro elementos:
      - all_rows: filas combinadas manifest + summary.
      - summary_fields: columnas del summary CSV.
      - ttl_values: lista ordenada de TTLs únicos.
      - buffer_values: lista ordenada de capacidades de buffer únicas.
    """
    all_rows: list[dict[str, str]] = []
    summary_fields: list[str] = []
    ttl_values: set[float] = set()
    buffer_values: set[int] = set()

    for manifest_row in manifest_rows:
        ttl_value = float(manifest_row[TTL_FIELD])
        buffer_value = int(float(manifest_row[BUFFER_FIELD]))
        summary_path = resolve_artifact_path(results_root, manifest_row["summary_file"])
        summary_row = read_single_row_csv(summary_path)
        if not summary_fields:
            summary_fields = list(summary_row.keys())

        merged_row = dict(manifest_row)
        for key in summary_fields:
            if key not in merged_row:
                merged_row[key] = summary_row[key]

        all_rows.append(merged_row)
        ttl_values.add(ttl_value)
        buffer_values.add(buffer_value)

    return all_rows, summary_fields, sorted(ttl_values), sorted(buffer_values)


def aggregate_summary_2d(
    summary_rows: list[dict[str, str]],
    summary_fields: list[str],
    ttl_values: list[float],
    buffer_values: list[int],
) -> tuple[list[dict[str, str]], list[str]]:
    """
    Calcula estadísticas descriptivas agrupando por la clave compuesta (TTL, buffer).

    Para cada combinación (ttl, buffer) calcula media, desv. estándar, mínimo y
    máximo de cada métrica numérica sobre todas las corridas del grupo.
    Retorna (aggregated_rows, fieldnames).
    """
    numeric_fields = [
        field
        for field in summary_fields
        if field not in {"run_label", "rng_run"} and is_float(summary_rows[0][field])
    ]
    grouped: dict[tuple[float, int], list[dict[str, str]]] = defaultdict(list)
    for row in summary_rows:
        grouped[(float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD])))].append(row)

    aggregated_rows: list[dict[str, str]] = []
    fieldnames = [TTL_FIELD, BUFFER_FIELD, "runs"]
    for field in numeric_fields:
        fieldnames.extend([f"{field}_mean", f"{field}_std", f"{field}_min", f"{field}_max"])

    for ttl_value in ttl_values:
        for buffer_value in buffer_values:
            rows = grouped.get((ttl_value, buffer_value))
            if not rows:
                continue
            aggregated = {
                TTL_FIELD: format_float(ttl_value, 1),
                BUFFER_FIELD: str(buffer_value),
                "runs": str(len(rows)),
            }
            for field in numeric_fields:
                values = [float(row[field]) for row in rows]
                aggregated[f"{field}_mean"] = format_float(statistics.mean(values), 6)
                aggregated[f"{field}_std"] = format_float(statistics.stdev(values) if len(values) > 1 else 0.0, 6)
                aggregated[f"{field}_min"] = format_float(min(values), 6)
                aggregated[f"{field}_max"] = format_float(max(values), 6)
            aggregated_rows.append(aggregated)

    return aggregated_rows, fieldnames


def aggregate_timeseries_2d(
    results_root: Path,
    manifest_rows: list[dict[str, str]],
) -> tuple[list[dict[str, str]], list[str]]:
    """
    Lee los archivos ``timeseries_<label>.csv`` y los agrega por (TTL, buffer, time_s).

    Produce una fila por combinación ``(ttl, buffer, instante)`` con la media y
    estadísticas de todas las corridas del grupo en ese instante de tiempo.
    Retorna (aggregated_rows, fieldnames).
    """
    grouped: dict[tuple[float, int, float], dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    timeseries_fields: list[str] = []

    for manifest_row in manifest_rows:
        ttl_value = float(manifest_row[TTL_FIELD])
        buffer_value = int(float(manifest_row[BUFFER_FIELD]))
        timeseries_path = resolve_artifact_path(results_root, manifest_row["timeseries_file"])
        with timeseries_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if not timeseries_fields:
                    timeseries_fields = list(row.keys())
                time_s = float(row["time_s"])
                bucket = grouped[(ttl_value, buffer_value, time_s)]
                for field in timeseries_fields:
                    if field in {"run_label", "time_s"}:
                        continue
                    if is_float(row[field]):
                        bucket[field].append(float(row[field]))

    numeric_fields = [field for field in timeseries_fields if field not in {"run_label", "time_s"}]
    fieldnames = [TTL_FIELD, BUFFER_FIELD, "time_s", "runs"]
    for field in numeric_fields:
        fieldnames.extend([f"{field}_mean", f"{field}_std", f"{field}_min", f"{field}_max"])

    aggregated_rows: list[dict[str, str]] = []
    for ttl_value, buffer_value, time_s in sorted(grouped, key=lambda item: (item[0], item[1], item[2])):
        stats_by_field = grouped[(ttl_value, buffer_value, time_s)]
        row = {
            TTL_FIELD: format_float(ttl_value, 1),
            BUFFER_FIELD: str(buffer_value),
            "time_s": format_float(time_s, 3),
            "runs": str(len(stats_by_field[numeric_fields[0]])) if numeric_fields else "0",
        }
        for field in numeric_fields:
            values = stats_by_field[field]
            row[f"{field}_mean"] = format_float(statistics.mean(values), 6)
            row[f"{field}_std"] = format_float(statistics.stdev(values) if len(values) > 1 else 0.0, 6)
            row[f"{field}_min"] = format_float(min(values), 6)
            row[f"{field}_max"] = format_float(max(values), 6)
        aggregated_rows.append(row)

    return aggregated_rows, fieldnames


def build_highlight_table(aggregated_rows: list[dict[str, str]]) -> str:
    """
    Genera la tabla HTML de resumen con métricas clave por combinación (TTL, buffer).

    Columnas: TTL (s), Buffer (paquetes), Cobertura final (%), Delay E2E (s),
    AoI (s), TTL drops en sensores.
    """
    headers = ["TTL", "Buffer", "Cobertura final", "Delay E2E", "AoI", "TTL drops sensor"]
    parts = ['<table class="metric-table"><thead><tr>']
    for header in headers:
        parts.append(f"<th>{html.escape(header)}</th>")
    parts.append("</tr></thead><tbody>")

    for row in aggregated_rows:
        parts.append("<tr>")
        cells = [
            ttl_label(float(row[TTL_FIELD])),
            row[BUFFER_FIELD],
            format_metric_value("house_coverage_pct", float(row["house_coverage_pct_mean"])),
            format_metric_value("house_e2e_delay_avg_s", float(row["house_e2e_delay_avg_s_mean"])),
            format_metric_value("house_aoi_avg_s", float(row["house_aoi_avg_s_mean"])),
            format_metric_value("sensor_ttl_drops", float(row["sensor_ttl_drops_mean"])),
        ]
        for cell in cells:
            parts.append(f"<td>{html.escape(cell)}</td>")
        parts.append("</tr>")

    parts.append("</tbody></table>")
    return "".join(parts)


def select_reference_case(aggregated_rows: list[dict[str, str]], reserved_keys: set[tuple[float, int]]) -> tuple[float, int]:
    """
    Selecciona un caso "de referencia" para las series temporales del dashboard.

    Primero intenta usar ``(TTL=120, buffer=128)`` como caso de referencia canónico.
    Si ya está reservado (best o worst), selecciona la combinación con cobertura
    más cercana a la mediana del grupo y parámetros más próximos al punto canónico.
    """
    preferred_key = (120.0, 128)
    available_keys = [
        (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD])))
        for row in aggregated_rows
        if (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))) not in reserved_keys
    ]
    if not available_keys:
        return preferred_key
    if preferred_key in available_keys:
        return preferred_key

    coverages = [float(row["house_coverage_pct_mean"]) for row in aggregated_rows]
    coverage_midpoint = statistics.median(coverages)
    ranked = sorted(
        (
            (
                abs(float(row["house_coverage_pct_mean"]) - coverage_midpoint),
                abs(float(row[TTL_FIELD]) - 120.0) + abs(int(float(row[BUFFER_FIELD])) - 128),
                (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))),
            )
            for row in aggregated_rows
            if (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))) not in reserved_keys
        ),
        key=lambda item: (item[0], item[1]),
    )
    return ranked[0][2]


def build_timeseries_signature(
    timeseries_rows: list[dict[str, str]],
    case_key: tuple[float, int],
) -> tuple[tuple[float, float, float], ...]:
    """
    Construye una firma hashable de la serie temporal de un caso (TTL, buffer).

    La firma es una tupla de tripletes ``(time_s, coverage_mean, aoi_mean)``
    ordenados por tiempo.  Permite detectar combinaciones que producen
    comportamientos idénticos y evitar mostrar curvas duplicadas en el dashboard.
    """
    ttl_value, buffer_value = case_key
    rows = [
        row
        for row in timeseries_rows
        if math.isclose(float(row[TTL_FIELD]), ttl_value)
        and int(float(row[BUFFER_FIELD])) == buffer_value
    ]
    rows.sort(key=lambda row: float(row["time_s"]))
    return tuple(
        (
            round(float(row["time_s"]), 3),
            round(float(row["house_coverage_pct_mean"]), 4),
            round(float(row["house_aoi_avg_s_mean"]), 4),
        )
        for row in rows
    )


def build_unique_case_rows(
    aggregated_rows: list[dict[str, str]],
    timeseries_rows: list[dict[str, str]],
) -> list[dict[str, str]]:
    """
    Filtra las combinaciones (TTL, buffer) que producen comportamientos temporales únicos.

    Agrupa las combinaciones por su firma de serie temporal; dentro de cada grupo
    elige la combinación "canónica" preferiendo ``(120, 128)`` y luego la de mayor
    cobertura.  Retorna la lista de filas únicas ordenadas por cobertura ascendente.
    """
    preferred_key = (120.0, 128)
    rows_by_key = {
        (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))): row
        for row in aggregated_rows
    }
    keys_by_signature: dict[tuple[tuple[float, float, float], ...], list[tuple[float, int]]] = defaultdict(list)

    for case_key in rows_by_key:
        keys_by_signature[build_timeseries_signature(timeseries_rows, case_key)].append(case_key)

    unique_rows: list[dict[str, str]] = []
    for keys in keys_by_signature.values():
        canonical_key = min(
            keys,
            key=lambda key: (
                0 if key == preferred_key else 1,
                abs(key[0] - preferred_key[0]) + abs(key[1] - preferred_key[1]),
                -float(rows_by_key[key]["house_coverage_pct_mean"]),
                float(rows_by_key[key]["house_e2e_delay_avg_s_mean"]),
                key[0],
                key[1],
            ),
        )
        unique_rows.append(rows_by_key[canonical_key])

    return sorted(
        unique_rows,
        key=lambda row: (
            float(row["house_coverage_pct_mean"]),
            -float(row["house_aoi_avg_s_mean"]),
            float(row[TTL_FIELD]),
            float(row[BUFFER_FIELD]),
        ),
    )


def select_cases(
    aggregated_rows: list[dict[str, str]],
    timeseries_rows: list[dict[str, str]],
) -> list[dict[str, object]]:
    """
    Selecciona hasta 3 casos representativos para las series temporales del dashboard.

    Lógica de selección:
      - "Mejor cobertura": combinación con mayor ``house_coverage_pct_mean``.
      - "Menor cobertura": combinación con menor ``house_coverage_pct_mean``.
      - "Referencia": combinación con cobertura media próxima a la mediana
        (preferiblemente TTL=120 s, buffer=128 paquetes).

    Si todas las combinaciones tienen la misma cobertura se devuelve un único
    "caso representativo".  Se deduplicam casos con comportamientos idénticos.

    Retorna lista de dicts con claves ``kind``, ``name``, ``key`` y ``color``.
    """
    if not aggregated_rows:
        return []

    candidate_rows = build_unique_case_rows(aggregated_rows, timeseries_rows)
    best_row = max(candidate_rows, key=lambda row: float(row["house_coverage_pct_mean"]))
    worst_row = min(candidate_rows, key=lambda row: float(row["house_coverage_pct_mean"]))
    best_key = (float(best_row[TTL_FIELD]), int(float(best_row[BUFFER_FIELD])))
    worst_key = (float(worst_row[TTL_FIELD]), int(float(worst_row[BUFFER_FIELD])))
    if best_key == worst_key:
        return [
            {
                "kind": "representative",
                "name": "Caso representativo",
                "key": best_key,
                "color": DEFAULT_CASE_COLORS["best"],
            }
        ]

    cases = [
        {"kind": "best", "name": "Mejor cobertura", "key": best_key, "color": DEFAULT_CASE_COLORS["best"]},
        {"kind": "worst", "name": "Menor cobertura", "key": worst_key, "color": DEFAULT_CASE_COLORS["worst"]},
    ]

    if len(candidate_rows) >= 3:
        reference_key = select_reference_case(candidate_rows, {best_key, worst_key})
        if reference_key not in {best_key, worst_key}:
            cases.insert(
                1,
                {
                    "kind": "reference",
                    "name": "Referencia",
                    "key": reference_key,
                    "color": DEFAULT_CASE_COLORS["reference"],
                },
            )

    return cases


def summarize_findings(aggregated_rows: list[dict[str, str]]) -> list[str]:
    """
    Genera automáticamente hallazgos textuales para la campaña TTL × Buffer.

    Identifica: mejor cobertura, menor delay E2E, menor AoI, mayor TTL drop
    en sensores y mayor ocupación de buffer en agrobots.
    Retorna lista de strings para el HTML del dashboard.
    """
    best_coverage = max(aggregated_rows, key=lambda row: float(row["house_coverage_pct_mean"]))
    lowest_delay = min(aggregated_rows, key=lambda row: float(row["house_e2e_delay_avg_s_mean"]))
    lowest_aoi = min(aggregated_rows, key=lambda row: float(row["house_aoi_avg_s_mean"]))
    max_sensor_ttl = max(aggregated_rows, key=lambda row: float(row["sensor_ttl_drops_mean"]))
    max_agrobot_buffer = max(aggregated_rows, key=lambda row: float(row["agrobot_avg_buffer_occ_mean"]))

    return [
        f"La mejor cobertura final media aparece en {ttl_label(float(best_coverage[TTL_FIELD]))} con buffer {best_coverage[BUFFER_FIELD]} ({float(best_coverage['house_coverage_pct_mean']):.2f}%).",
        f"El menor delay E2E promedio aparece en {ttl_label(float(lowest_delay[TTL_FIELD]))} con buffer {lowest_delay[BUFFER_FIELD]} ({float(lowest_delay['house_e2e_delay_avg_s_mean']):.2f} s).",
        f"El menor AoI promedio aparece en {ttl_label(float(lowest_aoi[TTL_FIELD]))} con buffer {lowest_aoi[BUFFER_FIELD]} ({float(lowest_aoi['house_aoi_avg_s_mean']):.2f} s).",
        f"El mayor TTL drop en sensores aparece en {ttl_label(float(max_sensor_ttl[TTL_FIELD]))} con buffer {max_sensor_ttl[BUFFER_FIELD]} ({float(max_sensor_ttl['sensor_ttl_drops_mean']):.2f}).",
        f"La mayor ocupacion media de buffer en agrobots aparece en {ttl_label(float(max_agrobot_buffer[TTL_FIELD]))} con buffer {max_agrobot_buffer[BUFFER_FIELD]} ({float(max_agrobot_buffer['agrobot_avg_buffer_occ_mean']):.2f}).",
    ]


def metric_matrix(
    aggregated_rows: list[dict[str, str]],
    metric: str,
) -> dict[tuple[float, int], float]:
    """
    Construye un diccionario ``{(ttl, buffer): metric_mean}`` para un heatmap.

    Extrae la columna ``<metric>_mean`` de cada fila agregada y la indexa
    por la clave compuesta (TTL, buffer).
    """
    matrix: dict[tuple[float, int], float] = {}
    for row in aggregated_rows:
        key = (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD])))
        matrix[key] = float(row[f"{metric}_mean"])
    return matrix


def render_heatmaps_dashboard(
    output_path: Path,
    aggregated_rows: list[dict[str, str]],
    ttl_values: list[float],
    buffer_values: list[int],
) -> None:
    """
    Genera el SVG de heatmaps de la campaña TTL × Buffer.

    Produce un SVG de 1560×1320 px con 9 heatmaps en cuadrícula 3×3:
      Fila 1: cobertura final, delay E2E, AoI.
      Fila 2: TTL drops en sensores, en agrobots, en UGVs.
      Fila 3: buffer medio en sensores, en agrobots, en UGVs.

    Cada celda del heatmap muestra la media sobre las corridas del grupo y
    usa una escala de color interpolada de bajo (azul claro) a alto (azul intenso).
    """
    width = 1560
    height = 1320
    gap = 24
    header_height = 92
    top_margin = header_height
    bottom_margin = gap
    panel_width = (width - 4 * gap) / 3
    panel_height = (height - top_margin - bottom_margin - 2 * gap) / 3

    panel_specs = [
        ("Cobertura final en casa", "Entrega media final a la casa.", "house_coverage_pct", True),
        ("Delay E2E promedio", "Retardo medio final de extremo a extremo.", "house_e2e_delay_avg_s", False),
        ("AoI promedio", "Frescura media observada en la casa.", "house_aoi_avg_s", False),
        ("TTL drops en sensores", "Expiracion de muestras en origen.", "sensor_ttl_drops", False),
        ("TTL drops en agrobots", "Expiracion en el relevo intermedio.", "agrobot_ttl_drops", False),
        ("TTL drops en UGVs", "Expiracion en el ultimo relevo.", "ugv_ttl_drops", False),
        ("Buffer medio en sensores", "Ocupacion promedio SCF en origen.", "sensor_avg_buffer_occ", False),
        ("Buffer medio en agrobots", "Presion promedio del buffer intermedio.", "agrobot_avg_buffer_occ", False),
        ("Buffer medio en UGVs", "Presion promedio del buffer final.", "ugv_avg_buffer_occ", False),
    ]

    panels: list[str] = []
    for index, (title, subtitle, metric, percent) in enumerate(panel_specs):
        column = index % 3
        row = index // 3
        panel_x = gap + column * (panel_width + gap)
        panel_y = top_margin + row * (panel_height + gap)
        panels.append(
            draw_heatmap_panel(
                panel_x,
                panel_y,
                panel_width,
                panel_height,
                title,
                subtitle,
                ttl_values,
                buffer_values,
                metric_matrix(aggregated_rows, metric),
                ttl_label,
                buffer_label,
                metric,
                percent=percent,
            )
        )

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "TTL + Buffer Campaign: Heatmaps", size=24, weight="700"),
        svg_text(28, 56, "Filas: TTL (s). Columnas: buffer (paquetes). Cada celda muestra la media sobre las corridas.", size=12, fill="#475569"),
        *panels,
        "</svg>",
    ]
    output_path.write_text("".join(svg), encoding="utf-8")


def render_timeseries_dashboard(
    output_path: Path,
    timeseries_rows: list[dict[str, str]],
    selected_cases: list[dict[str, object]],
) -> None:
    """
    Genera el SVG de series temporales de casos representativos de TTL × Buffer.

    Produce un SVG de 1440×520 px con 2 paneles:
      1. Cobertura en casa vs tiempo para los casos seleccionados.
      2. AoI promedio vs tiempo para los casos seleccionados.

    Las leyendas muestran el nombre del caso (mejor/referencia/menor) junto con
    la etiqueta ``TTL X s / Buf Y``.
    """
    case_count = len(selected_cases)
    case_id_map = {float(index + 1): case for index, case in enumerate(selected_cases)}
    color_map = {format_float(case_id, 1): case["color"] for case_id, case in case_id_map.items()}

    def metric_series(metric: str) -> dict[float, list[tuple[float, float]]]:
        series: dict[float, list[tuple[float, float]]] = {}
        for case_id, case in case_id_map.items():
            ttl_value, buffer_value = case["key"]
            rows = [
                row
                for row in timeseries_rows
                if math.isclose(float(row[TTL_FIELD]), ttl_value)
                and int(float(row[BUFFER_FIELD])) == buffer_value
            ]
            series[case_id] = [
                (float(row["time_s"]), float(row[f"{metric}_mean"]))
                for row in rows
            ]
        return series

    def legend(case_id: float) -> str:
        case = case_id_map[case_id]
        ttl_value, buffer_value = case["key"]
        return f"{case['name']}: {case_label(ttl_value, buffer_value)}"

    width = 1440
    height = 520
    gap = 24
    header_height = 92
    top_margin = header_height
    bottom_margin = gap
    panel_width = (width - 3 * gap) / 2
    panel_height = height - top_margin - bottom_margin
    panels = [
        draw_line_panel(
            gap,
            top_margin,
            panel_width,
            panel_height,
            "Cobertura en casa vs tiempo",
            f"Comparacion temporal para {case_count} caso{'s' if case_count != 1 else ''} representativo{'s' if case_count != 1 else ''}.",
            metric_series("house_coverage_pct"),
            color_map,
            legend,
            percent=True,
        ),
        draw_line_panel(
            gap * 2 + panel_width,
            top_margin,
            panel_width,
            panel_height,
            "AoI promedio vs tiempo",
            f"Frescura observada para {case_count} caso{'s' if case_count != 1 else ''} representativo{'s' if case_count != 1 else ''}.",
            metric_series("house_aoi_avg_s"),
            color_map,
            legend,
        ),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "TTL + Buffer Campaign: Casos Representativos", size=24, weight="700"),
        svg_text(28, 56, f"Series temporales agregadas para {case_count} combinacion{'es' if case_count != 1 else ''} distintiva{'s' if case_count != 1 else ''} de TTL y buffer.", size=12, fill="#475569"),
        *panels,
        "</svg>",
    ]
    output_path.write_text("".join(svg), encoding="utf-8")


def render_index_html(
    output_path: Path,
    results_root: Path,
    aggregated_rows: list[dict[str, str]],
    summary_rows: list[dict[str, str]],
    selected_cases: list[dict[str, object]],
) -> None:
    """
    Genera el ``index.html`` del dashboard de la campaña TTL × Buffer.

    Incluye hallazgos automáticos, lista de casos temporales seleccionados,
    tabla de resumen por combinación y los SVG de heatmaps y series temporales.
    """
    findings = summarize_findings(aggregated_rows)
    case_items = [
        f"{case['name']}: {case_label(case['key'][0], case['key'][1])}"
        for case in selected_cases
    ]
    render_dashboard_index_html(
        output_path=output_path,
        document_title="TTL + Buffer Campaign Dashboard",
        page_heading="TTL + Buffer Campaign",
        source_path=results_root,
        summary_text=f"Corridas consolidadas: {len(summary_rows)}. La matriz cruza TTL de muestra con capacidad de buffer.",
        links=[
            ("summary_all_runs.csv", "../tables/summary_all_runs.csv"),
            ("summary_aggregated_by_ttl_buffer.csv", "../tables/summary_aggregated_by_ttl_buffer.csv"),
            ("timeseries_mean_selected_cases.csv", "../tables/timeseries_mean_selected_cases.csv"),
        ],
        sections=[
            ("Hallazgos rapidos", build_list_html(findings)),
            ("Casos temporales seleccionados", build_list_html(case_items)),
            ("Resumen por combinacion", build_highlight_table(aggregated_rows)),
            ("Heatmaps", '<img src="heatmaps_dashboard.svg" alt="Dashboard de heatmaps TTL y buffer" />'),
            ("Series temporales", '<img src="timeseries_dashboard.svg" alt="Dashboard temporal TTL y buffer" />'),
        ],
        table_max_width=960,
    )


def main() -> int:
    """
    Punto de entrada del script de visualización de la campaña TTL × Buffer.

    Flujo: resuelve directorio → lee manifest → une summaries 2D → agrega 2D
    → agrega timeseries 2D → selecciona casos representativos → escribe CSVs
    → genera heatmaps SVG → genera timeseries SVG → genera index.html.
    """
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    results_root = resolve_results_dir(args.results_dir, repo_root, CAMPAIGN_NAME)
    dirs = ensure_output_dirs(results_root)

    manifest_rows = read_manifest(results_root)
    summary_rows, summary_fields, ttl_values, buffer_values = merge_summary_rows_2d(results_root, manifest_rows)
    summary_fieldnames = list(manifest_rows[0].keys()) + [field for field in summary_fields if field not in manifest_rows[0]]
    write_csv(dirs["tables"] / "summary_all_runs.csv", summary_fieldnames, summary_rows)

    aggregated_summary_rows, aggregated_summary_fieldnames = aggregate_summary_2d(
        summary_rows,
        summary_fields,
        ttl_values,
        buffer_values,
    )
    write_csv(dirs["tables"] / "summary_aggregated_by_ttl_buffer.csv", aggregated_summary_fieldnames, aggregated_summary_rows)

    aggregated_timeseries_rows, aggregated_timeseries_fieldnames = aggregate_timeseries_2d(
        results_root,
        manifest_rows,
    )

    selected_cases = select_cases(aggregated_summary_rows, aggregated_timeseries_rows)
    selected_case_keys = {case["key"] for case in selected_cases}
    selected_timeseries_rows = [
        {
            **row,
            "case_name": next(
                case["name"]
                for case in selected_cases
                if case["key"] == (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD])))
            ),
            "case_label": case_label(float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))),
        }
        for row in aggregated_timeseries_rows
        if (float(row[TTL_FIELD]), int(float(row[BUFFER_FIELD]))) in selected_case_keys
    ]
    selected_timeseries_fieldnames = ["case_name", "case_label"] + aggregated_timeseries_fieldnames
    write_csv(dirs["tables"] / "timeseries_mean_selected_cases.csv", selected_timeseries_fieldnames, selected_timeseries_rows)

    render_heatmaps_dashboard(dirs["plots"] / "heatmaps_dashboard.svg", aggregated_summary_rows, ttl_values, buffer_values)
    render_timeseries_dashboard(dirs["plots"] / "timeseries_dashboard.svg", aggregated_timeseries_rows, selected_cases)
    render_index_html(dirs["plots"] / "index.html", results_root, aggregated_summary_rows, summary_rows, selected_cases)
    print(f"Graficas y tablas generadas en {results_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
