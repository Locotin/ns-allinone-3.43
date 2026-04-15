#!/usr/bin/env python3
"""
plot_contact_range_campaign.py
===============================
Script de análisis y visualización para la campaña de rango de contacto.

Lee los artefactos generados por ``run_contact_range_campaign.py``, consolida
las métricas de todas las corridas y produce un dashboard HTML+SVG autocontenido.

Salidas generadas
-----------------
Dentro de ``results/contact_range_campaign/<timestamp>/``:

``tables/``
    - ``summary_all_runs.csv``
        Tabla con una fila por corrida que combina columnas del manifest
        y del archivo ``summary_<label>.csv`` de adhoc.cc.
    - ``summary_aggregated_by_range.csv``
        Estadísticas descriptivas (media, desv. estándar, mín, máx)
        de cada métrica agrupadas por valor de rango de contacto.
    - ``timeseries_mean_by_range_time.csv``
        Series temporales medias por (rango, instante) sobre todas las corridas.

``plots/``
    - ``summary_dashboard.svg``
        Dashboard SVG con 6 paneles de métricas finales (cobertura, delay E2E,
        PDR por etapa, AoI, ocupación de buffer y drops por TTL).
    - ``timeseries_dashboard.svg``
        Dashboard SVG con 3 paneles de series temporales (cobertura en casa,
        cobertura en agrobots, AoI media).
    - ``index.html``
        Dashboard HTML con hallazgos automáticos, tabla resumen y visualizaciones.

Uso
---
    python3 scratch/plot_contact_range_campaign.py
    python3 scratch/plot_contact_range_campaign.py results/contact_range_campaign/20240101_120000

Si no se indica directorio, usa el más reciente en ``results/contact_range_campaign/``.
"""

from __future__ import annotations

import argparse
import html
from pathlib import Path

from campaign_plot_utils import (
    aggregate_summary,
    aggregate_timeseries,
    build_list_html,
    build_group_color_map,
    compact_label,
    DEFAULT_LAYER_COLORS,
    DEFAULT_STAGE_COLORS,
    draw_grouped_bar_panel,
    draw_line_panel,
    draw_scatter_mean_panel,
    ensure_output_dirs,
    format_metric_value,
    merge_summary_rows,
    read_manifest,
    render_dashboard_index_html,
    resolve_results_dir,
    svg_rect,
    svg_text,
    write_csv,
)


CAMPAIGN_NAME = "contact_range_campaign"
GROUP_FIELD = "contact_range"


def parse_args() -> argparse.Namespace:
    """Parsea el argumento opcional ``results_dir`` (directorio de la campaña a visualizar)."""
    parser = argparse.ArgumentParser(
        description="Consolida y visualiza una carpeta de contact_range_campaign."
    )
    parser.add_argument(
        "results_dir",
        nargs="?",
        help="Carpeta results/contact_range_campaign/<timestamp>. Si se omite, usa la mas reciente.",
    )
    return parser.parse_args()


def x_tick_label(contact_range: float) -> str:
    """Etiqueta compacta del eje X (sin unidad) para un valor de rango de contacto."""
    return compact_label(contact_range)


def legend_label(contact_range: float) -> str:
    """Etiqueta de leyenda con unidades (ej. ``"52 m"``) para un valor de rango."""
    return f"{compact_label(contact_range)} m"


def run_count_label(aggregated_rows: list[dict[str, str]]) -> str:
    """Genera una cadena descriptiva del número de corridas por rango (para subtítulos)."""
    run_counts = sorted({int(row["runs"]) for row in aggregated_rows})
    if len(run_counts) == 1:
        count = run_counts[0]
        return f"{count} corrida por rango" if count == 1 else f"{count} corridas por rango"
    return f"entre {run_counts[0]} y {run_counts[-1]} corridas por rango"


def build_highlight_table(aggregated_rows: list[dict[str, str]]) -> str:
    """
    Genera la tabla HTML de resumen con las métricas clave por rango de contacto.

    Columnas: Rango (m), Cobertura final (%), Delay E2E (s), AoI (s), PDR Stage 3 (%).
    """
    headers = ["Rango", "Cobertura final", "Delay E2E", "AoI", "PDR Stage 3"]
    rows = []
    for row in aggregated_rows:
        rows.append(
            [
                legend_label(float(row[GROUP_FIELD])),
                format_metric_value("house_coverage_pct", float(row["house_coverage_pct_mean"])),
                format_metric_value("house_e2e_delay_avg_s", float(row["house_e2e_delay_avg_s_mean"])),
                format_metric_value("house_aoi_avg_s", float(row["house_aoi_avg_s_mean"])),
                format_metric_value("stage3_pdr_pct", float(row["stage3_pdr_pct_mean"])),
            ]
        )

    parts = ['<table class="metric-table"><thead><tr>']
    for header in headers:
        parts.append(f"<th>{html.escape(header)}</th>")
    parts.append("</tr></thead><tbody>")
    for row in rows:
        parts.append("<tr>")
        for cell in row:
            parts.append(f"<td>{html.escape(cell)}</td>")
        parts.append("</tr>")
    parts.append("</tbody></table>")
    return "".join(parts)


def summarize_findings(aggregated_rows: list[dict[str, str]]) -> list[str]:
    """
    Genera automáticamente hallazgos textuales a partir de las filas agregadas.

    Identifica el mejor rango para cobertura, menor delay E2E, menor AoI,
    la variación total de cobertura entre extremos, y los mejores PDR de Stage 2 y 3.
    Retorna una lista de strings listos para mostrar como ``<li>`` en el dashboard.
    """
    best_coverage = max(aggregated_rows, key=lambda row: float(row["house_coverage_pct_mean"]))
    best_delay = min(aggregated_rows, key=lambda row: float(row["house_e2e_delay_avg_s_mean"]))
    best_aoi = min(aggregated_rows, key=lambda row: float(row["house_aoi_avg_s_mean"]))
    weakest_stage3 = min(aggregated_rows, key=lambda row: float(row["stage3_pdr_pct_mean"]))
    strongest_stage2 = max(aggregated_rows, key=lambda row: float(row["stage2_pdr_pct_mean"]))
    sorted_rows = sorted(aggregated_rows, key=lambda row: float(row[GROUP_FIELD]))
    low_range = sorted_rows[0]
    high_range = sorted_rows[-1]

    findings: list[str] = []
    findings.append(
        f"La mejor cobertura media en casa aparece en {legend_label(float(best_coverage[GROUP_FIELD]))} con {float(best_coverage['house_coverage_pct_mean']):.2f}%."
    )
    findings.append(
        f"El menor delay E2E medio aparece en {legend_label(float(best_delay[GROUP_FIELD]))} ({float(best_delay['house_e2e_delay_avg_s_mean']):.2f} s) y el menor AoI medio en {legend_label(float(best_aoi[GROUP_FIELD]))} ({float(best_aoi['house_aoi_avg_s_mean']):.2f} s)."
    )
    findings.append(
        f"Entre {legend_label(float(low_range[GROUP_FIELD]))} y {legend_label(float(high_range[GROUP_FIELD]))}, la cobertura media cambia {float(high_range['house_coverage_pct_mean']) - float(low_range['house_coverage_pct_mean']):.2f} puntos porcentuales."
    )
    findings.append(
        f"El mejor PDR Stage 2 aparece en {legend_label(float(strongest_stage2[GROUP_FIELD]))} ({float(strongest_stage2['stage2_pdr_pct_mean']):.2f}%), mientras que el Stage 3 mas bajo aparece en {legend_label(float(weakest_stage3[GROUP_FIELD]))} ({float(weakest_stage3['stage3_pdr_pct_mean']):.2f}%)."
    )
    return findings


def render_summary_dashboard(
    output_path: Path,
    summary_rows: list[dict[str, str]],
    aggregated_rows: list[dict[str, str]],
    group_values: list[float],
) -> None:
    """
    Genera el SVG de métricas finales de la campaña de rango de contacto.

    Produce un SVG de 1400×1120 px con 6 paneles en cuadrícula 2×3:
      1. Scatter + media de cobertura final en casa por rango.
      2. Scatter + media de delay E2E por rango.
      3. Barras agrupadas de PDR Stage 2 vs Stage 3.
      4. Scatter + media de AoI por rango.
      5. Barras de ocupación de buffer (agrobot vs UGV).
      6. Barras de drops por TTL (agrobot vs UGV).

    Escribe el SVG directamente en ``output_path``.
    """
    range_colors = build_group_color_map(group_values)
    run_values = {
        "coverage": {group_value: [] for group_value in group_values},
        "delay": {group_value: [] for group_value in group_values},
        "aoi": {group_value: [] for group_value in group_values},
    }
    coverage_means: dict[float, float] = {}
    delay_means: dict[float, float] = {}
    aoi_means: dict[float, float] = {}
    stage2_pdr_means: dict[float, float] = {}
    stage3_pdr_means: dict[float, float] = {}
    agrobot_buffer_means: dict[float, float] = {}
    ugv_buffer_means: dict[float, float] = {}
    agrobot_ttl_means: dict[float, float] = {}
    ugv_ttl_means: dict[float, float] = {}

    for row in summary_rows:
        group_value = float(row[GROUP_FIELD])
        run_values["coverage"][group_value].append(float(row["house_coverage_pct"]))
        run_values["delay"][group_value].append(float(row["house_e2e_delay_avg_s"]))
        run_values["aoi"][group_value].append(float(row["house_aoi_avg_s"]))

    for row in aggregated_rows:
        group_value = float(row[GROUP_FIELD])
        coverage_means[group_value] = float(row["house_coverage_pct_mean"])
        delay_means[group_value] = float(row["house_e2e_delay_avg_s_mean"])
        aoi_means[group_value] = float(row["house_aoi_avg_s_mean"])
        stage2_pdr_means[group_value] = float(row["stage2_pdr_pct_mean"])
        stage3_pdr_means[group_value] = float(row["stage3_pdr_pct_mean"])
        agrobot_buffer_means[group_value] = float(row["agrobot_avg_buffer_occ_mean"])
        ugv_buffer_means[group_value] = float(row["ugv_avg_buffer_occ_mean"])
        agrobot_ttl_means[group_value] = float(row["agrobot_ttl_drops_mean"])
        ugv_ttl_means[group_value] = float(row["ugv_ttl_drops_mean"])

    width = 1400
    height = 1120
    gap = 24
    header_height = 92
    left_margin = gap
    top_margin = header_height
    bottom_margin = gap
    panel_width = (width - 3 * gap) / 2
    panel_height = (height - top_margin - bottom_margin - 2 * gap) / 3
    first_col_x = left_margin
    second_col_x = left_margin + panel_width + gap
    first_row_y = top_margin
    second_row_y = top_margin + panel_height + gap
    third_row_y = top_margin + 2 * (panel_height + gap)
    panels = [
        draw_scatter_mean_panel(
            first_col_x,
            first_row_y,
            panel_width,
            panel_height,
            "Cobertura final en casa",
            "Puntos por corrida y linea de media por rango.",
            group_values,
            run_values["coverage"],
            coverage_means,
            range_colors,
            x_tick_label,
            percent=True,
        ),
        draw_scatter_mean_panel(
            second_col_x,
            first_row_y,
            panel_width,
            panel_height,
            "Delay extremo a extremo",
            "Promedio final en casa por corrida.",
            group_values,
            run_values["delay"],
            delay_means,
            range_colors,
            x_tick_label,
        ),
        draw_grouped_bar_panel(
            first_col_x,
            second_row_y,
            panel_width,
            panel_height,
            "PDR en etapas finales",
            "Comparacion de Stage 2 y Stage 3.",
            group_values,
            [
                ("Stage 2", DEFAULT_STAGE_COLORS["stage2"], stage2_pdr_means),
                ("Stage 3", DEFAULT_STAGE_COLORS["stage3"], stage3_pdr_means),
            ],
            x_tick_label,
            percent=True,
        ),
        draw_scatter_mean_panel(
            second_col_x,
            second_row_y,
            panel_width,
            panel_height,
            "Age of Information",
            "AoI promedio final observado en la casa.",
            group_values,
            run_values["aoi"],
            aoi_means,
            range_colors,
            x_tick_label,
        ),
        draw_grouped_bar_panel(
            first_col_x,
            third_row_y,
            panel_width,
            panel_height,
            "Buffers medios en relays",
            "Cambio de presion entre agrobots y UGVs.",
            group_values,
            [
                ("Agrobot", DEFAULT_LAYER_COLORS["agrobot"], agrobot_buffer_means),
                ("UGV", DEFAULT_LAYER_COLORS["ugv"], ugv_buffer_means),
            ],
            x_tick_label,
        ),
        draw_grouped_bar_panel(
            second_col_x,
            third_row_y,
            panel_width,
            panel_height,
            "Drops por TTL en relays",
            "Expiracion de muestras entre niveles de relevo.",
            group_values,
            [
                ("Agrobot", DEFAULT_LAYER_COLORS["agrobot"], agrobot_ttl_means),
                ("UGV", DEFAULT_LAYER_COLORS["ugv"], ugv_ttl_means),
            ],
            x_tick_label,
        ),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "Contact Range Campaign: Metricas Finales", size=24, weight="700"),
        svg_text(28, 56, f"Campana consolidada en {len(group_values)} rangos de contacto con {run_count_label(aggregated_rows)}.", size=12, fill="#475569"),
        *panels,
        "</svg>",
    ]
    output_path.write_text("".join(svg), encoding="utf-8")


def render_timeseries_dashboard(
    output_path: Path,
    timeseries_rows: list[dict[str, str]],
    group_values: list[float],
) -> None:
    """
    Genera el SVG de dinámica temporal de la campaña de rango de contacto.

    Produce un SVG de 1440×520 px con 3 paneles de series temporales:
      1. Cobertura en casa vs tiempo (media por rango).
      2. Cobertura en agrobots vs tiempo (recepción intermedia).
      3. AoI promedio vs tiempo (frescura de información).

    Las series muestran promedios agregados sobre todas las corridas de cada rango.
    """
    range_colors = build_group_color_map(group_values)

    def metric_series(metric: str) -> dict[float, list[tuple[float, float]]]:
        series: dict[float, list[tuple[float, float]]] = {}
        for group_value in group_values:
            rows = [row for row in timeseries_rows if float(row[GROUP_FIELD]) == group_value]
            series[group_value] = [
                (float(row["time_s"]), float(row[f"{metric}_mean"]))
                for row in rows
            ]
        return series

    width = 1440
    height = 520
    gap = 24
    header_height = 92
    top_margin = header_height
    bottom_margin = gap
    panel_width = (width - 4 * gap) / 3
    panel_height = height - top_margin - bottom_margin
    panels = [
        draw_line_panel(
            gap,
            top_margin,
            panel_width,
            panel_height,
            "Cobertura en casa vs tiempo",
            "Media por rango sobre las corridas agregadas.",
            metric_series("house_coverage_pct"),
            range_colors,
            legend_label,
            percent=True,
        ),
        draw_line_panel(
            gap * 2 + panel_width,
            top_margin,
            panel_width,
            panel_height,
            "Cobertura en agrobots vs tiempo",
            "Recepcion intermedia antes de la entrega final.",
            metric_series("agrobot_coverage_pct"),
            range_colors,
            legend_label,
            percent=True,
        ),
        draw_line_panel(
            gap * 3 + panel_width * 2,
            top_margin,
            panel_width,
            panel_height,
            "AoI promedio vs tiempo",
            "Frescura de informacion observada en la casa.",
            metric_series("house_aoi_avg_s"),
            range_colors,
            legend_label,
        ),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "Contact Range Campaign: Dinamica Temporal", size=24, weight="700"),
        svg_text(28, 56, "Las series temporales llegan hasta 299 s por la periodicidad de muestreo.", size=12, fill="#475569"),
        *panels,
        "</svg>",
    ]
    output_path.write_text("".join(svg), encoding="utf-8")


def render_index_html(
    output_path: Path,
    results_root: Path,
    aggregated_rows: list[dict[str, str]],
    summary_rows: list[dict[str, str]],
    group_values: list[float],
) -> None:
    """
    Genera el ``index.html`` del dashboard de la campaña de rango de contacto.

    Combina hallazgos automáticos, tabla de resumen por rango y los SVG de
    métricas finales y dinámica temporal en un HTML con tarjetas estilizadas.
    """
    findings = summarize_findings(aggregated_rows)
    range_summary = ", ".join(legend_label(group_value) for group_value in group_values)
    render_dashboard_index_html(
        output_path=output_path,
        document_title="Contact Range Campaign Dashboard",
        page_heading="Contact Range Campaign",
        source_path=results_root,
        summary_text=f"Corridas consolidadas: {len(summary_rows)}. Rangos analizados: {range_summary}. Configuracion: {run_count_label(aggregated_rows)}.",
        links=[
            ("summary_all_runs.csv", "../tables/summary_all_runs.csv"),
            ("summary_aggregated_by_range.csv", "../tables/summary_aggregated_by_range.csv"),
            ("timeseries_mean_by_range_time.csv", "../tables/timeseries_mean_by_range_time.csv"),
        ],
        sections=[
            ("Hallazgos rapidos", build_list_html(findings)),
            ("Resumen por rango", build_highlight_table(aggregated_rows)),
            ("Metricas finales", '<img src="summary_dashboard.svg" alt="Dashboard de metricas finales" />'),
            ("Dinamica temporal", '<img src="timeseries_dashboard.svg" alt="Dashboard temporal" />'),
        ],
    )


def main() -> int:
    """
    Punto de entrada del script de visualización.

    Flujo de ejecución
    ------------------
    1. Resuelve el directorio de resultados de la campaña.
    2. Lee el manifest y une las filas de resumen de cada corrida.
    3. Calcula estadísticas agregadas por rango (summary y timeseries).
    4. Escribe los CSVs consolidados en ``tables/``.
    5. Genera los SVG de métricas finales y temporales en ``plots/``.
    6. Genera el ``index.html`` del dashboard.
    """
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    results_root = resolve_results_dir(args.results_dir, repo_root, CAMPAIGN_NAME)
    dirs = ensure_output_dirs(results_root)

    manifest_rows = read_manifest(results_root)
    summary_rows, summary_fields, group_values = merge_summary_rows(results_root, manifest_rows, GROUP_FIELD)
    summary_fieldnames = list(manifest_rows[0].keys()) + [field for field in summary_fields if field not in manifest_rows[0]]
    write_csv(dirs["tables"] / "summary_all_runs.csv", summary_fieldnames, summary_rows)

    aggregated_summary_rows, aggregated_summary_fieldnames = aggregate_summary(
        summary_rows,
        summary_fields,
        GROUP_FIELD,
        group_values,
    )
    write_csv(dirs["tables"] / "summary_aggregated_by_range.csv", aggregated_summary_fieldnames, aggregated_summary_rows)

    aggregated_timeseries_rows, aggregated_timeseries_fieldnames = aggregate_timeseries(
        results_root,
        manifest_rows,
        GROUP_FIELD,
        group_values,
    )
    write_csv(dirs["tables"] / "timeseries_mean_by_range_time.csv", aggregated_timeseries_fieldnames, aggregated_timeseries_rows)

    render_summary_dashboard(dirs["plots"] / "summary_dashboard.svg", summary_rows, aggregated_summary_rows, group_values)
    render_timeseries_dashboard(dirs["plots"] / "timeseries_dashboard.svg", aggregated_timeseries_rows, group_values)
    render_index_html(dirs["plots"] / "index.html", results_root, aggregated_summary_rows, summary_rows, group_values)
    print(f"Graficas y tablas generadas en {results_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
