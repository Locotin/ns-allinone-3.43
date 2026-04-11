#!/usr/bin/env python3

from __future__ import annotations

import argparse
import html
from pathlib import Path

from campaign_plot_utils import (
    aggregate_summary,
    aggregate_timeseries,
    build_list_html,
    build_group_color_map,
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


CAMPAIGN_NAME = "speed_campaign"
GROUP_FIELD = "follower_speed_max"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Consolida y visualiza una carpeta de speed_campaign."
    )
    parser.add_argument(
        "results_dir",
        nargs="?",
        help="Carpeta results/speed_campaign/<timestamp>. Si se omite, usa la mas reciente.",
    )
    return parser.parse_args()


def x_tick_label(speed: float) -> str:
    return f"{speed:.1f}"


def legend_label(speed: float) -> str:
    return f"{speed:.1f} m/s"


def run_count_label(aggregated_rows: list[dict[str, str]]) -> str:
    run_counts = sorted({int(row["runs"]) for row in aggregated_rows})
    if len(run_counts) == 1:
        count = run_counts[0]
        return f"{count} corrida por velocidad" if count == 1 else f"{count} corridas por velocidad"
    return f"entre {run_counts[0]} y {run_counts[-1]} corridas por velocidad"


def build_highlight_table(aggregated_rows: list[dict[str, str]]) -> str:
    headers = ["Velocidad", "Cobertura final", "Delay E2E", "AoI", "PDR Stage 3"]
    rows = []
    for row in aggregated_rows:
        rows.append(
            [
                f'{float(row[GROUP_FIELD]):.1f} m/s',
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
    coverage_means = [float(row["house_coverage_pct_mean"]) for row in aggregated_rows]
    delay_means = [float(row["house_e2e_delay_avg_s_mean"]) for row in aggregated_rows]
    aoi_means = [float(row["house_aoi_avg_s_mean"]) for row in aggregated_rows]
    best_coverage = max(aggregated_rows, key=lambda row: float(row["house_coverage_pct_mean"]))
    best_delay = min(aggregated_rows, key=lambda row: float(row["house_e2e_delay_avg_s_mean"]))
    return [
        f"La cobertura final media en casa varia solo {max(coverage_means) - min(coverage_means):.2f} puntos porcentuales entre velocidades.",
        f"La mejor cobertura media aparece en {float(best_coverage[GROUP_FIELD]):.1f} m/s con {float(best_coverage['house_coverage_pct_mean']):.2f}%.",
        f"El menor delay E2E promedio aparece en {float(best_delay[GROUP_FIELD]):.1f} m/s con {float(best_delay['house_e2e_delay_avg_s_mean']):.2f} s.",
        f"El AoI promedio se mueve en un rango acotado de {min(aoi_means):.2f} s a {max(aoi_means):.2f} s.",
    ]


def render_summary_dashboard(
    output_path: Path,
    summary_rows: list[dict[str, str]],
    aggregated_rows: list[dict[str, str]],
    group_values: list[float],
) -> None:
    speed_colors = build_group_color_map(group_values)
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
    sensor_buffer_means: dict[float, float] = {}
    agrobot_buffer_means: dict[float, float] = {}
    ugv_buffer_means: dict[float, float] = {}
    sensor_ttl_means: dict[float, float] = {}
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
        sensor_buffer_means[group_value] = float(row["sensor_avg_buffer_occ_mean"])
        agrobot_buffer_means[group_value] = float(row["agrobot_avg_buffer_occ_mean"])
        ugv_buffer_means[group_value] = float(row["ugv_avg_buffer_occ_mean"])
        sensor_ttl_means[group_value] = float(row["sensor_ttl_drops_mean"])
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
            "Puntos por corrida y linea de media por velocidad.",
            group_values,
            run_values["coverage"],
            coverage_means,
            speed_colors,
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
            speed_colors,
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
            speed_colors,
            x_tick_label,
        ),
        draw_grouped_bar_panel(
            first_col_x,
            third_row_y,
            panel_width,
            panel_height,
            "Ocupacion promedio de buffer",
            "Comparacion por capa de la red.",
            group_values,
            [
                ("Sensor", DEFAULT_LAYER_COLORS["sensor"], sensor_buffer_means),
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
            "Drops por TTL",
            "La capa sensor domina la expiracion de muestras.",
            group_values,
            [
                ("Sensor", DEFAULT_LAYER_COLORS["sensor"], sensor_ttl_means),
                ("Agrobot", DEFAULT_LAYER_COLORS["agrobot"], agrobot_ttl_means),
                ("UGV", DEFAULT_LAYER_COLORS["ugv"], ugv_ttl_means),
            ],
            x_tick_label,
        ),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "Speed Campaign: Metricas Finales", size=24, weight="700"),
        svg_text(28, 56, f"Campana consolidada en {len(group_values)} velocidades con {run_count_label(aggregated_rows)}.", size=12, fill="#475569"),
        *panels,
        "</svg>",
    ]
    output_path.write_text("".join(svg), encoding="utf-8")


def render_timeseries_dashboard(
    output_path: Path,
    timeseries_rows: list[dict[str, str]],
    group_values: list[float],
) -> None:
    speed_colors = build_group_color_map(group_values)

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
            "Media por velocidad sobre las corridas agregadas.",
            metric_series("house_coverage_pct"),
            speed_colors,
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
            speed_colors,
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
            speed_colors,
            legend_label,
        ),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        svg_rect(0, 0, width, height, fill="#f8fafc"),
        svg_text(28, 34, "Speed Campaign: Dinamica Temporal", size=24, weight="700"),
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
    findings = summarize_findings(aggregated_rows)
    speed_summary = ", ".join(legend_label(group_value) for group_value in group_values)
    render_dashboard_index_html(
        output_path=output_path,
        document_title="Speed Campaign Dashboard",
        page_heading="Speed Campaign",
        source_path=results_root,
        summary_text=f"Corridas consolidadas: {len(summary_rows)}. Velocidades analizadas: {speed_summary}. Configuracion: {run_count_label(aggregated_rows)}.",
        links=[
            ("summary_all_runs.csv", "../tables/summary_all_runs.csv"),
            ("summary_aggregated_by_speed.csv", "../tables/summary_aggregated_by_speed.csv"),
            ("timeseries_mean_by_speed_time.csv", "../tables/timeseries_mean_by_speed_time.csv"),
        ],
        sections=[
            ("Hallazgos rapidos", build_list_html(findings)),
            ("Resumen por velocidad", build_highlight_table(aggregated_rows)),
            ("Metricas finales", '<img src="summary_dashboard.svg" alt="Dashboard de metricas finales" />'),
            ("Dinamica temporal", '<img src="timeseries_dashboard.svg" alt="Dashboard temporal" />'),
        ],
    )


def main() -> int:
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
    write_csv(dirs["tables"] / "summary_aggregated_by_speed.csv", aggregated_summary_fieldnames, aggregated_summary_rows)

    aggregated_timeseries_rows, aggregated_timeseries_fieldnames = aggregate_timeseries(
        results_root,
        manifest_rows,
        GROUP_FIELD,
        group_values,
    )
    write_csv(dirs["tables"] / "timeseries_mean_by_speed_time.csv", aggregated_timeseries_fieldnames, aggregated_timeseries_rows)

    render_summary_dashboard(dirs["plots"] / "summary_dashboard.svg", summary_rows, aggregated_summary_rows, group_values)
    render_timeseries_dashboard(dirs["plots"] / "timeseries_dashboard.svg", aggregated_timeseries_rows, group_values)
    render_index_html(dirs["plots"] / "index.html", results_root, aggregated_summary_rows, summary_rows, group_values)
    print(f"Graficas y tablas generadas en {results_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
