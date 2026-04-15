#!/usr/bin/env python3
"""
campaign_plot_utils.py
=======================
Biblioteca compartida de utilidades de análisis estadístico y visualización SVG
para las campañas paramétricas de la simulación MANET de agricultura de precisión.

Este módulo es importado por los tres scripts de visualización:
  - ``plot_contact_range_campaign.py``
  - ``plot_speed_campaign.py``
  - ``plot_ttl_buffer_campaign.py``

Organización del módulo
-----------------------

Paleta y tema visual
    Constantes de colores usadas en todos los dashboards para mantener coherencia
    visual entre campañas (``CATEGORICAL_PALETTE``, ``DEFAULT_STAGE_COLORS``,
    ``DEFAULT_LAYER_COLORS``, ``DEFAULT_CASE_COLORS``, ``THEME_*``).

Entrada/Salida de archivos
    Funciones para localizar directorios de resultados, leer manifests CSV, leer
    archivos de resumen de corrida individual y escribir CSVs de salida.

Procesamiento estadístico
    Funciones para combinar filas de resumen con el manifest (``merge_summary_rows``),
    calcular estadísticas por grupo (media, desv. estándar, mín, máx) sobre corridas
    múltiples (``aggregate_summary``, ``aggregate_timeseries``).

Generación de HTML
    Funciones para producir el ``index.html`` del dashboard con tarjetas, tablas de
    métricas y enlaces a los SVG y CSV generados (``render_dashboard_index_html``,
    ``build_links_html``, ``build_list_html``).

Primitivas SVG
    Funciones de bajo nivel para construir elementos SVG sin dependencias externas:
    ``svg_text``, ``svg_rect``, ``svg_line``, ``svg_circle``, ``svg_polyline``.

Utilidades de escala y coordenadas
    ``linear_map``: interpolación lineal entre dominios.
    ``padded_domain``: calcula los límites del eje Y con margen automático.
    ``spread_offsets``: distribuye puntos de dispersión horizontalmente.

Componentes de panel SVG
    ``draw_panel_frame``: marco con título y subtítulo para un panel.
    ``draw_y_axis``: eje Y con líneas de cuadrícula y etiquetas.
    ``draw_scatter_mean_panel``: scatter de corridas individuales + línea de media.
    ``draw_grouped_bar_panel``: barras agrupadas por categoría y serie.
    ``draw_line_panel``: series temporales de líneas múltiples con leyenda.
    ``draw_heatmap_panel``: mapa de calor 2D con escala de color interpolada.
"""

from __future__ import annotations

import csv
import html
import math
import statistics
from collections import defaultdict
from pathlib import Path


CATEGORICAL_PALETTE = (
    "#475569",
    "#0f766e",
    "#c2410c",
    "#1d4ed8",
    "#be123c",
)
DEFAULT_STAGE_COLORS = {
    "stage2": CATEGORICAL_PALETTE[1],
    "stage3": CATEGORICAL_PALETTE[2],
}
DEFAULT_LAYER_COLORS = {
    "sensor": CATEGORICAL_PALETTE[0],
    "agrobot": CATEGORICAL_PALETTE[1],
    "ugv": CATEGORICAL_PALETTE[2],
}
DEFAULT_CASE_COLORS = {
    "best": CATEGORICAL_PALETTE[1],
    "reference": CATEGORICAL_PALETTE[3],
    "worst": CATEGORICAL_PALETTE[2],
}
HEATMAP_LOW_COLOR = "#eff6ff"
HEATMAP_HIGH_COLOR = CATEGORICAL_PALETTE[3]
THEME_TEXT_COLOR = "#0f172a"
THEME_MUTED_TEXT = "#334155"
THEME_SOFT_MUTED_TEXT = "#475569"
THEME_BACKGROUND = "#f8fafc"
THEME_CARD_BACKGROUND = "#ffffff"
THEME_BORDER = "#cbd5e1"
THEME_ROW_BORDER = "#e2e8f0"
THEME_LINK = CATEGORICAL_PALETTE[3]


def newest_campaign_dir(repo_root: Path, campaign_name: str) -> Path:
    """
    Encuentra el directorio de campaña más reciente en ``results/<campaign_name>/``.

    El criterio de "más reciente" es el orden lexicográfico del nombre de la
    subcarpeta (que corresponde a timestamps ``YYYYMMDD_HHMMSS``).

    Lanza ``FileNotFoundError`` si no existe ninguna campaña.
    """
    base_dir = repo_root / "results" / campaign_name
    candidates = sorted(
        path for path in base_dir.iterdir() if path.is_dir() and (path / "manifest.csv").exists()
    )
    if not candidates:
        raise FileNotFoundError(f"No hay campañas en {base_dir}")
    return candidates[-1]


def resolve_results_dir(results_dir: str | None, repo_root: Path, campaign_name: str) -> Path:
    """
    Resuelve el directorio de resultados a usar.

    Si ``results_dir`` es una cadena no vacía, la convierte a ``Path`` absoluto.
    Si es ``None``, delega en ``newest_campaign_dir`` para usar la campaña más reciente.
    """
    if results_dir:
        return Path(results_dir).expanduser().resolve()
    return newest_campaign_dir(repo_root, campaign_name)


def ensure_output_dirs(results_root: Path) -> dict[str, Path]:
    """
    Crea los subdirectorios de salida para tablas y gráficas dentro de la campaña.

    Retorna un diccionario con claves ``"root"``, ``"tables"`` y ``"plots"``.
    Los directorios se crean si no existen (``parents=True, exist_ok=True``).
    """
    dirs = {
        "root": results_root,
        "tables": results_root / "tables",
        "plots": results_root / "plots",
    }
    for directory in dirs.values():
        directory.mkdir(parents=True, exist_ok=True)
    return dirs


def resolve_artifact_path(results_root: Path, raw_path: str) -> Path:
    """
    Convierte la ruta de un artefacto registrada en el manifest a un ``Path`` absoluto.

    Si ``raw_path`` ya es absoluto lo devuelve tal cual; si es relativo, lo une
    con ``results_root``. Esto permite que el manifest funcione correctamente
    aunque los archivos se hayan generado en una máquina diferente.
    """
    path = Path(raw_path)
    if path.is_absolute():
        return path
    return (results_root / path).resolve()


def read_manifest(results_root: Path) -> list[dict[str, str]]:
    """
    Lee el ``manifest.csv`` de una carpeta de resultados de campaña.

    Retorna una lista de diccionarios, uno por corrida, con los campos del manifest.
    """
    manifest_path = results_root / "manifest.csv"
    with manifest_path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def read_single_row_csv(path: Path) -> dict[str, str]:
    """
    Lee un CSV que contiene exactamente una fila de datos (más cabecera).

    Lanza ``ValueError`` si el archivo tiene cero o más de una fila de datos.
    Usado para leer los archivos ``summary_<label>.csv`` producidos por adhoc.cc.
    """
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 1:
        raise ValueError(f"Se esperaba una sola fila de datos en {path}, se encontraron {len(rows)}")
    return rows[0]


def is_float(value: str) -> bool:
    """Retorna ``True`` si ``value`` puede convertirse a ``float`` sin error."""
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False


def format_float(value: float, digits: int = 6) -> str:
    """Formatea un float con un número fijo de decimales para almacenamiento en CSV."""
    return f"{value:.{digits}f}"


def format_metric_value(metric: str, value: float) -> str:
    """
    Formatea un valor de métrica con la unidad apropiada según su nombre.

    - Sufijo ``_pct`` → porcentaje con dos decimales (ej. ``"87.34%"``).
    - Sufijo ``_s`` → segundos con dos decimales (ej. ``"12.50 s"``).
    - Cualquier otro → dos decimales sin unidad.
    """
    if metric.endswith("_pct"):
        return f"{value:.2f}%"
    if metric.endswith("_s"):
        return f"{value:.2f} s"
    return f"{value:.2f}"


def compact_label(value: float, digits_if_needed: int = 1) -> str:
    """
    Devuelve una etiqueta compacta para un valor numérico en ejes y leyendas.

    Si el valor es prácticamente un entero lo muestra sin decimales (ej. ``52``).
    Si tiene parte fraccionaria relevante, usa ``digits_if_needed`` decimales.
    """
    rounded = round(value)
    if math.isclose(value, rounded):
        return str(int(rounded))
    return f"{value:.{digits_if_needed}f}"


def build_group_color_map(group_values: list[float]) -> dict[str, str]:
    """
    Asigna un color de la paleta categórica a cada valor de grupo único.

    La clave del diccionario es el valor formateado con un decimal (ej. ``"52.5"``).
    Los colores se asignan en orden creciente y se reciclan si hay más grupos que colores.
    """
    ordered = sorted(set(group_values))
    return {
        format_float(group_value, 1): CATEGORICAL_PALETTE[index % len(CATEGORICAL_PALETTE)]
        for index, group_value in enumerate(ordered)
    }


def merge_summary_rows(
    results_root: Path,
    manifest_rows: list[dict[str, str]],
    group_field: str,
) -> tuple[list[dict[str, str]], list[str], list[float]]:
    """
    Une las filas del manifest con los archivos ``summary_<label>.csv`` de cada corrida.

    Por cada fila del manifest, lee el CSV de resumen correspondiente y agrega sus
    columnas a la fila del manifest (sin sobreescribir columnas existentes).

    Parámetros
    ----------
    results_root : Path
        Raíz del directorio de la campaña.
    manifest_rows : list[dict]
        Filas del manifest.csv (leídas por ``read_manifest``).
    group_field : str
        Nombre de la columna que identifica el grupo experimental
        (ej. ``"contact_range"`` o ``"follower_speed_max"``).

    Retorna
    -------
    Tupla de tres elementos:
      - all_rows: filas combinadas manifest + summary, una por corrida.
      - summary_fields: nombres de columnas que vienen del summary CSV.
      - group_values: lista ordenada y deduplicada de los valores del grupo.
    """
    all_rows: list[dict[str, str]] = []
    summary_fields: list[str] = []
    group_values: list[float] = []

    for manifest_row in manifest_rows:
        group_value = float(manifest_row[group_field])
        summary_path = resolve_artifact_path(results_root, manifest_row["summary_file"])
        summary_row = read_single_row_csv(summary_path)
        if not summary_fields:
            summary_fields = list(summary_row.keys())

        merged_row: dict[str, str] = {}
        for key, value in manifest_row.items():
            merged_row[key] = value
        for key in summary_fields:
            if key not in merged_row:
                merged_row[key] = summary_row[key]

        all_rows.append(merged_row)
        group_values.append(group_value)

    return all_rows, summary_fields, sorted(set(group_values))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    """Escribe una lista de diccionarios como CSV con cabecera en ``path``."""
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_links_html(links: list[tuple[str, str]]) -> str:
    """
    Genera un bloque HTML ``<div class="links">`` con enlaces.

    Parámetros
    ----------
    links : list of (label, href)
        Cada tupla produce un ``<a>`` con el texto ``label`` apuntando a ``href``.
    """
    parts = ['<div class="links">']
    for label, href in links:
        parts.append(f'<a href="{html.escape(href, quote=True)}">{html.escape(label)}</a>')
    parts.append("</div>")
    return "".join(parts)


def build_list_html(items: list[str]) -> str:
    """Genera una lista HTML ``<ul>`` con un ``<li>`` por cada elemento de ``items``."""
    return "<ul>" + "".join(f"<li>{html.escape(item)}</li>" for item in items) + "</ul>"


def render_dashboard_index_html(
    output_path: Path,
    document_title: str,
    page_heading: str,
    source_path: Path,
    summary_text: str,
    links: list[tuple[str, str]],
    sections: list[tuple[str, str]],
    table_max_width: int = 760,
) -> None:
    """
    Genera el archivo ``index.html`` del dashboard de una campaña.

    El HTML usa un diseño de tarjetas (``<div class="card">``) con CSS embebido
    compatible con la paleta del proyecto. La primera tarjeta actúa como encabezado
    con la ruta de origen, texto resumen y enlaces a CSVs; las siguientes tarjetas
    contienen las secciones de análisis (hallazgos, tablas, imágenes SVG).

    Parámetros
    ----------
    output_path : Path
        Ruta de escritura del archivo ``index.html``.
    document_title : str
        Título del documento HTML (aparece en la pestaña del navegador).
    page_heading : str
        Encabezado principal ``<h1>`` de la primera tarjeta.
    source_path : Path
        Ruta a la carpeta de resultados de la campaña (informativa).
    summary_text : str
        Párrafo de resumen de la campaña (número de corridas, parámetros, etc.).
    links : list of (label, href)
        Pares (etiqueta, ruta relativa) de los CSVs de salida.
    sections : list of (title, html_body)
        Cada sección produce una tarjeta ``<h2>`` + contenido HTML arbitrario.
    table_max_width : int
        Ancho máximo en píxeles de las tablas de métricas.
    """
    cards = [
        (
            page_heading,
            f"<p>Fuente: {html.escape(str(source_path))}</p>"
            f"<p>{html.escape(summary_text)}</p>"
            f"{build_links_html(links)}",
        )
    ]
    cards.extend(sections)

    html_parts = [
        "<!DOCTYPE html>",
        '<html lang="es">',
        "<head>",
        '  <meta charset="utf-8" />',
        f"  <title>{html.escape(document_title)}</title>",
        "  <style>",
        "    body {",
        "      font-family: Verdana, Geneva, sans-serif;",
        "      margin: 24px;",
        f"      color: {THEME_TEXT_COLOR};",
        f"      background: {THEME_BACKGROUND};",
        "    }",
        "    h1, h2 { margin: 0 0 12px 0; }",
        f"    p {{ margin: 0 0 12px 0; color: {THEME_MUTED_TEXT}; }}",
        "    .card {",
        f"      background: {THEME_CARD_BACKGROUND};",
        f"      border: 1px solid {THEME_BORDER};",
        "      border-radius: 14px;",
        "      padding: 18px 20px;",
        "      margin-bottom: 20px;",
        "      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.04);",
        "    }",
        "    .metric-table {",
        "      border-collapse: collapse;",
        "      width: 100%;",
        f"      max-width: {table_max_width}px;",
        "    }",
        "    .metric-table th, .metric-table td {",
        f"      border-bottom: 1px solid {THEME_ROW_BORDER};",
        "      padding: 8px 10px;",
        "      text-align: left;",
        "      font-size: 14px;",
        "    }",
        "    .metric-table th {",
        f"      color: {THEME_MUTED_TEXT};",
        f"      background: {THEME_BACKGROUND};",
        "    }",
        f"    ul {{ margin: 0; padding-left: 18px; color: {THEME_MUTED_TEXT}; }}",
        "    li { margin-bottom: 8px; }",
        "    img {",
        "      width: 100%;",
        "      max-width: 100%;",
        "      height: auto;",
        "      display: block;",
        f"      border: 1px solid {THEME_BORDER};",
        "      border-radius: 14px;",
        f"      background: {THEME_CARD_BACKGROUND};",
        "    }",
        "    .links a {",
        "      margin-right: 16px;",
        f"      color: {THEME_LINK};",
        "      text-decoration: none;",
        "    }",
        "  </style>",
        "</head>",
        "<body>",
    ]

    for index, (title, body_html) in enumerate(cards):
        heading_tag = "h1" if index == 0 else "h2"
        html_parts.extend(
            [
                '  <div class="card">',
                f"    <{heading_tag}>{html.escape(title)}</{heading_tag}>",
                f"    {body_html}",
                "  </div>",
            ]
        )

    html_parts.extend(["</body>", "</html>"])
    output_path.write_text("\n".join(html_parts), encoding="utf-8")


def aggregate_summary(
    summary_rows: list[dict[str, str]],
    summary_fields: list[str],
    group_field: str,
    group_values: list[float],
) -> tuple[list[dict[str, str]], list[str]]:
    """
    Calcula estadísticas descriptivas de las métricas de resumen agrupando por ``group_field``.

    Para cada valor de grupo calcula media, desviación estándar, mínimo y máximo
    de cada campo numérico del summary (excluye ``run_label`` y ``rng_run``).

    Retorna una tupla (aggregated_rows, fieldnames) donde cada fila de
    ``aggregated_rows`` corresponde a un único valor de grupo y sus estadísticas.
    """
    numeric_fields = [
        field
        for field in summary_fields
        if field not in {"run_label", "rng_run"} and is_float(summary_rows[0][field])
    ]
    grouped: dict[float, list[dict[str, str]]] = defaultdict(list)
    for row in summary_rows:
        grouped[float(row[group_field])].append(row)

    aggregated_rows: list[dict[str, str]] = []
    fieldnames = [group_field, "runs"]
    for field in numeric_fields:
        fieldnames.extend([f"{field}_mean", f"{field}_std", f"{field}_min", f"{field}_max"])

    for group_value in group_values:
        rows = grouped[group_value]
        aggregated = {
            group_field: format_float(group_value, 1),
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


def aggregate_timeseries(
    results_root: Path,
    manifest_rows: list[dict[str, str]],
    group_field: str,
    group_values: list[float],
) -> tuple[list[dict[str, str]], list[str]]:
    """
    Lee los archivos ``timeseries_<label>.csv`` de cada corrida y los agrega por
    (grupo, instante de tiempo), calculando media, desv. estándar, mínimo y máximo.

    Resultado: una fila por combinación ``(group_value, time_s)`` con las
    estadísticas de todas las corridas del grupo en ese instante.
    Útil para trazar series temporales promediadas sobre múltiples semillas.
    """
    grouped: dict[tuple[float, float], dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    timeseries_fields: list[str] = []

    for manifest_row in manifest_rows:
        group_value = float(manifest_row[group_field])
        timeseries_path = resolve_artifact_path(results_root, manifest_row["timeseries_file"])
        with timeseries_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if not timeseries_fields:
                    timeseries_fields = list(row.keys())
                time_s = float(row["time_s"])
                bucket = grouped[(group_value, time_s)]
                for field in timeseries_fields:
                    if field in {"run_label", "time_s"}:
                        continue
                    if is_float(row[field]):
                        bucket[field].append(float(row[field]))

    numeric_fields = [field for field in timeseries_fields if field not in {"run_label", "time_s"}]
    aggregated_rows: list[dict[str, str]] = []
    fieldnames = [group_field, "time_s", "runs"]
    for field in numeric_fields:
        fieldnames.extend([f"{field}_mean", f"{field}_std", f"{field}_min", f"{field}_max"])

    for group_value in group_values:
        group_times = sorted(time for candidate_group, time in grouped if candidate_group == group_value)
        for time_s in group_times:
            stats_by_field = grouped[(group_value, time_s)]
            row = {
                group_field: format_float(group_value, 1),
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


def wrap_text_lines(text: str | None, max_chars: int) -> list[str]:
    """
    Divide un texto en líneas que no excedan ``max_chars`` caracteres.

    Respeta los saltos de línea originales y realiza word-wrap sin cortar palabras.
    Retorna lista vacía si ``text`` es ``None`` o vacío.
    """
    if not text:
        return []
    lines: list[str] = []
    for paragraph in text.splitlines():
        words = paragraph.split()
        if not words:
            lines.append("")
            continue
        current = words[0]
        for word in words[1:]:
            candidate = f"{current} {word}"
            if len(candidate) <= max_chars:
                current = candidate
            else:
                lines.append(current)
                current = word
        lines.append(current)
    return lines


def svg_text(
    x: float,
    y: float,
    text: str,
    size: int = 14,
    weight: str = "400",
    fill: str = "#0f172a",
    anchor: str = "start",
) -> str:
    """Genera un elemento SVG ``<text>`` con la fuente y estilo del tema del proyecto."""
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" '
        f'fill="{fill}" text-anchor="{anchor}" font-family="Verdana, Geneva, sans-serif">{html.escape(text)}</text>'
    )


def svg_rect(
    x: float,
    y: float,
    width: float,
    height: float,
    fill: str,
    stroke: str = "none",
    stroke_width: float = 1.0,
    rx: float = 0.0,
) -> str:
    """Genera un elemento SVG ``<rect>`` con esquinas redondeadas opcionales (``rx``)."""
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width:.1f}" rx="{rx:.1f}" />'
    )


def svg_line(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    stroke: str,
    stroke_width: float = 2.0,
    dash: str | None = None,
) -> str:
    """Genera un elemento SVG ``<line>``. Si ``dash`` no es ``None`` aplica ``stroke-dasharray``."""
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
        f'stroke="{stroke}" stroke-width="{stroke_width:.1f}"{dash_attr} />'
    )


def svg_circle(
    cx: float,
    cy: float,
    radius: float,
    fill: str,
    stroke: str = "none",
    stroke_width: float = 1.0,
    opacity: float = 1.0,
) -> str:
    """Genera un elemento SVG ``<circle>`` con opacidad configurable."""
    return (
        f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{radius:.1f}" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="{stroke_width:.1f}" opacity="{opacity:.3f}" />'
    )


def svg_polyline(points: list[tuple[float, float]], stroke: str, stroke_width: float = 2.5, fill: str = "none") -> str:
    """Genera un elemento SVG ``<polyline>`` con uniones y extremos redondeados."""
    serialized = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return (
        f'<polyline points="{serialized}" stroke="{stroke}" stroke-width="{stroke_width:.1f}" '
        f'fill="{fill}" stroke-linejoin="round" stroke-linecap="round" />'
    )


def linear_map(value: float, domain_min: float, domain_max: float, range_min: float, range_max: float) -> float:
    """
    Interpola linealmente ``value`` del dominio ``[domain_min, domain_max]``
    al rango ``[range_min, range_max]``.

    Si ``domain_min == domain_max`` devuelve el punto medio del rango para evitar
    división por cero.  Usado para convertir valores de datos a coordenadas SVG.
    """
    if math.isclose(domain_min, domain_max):
        return (range_min + range_max) * 0.5
    ratio = (value - domain_min) / (domain_max - domain_min)
    return range_min + ratio * (range_max - range_min)


def padded_domain(values: list[float], min_floor: float | None = None, max_ceiling: float | None = None) -> tuple[float, float]:
    """
    Calcula los límites del eje Y con un margen automático del 12 % del rango de datos.

    Parámetros opcionales ``min_floor`` y ``max_ceiling`` imponen límites duros
    (p. ej., ``min_floor=0.0`` para métricas no negativas). Si los datos son
    constantes, añade un margen fijo para que el eje no colapse.
    """
    data_min = min(values)
    data_max = max(values)
    if min_floor is not None:
        data_min = min(data_min, min_floor)
    if max_ceiling is not None:
        data_max = max(data_max, max_ceiling)
    if math.isclose(data_min, data_max):
        pad = max(1.0, abs(data_min) * 0.1)
    else:
        pad = (data_max - data_min) * 0.12
    low = data_min - pad
    high = data_max + pad
    if min_floor is not None:
        low = max(low, min_floor)
    if max_ceiling is not None:
        high = min(high, max_ceiling)
    return low, high


def color_for_group(color_map: dict[str, str], group_value: float) -> str:
    """Devuelve el color asignado a ``group_value`` en el mapa, o gris neutro si no existe."""
    return color_map.get(format_float(group_value, 1), "#475569")


def spread_offsets(count: int, max_spread: float) -> list[float]:
    """
    Genera ``count`` desplazamientos horizontales simétricos dentro de ``[-max_spread, +max_spread]``.

    Usado en ``draw_scatter_mean_panel`` para evitar que los puntos de corridas
    individuales se solapén exactamente (jitter manual sin aleatoriedad).
    """
    if count <= 1:
        return [0.0]
    if count == 2:
        return [-max_spread * 0.45, max_spread * 0.45]
    return [
        linear_map(index, 0.0, float(count - 1), -max_spread, max_spread)
        for index in range(count)
    ]


def draw_panel_frame(
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    subtitle: str | None = None,
    footer_height: float = 58.0,
) -> tuple[list[str], tuple[float, float, float, float, float]]:
    """
    Dibuja el marco de un panel SVG con título, subtítulo y área de trazado.

    Retorna una tupla ``(items, geometry)`` donde:
    - ``items`` es la lista de strings SVG del marco (fondo, título, subtítulo).
    - ``geometry`` es ``(plot_left, plot_top, plot_width, plot_height, footer_top)``
      que el llamador usa para posicionar el contenido del panel.
    """
    items = [
        svg_rect(x, y, width, height, fill="#ffffff", stroke="#cbd5e1", stroke_width=1.0, rx=14.0),
    ]
    title_lines = wrap_text_lines(title, max(18, int((width - 36.0) / 11.0)))
    subtitle_lines = wrap_text_lines(subtitle, max(22, int((width - 36.0) / 12.5)))

    cursor_y = y + 28.0
    for index, line in enumerate(title_lines):
        items.append(svg_text(x + 18.0, cursor_y + index * 18.0, line, size=17, weight="700"))
    cursor_y += max(0, len(title_lines) - 1) * 18.0
    if subtitle_lines:
        cursor_y += 18.0
        for index, line in enumerate(subtitle_lines):
            items.append(svg_text(x + 18.0, cursor_y + index * 14.0, line, size=11, fill="#475569"))
        cursor_y += max(0, len(subtitle_lines) - 1) * 14.0

    plot_top = cursor_y + 12.0
    plot_left = x + 56.0
    plot_width = width - 76.0
    plot_height = height - (plot_top - y) - footer_height
    footer_top = y + height - footer_height
    return items, (plot_left, plot_top, plot_width, plot_height, footer_top)


def draw_y_axis(
    plot_left: float,
    plot_top: float,
    plot_width: float,
    plot_height: float,
    domain: tuple[float, float],
    tick_count: int = 5,
    percent: bool = False,
) -> list[str]:
    """
    Genera las líneas de cuadrícula horizontal, el eje Y vertical y las etiquetas numéricas.

    Parámetros
    ----------
    domain : tuple (y_min, y_max)
        Rango visible del eje Y.
    tick_count : int
        Número de marcas/líneas de cuadrícula equidistantes.
    percent : bool
        Si ``True``, las etiquetas llevan el sufijo ``%``.
    """
    items = []
    y_min, y_max = domain
    for tick in range(tick_count):
        value = y_min + (y_max - y_min) * (tick / max(1, tick_count - 1))
        y = linear_map(value, y_min, y_max, plot_top + plot_height, plot_top)
        items.append(svg_line(plot_left, y, plot_left + plot_width, y, stroke="#e2e8f0", stroke_width=1.0))
        label = f"{value:.0f}%" if percent else f"{value:.1f}"
        items.append(svg_text(plot_left - 10.0, y + 4.0, label, size=10, fill="#64748b", anchor="end"))
    items.append(svg_line(plot_left, plot_top, plot_left, plot_top + plot_height, stroke="#94a3b8", stroke_width=1.2))
    items.append(svg_line(plot_left, plot_top + plot_height, plot_left + plot_width, plot_top + plot_height, stroke="#94a3b8", stroke_width=1.2))
    return items


def draw_scatter_mean_panel(
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    subtitle: str,
    group_values: list[float],
    run_values: dict[float, list[float]],
    means: dict[float, float],
    color_map: dict[str, str],
    x_label_formatter,
    percent: bool = False,
) -> str:
    """
    Genera un panel SVG de dispersión con línea de media superpuesta.

    Cada punto representa el valor final de una corrida individual; la línea
    negra conecta las medias de cada grupo.  Los puntos del mismo grupo se
    dispersan horizontalmente (jitter simétrico) para mejorar la legibilidad.

    Parámetros
    ----------
    group_values : list[float]
        Valores del parámetro que define el eje X (ej. rangos de contacto).
    run_values : dict[float → list[float]]
        Valores por corrida para cada grupo.
    means : dict[float → float]
        Media precalculada para cada grupo.
    color_map : dict[str → str]
        Mapa de ``format_float(group_value, 1)`` a color hexadecimal.
    x_label_formatter : callable(float) → str
        Función que convierte un valor de grupo a etiqueta del eje X.
    percent : bool
        Si ``True``, el eje Y va de 0 a 100 % fijo.
    """
    panel, (plot_left, plot_top, plot_width, plot_height, _) = draw_panel_frame(
        x, y, width, height, title, subtitle, footer_height=46.0
    )
    all_values = [value for values in run_values.values() for value in values]
    domain = (0.0, 100.0) if percent else padded_domain(all_values)
    panel.extend(draw_y_axis(plot_left, plot_top, plot_width, plot_height, domain, percent=percent))

    x_positions = {
        group_value: plot_left + plot_width * ((index + 0.5) / len(group_values))
        for index, group_value in enumerate(group_values)
    }
    mean_points: list[tuple[float, float]] = []
    plot_bottom = plot_top + plot_height
    max_points = max(len(values) for values in run_values.values())
    point_radius = 5.0 if max_points <= 12 else 4.2 if max_points <= 24 else 3.6
    group_band = plot_width / max(1, len(group_values))
    max_spread = min(18.0, group_band * 0.22)

    for group_value in group_values:
        xpos = x_positions[group_value]
        panel.append(svg_text(xpos, plot_bottom + 20.0, x_label_formatter(group_value), size=11, fill="#334155", anchor="middle"))
        values = run_values[group_value]
        offsets = spread_offsets(len(values), max_spread)
        for index, value in enumerate(values):
            y_pos = linear_map(value, domain[0], domain[1], plot_bottom, plot_top)
            panel.append(
                svg_circle(
                    xpos + offsets[index],
                    y_pos,
                    point_radius,
                    fill=color_for_group(color_map, group_value),
                    stroke="#ffffff",
                    stroke_width=1.2,
                    opacity=0.72,
                )
            )
        mean_y = linear_map(means[group_value], domain[0], domain[1], plot_bottom, plot_top)
        mean_points.append((xpos, mean_y))

    panel.append(svg_polyline(mean_points, stroke="#0f172a", stroke_width=2.0))
    for group_value, mean_y in zip(group_values, [point[1] for point in mean_points]):
        panel.append(svg_circle(x_positions[group_value], mean_y, 5.5, fill="#ffffff", stroke="#0f172a", stroke_width=2.0))

    return "".join(panel)


def draw_grouped_bar_panel(
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    subtitle: str,
    group_values: list[float],
    series: list[tuple[str, str, dict[float, float]]],
    x_label_formatter,
    percent: bool = False,
) -> str:
    """
    Genera un panel SVG de barras agrupadas por grupo y serie (ej. Stage 2 vs Stage 3).

    Parámetros
    ----------
    series : list of (label, color, values_dict)
        Cada elemento define una serie: su leyenda textual, color hexadecimal
        y un dict ``{group_value: bar_height}``.
    x_label_formatter : callable(float) → str
        Función que convierte un valor de grupo a etiqueta del eje X.
    percent : bool
        Si ``True``, el eje Y va de 0 a 100 % fijo.
    """
    panel, (plot_left, plot_top, plot_width, plot_height, footer_top) = draw_panel_frame(
        x, y, width, height, title, subtitle, footer_height=72.0
    )
    all_values = [values[group_value] for _, _, values in series for group_value in group_values]
    domain = (0.0, 100.0) if percent else padded_domain(all_values, min_floor=0.0)
    panel.extend(draw_y_axis(plot_left, plot_top, plot_width, plot_height, domain, percent=percent))

    group_width = plot_width / len(group_values)
    bar_area = group_width * 0.78
    bar_width = bar_area / len(series)
    plot_bottom = plot_top + plot_height
    for index, group_value in enumerate(group_values):
        x_start = plot_left + group_width * index + (group_width - bar_area) * 0.5
        panel.append(
            svg_text(
                plot_left + group_width * (index + 0.5),
                plot_bottom + 20.0,
                x_label_formatter(group_value),
                size=11,
                fill="#334155",
                anchor="middle",
            )
        )
        for series_index, (_, color, values) in enumerate(series):
            bar_x = x_start + bar_width * series_index
            value = values[group_value]
            bar_y = linear_map(value, domain[0], domain[1], plot_bottom, plot_top)
            bar_height = plot_bottom - bar_y
            panel.append(svg_rect(bar_x, bar_y, max(8.0, bar_width - 4.0), bar_height, fill=color, rx=4.0))

    usable_width = width - 36.0
    item_width = min(usable_width / max(1, len(series)), 170.0)
    legend_y = footer_top + 46.0
    for index, (label, color, _) in enumerate(series):
        lx = x + 18.0 + index * item_width
        panel.append(svg_rect(lx, legend_y - 10.0, 12.0, 12.0, fill=color, rx=2.0))
        panel.append(svg_text(lx + 18.0, legend_y, label, size=11, fill="#334155"))

    return "".join(panel)


def draw_line_panel(
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    subtitle: str,
    series: dict[float, list[tuple[float, float]]],
    color_map: dict[str, str],
    series_label_formatter,
    percent: bool = False,
) -> str:
    """
    Genera un panel SVG de series temporales (líneas múltiples).

    Cada clave de ``series`` identifica un grupo o caso; su valor es una lista de
    puntos ``(time_s, metric_value)`` ya ordenados por tiempo.  Cada serie se dibuja
    como una polilínea con un punto marcador al final.  La leyenda se adapta
    automáticamente en columnas según el número de series y la longitud de las
    etiquetas.

    Parámetros
    ----------
    series : dict[float → list[(time_s, value)]]
        Clave: identificador numérico del grupo.  Valor: lista de puntos temporales.
    color_map : dict[str → str]
        Mapa de ``format_float(group_value, 1)`` a color hexadecimal.
    series_label_formatter : callable(float) → str
        Función que convierte un identificador de grupo a etiqueta de leyenda.
    percent : bool
        Si ``True``, el eje Y va de 0 a 100 % fijo; de lo contrario se calcula
        automáticamente con ``padded_domain``.
    """
    group_values = sorted(series)
    legend_labels = [series_label_formatter(group_value) for group_value in group_values]
    longest_label = max((len(label) for label in legend_labels), default=0)
    legend_columns = 1 if len(group_values) <= 1 else 2 if len(group_values) >= 2 else len(group_values)
    if len(group_values) == 2 and longest_label <= 24:
        legend_columns = 2
    elif len(group_values) == 3 and longest_label <= 22:
        legend_columns = 3
    legend_item_width = (width - 36.0) / max(1, legend_columns)
    legend_wrap_chars = max(14, int((legend_item_width - 34.0) / 6.2))
    legend_line_sets = [wrap_text_lines(label, legend_wrap_chars) for label in legend_labels]
    legend_rows = max(1, math.ceil(len(group_values) / max(1, legend_columns)))
    max_legend_lines = max((len(lines) for lines in legend_line_sets), default=1)
    footer_height = 60.0 + legend_rows * (max_legend_lines * 14.0 + 10.0)

    panel, (plot_left, plot_top, plot_width, plot_height, footer_top) = draw_panel_frame(
        x, y, width, height, title, subtitle, footer_height=footer_height
    )
    all_x = [time for values in series.values() for time, _ in values]
    all_y = [value for values in series.values() for _, value in values]
    x_domain = (min(all_x), max(all_x))
    y_domain = (0.0, 100.0) if percent else padded_domain(all_y, min_floor=0.0)
    panel.extend(draw_y_axis(plot_left, plot_top, plot_width, plot_height, y_domain, percent=percent))

    plot_bottom = plot_top + plot_height
    for tick in range(6):
        time_value = x_domain[0] + (x_domain[1] - x_domain[0]) * (tick / 5)
        x_pos = linear_map(time_value, x_domain[0], x_domain[1], plot_left, plot_left + plot_width)
        panel.append(svg_line(x_pos, plot_top, x_pos, plot_bottom, stroke="#f1f5f9", stroke_width=1.0))
        panel.append(svg_text(x_pos, plot_bottom + 20.0, f"{time_value:.0f}", size=10, fill="#64748b", anchor="middle"))

    for group_value in sorted(series):
        color = color_for_group(color_map, group_value)
        values = series[group_value]
        points = [
            (
                linear_map(time, x_domain[0], x_domain[1], plot_left, plot_left + plot_width),
                linear_map(value, y_domain[0], y_domain[1], plot_bottom, plot_top),
            )
            for time, value in values
        ]
        panel.append(svg_polyline(points, stroke=color, stroke_width=2.6))
        panel.append(svg_circle(points[-1][0], points[-1][1], 4.0, fill=color))

    legend_y = footer_top + 28.0
    row_height = max_legend_lines * 14.0 + 10.0
    for index, group_value in enumerate(group_values):
        row = index // max(1, legend_columns)
        column = index % max(1, legend_columns)
        lx = x + 18.0 + column * legend_item_width
        ly = legend_y + row * row_height
        color = color_for_group(color_map, group_value)
        panel.append(svg_rect(lx, ly - 10.0, 12.0, 12.0, fill=color, rx=2.0))
        for line_index, line in enumerate(legend_line_sets[index]):
            panel.append(svg_text(lx + 18.0, ly + line_index * 14.0, line, size=11, fill="#334155"))

    return "".join(panel)


def hex_to_rgb(color: str) -> tuple[int, int, int]:
    """Convierte un color hexadecimal ``"#rrggbb"`` a tupla ``(r, g, b)`` de enteros."""
    color = color.lstrip("#")
    return tuple(int(color[index:index + 2], 16) for index in (0, 2, 4))


def rgb_to_hex(rgb: tuple[int, int, int]) -> str:
    """Convierte una tupla ``(r, g, b)`` de enteros a color hexadecimal ``"#rrggbb"``."""
    return "#" + "".join(f"{max(0, min(255, channel)):02x}" for channel in rgb)


def interpolate_color(color_start: str, color_end: str, ratio: float) -> str:
    """
    Interpola linealmente entre dos colores hexadecimales.

    ``ratio=0.0`` devuelve ``color_start``; ``ratio=1.0`` devuelve ``color_end``.
    El valor se fija al rango [0, 1] antes de interpolar.
    Usado para construir la escala de color continua del heatmap.
    """
    start = hex_to_rgb(color_start)
    end = hex_to_rgb(color_end)
    clamped = max(0.0, min(1.0, ratio))
    blended = tuple(
        int(round(start[channel] + (end[channel] - start[channel]) * clamped))
        for channel in range(3)
    )
    return rgb_to_hex(blended)


def heatmap_fill(value: float, domain_min: float, domain_max: float) -> str:
    """
    Devuelve el color de relleno para una celda del heatmap según la posición de
    ``value`` en ``[domain_min, domain_max]``.

    Usa la escala ``HEATMAP_LOW_COLOR`` → ``HEATMAP_HIGH_COLOR``.
    Si el dominio es constante asigna el color del punto medio.
    """
    if math.isclose(domain_min, domain_max):
        ratio = 0.5
    else:
        ratio = (value - domain_min) / (domain_max - domain_min)
    return interpolate_color(HEATMAP_LOW_COLOR, HEATMAP_HIGH_COLOR, ratio)


def ideal_text_color(fill: str) -> str:
    """
    Calcula el color de texto óptimo (oscuro o claro) para garantizar contraste
    legible sobre un fondo de color ``fill``.

    Usa la fórmula de luminancia relativa WCAG.  Devuelve ``"#0f172a"`` (oscuro)
    para fondos claros y ``"#f8fafc"`` (claro) para fondos oscuros.
    """
    red, green, blue = hex_to_rgb(fill)
    luminance = (0.2126 * red + 0.7152 * green + 0.0722 * blue) / 255.0
    return "#0f172a" if luminance > 0.62 else "#f8fafc"


def draw_heatmap_panel(
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    subtitle: str,
    row_values: list[float],
    column_values: list[int],
    values: dict[tuple[float, int], float],
    row_label_formatter,
    column_label_formatter,
    metric: str,
    percent: bool = False,
) -> str:
    """
    Genera un panel SVG de mapa de calor (heatmap) bidimensional.

    Diseñado para la campaña TTL × Buffer donde las filas representan valores
    de TTL y las columnas representan capacidades de buffer.  Cada celda se
    colorea con una escala continua interpolada y muestra el valor numérico
    centrado con contraste automático de texto.  Incluye una barra de escala
    de color en el footer del panel.

    Parámetros
    ----------
    row_values : list[float]
        Valores únicos del parámetro de filas (TTL en segundos), ordenados.
    column_values : list[int]
        Valores únicos del parámetro de columnas (buffer en paquetes), ordenados.
    values : dict[(ttl, buffer) → float]
        Valor de la métrica para cada combinación (TTL, buffer).
    row_label_formatter : callable(float) → str
        Convierte un valor de TTL a etiqueta de fila (ej. ``"120 s"``).
    column_label_formatter : callable(int) → str
        Convierte una capacidad de buffer a etiqueta de columna (ej. ``"128"``).
    metric : str
        Nombre de la métrica; determina el formato del valor en la celda
        (``_pct`` → porcentaje, ``_s`` → segundos, resto → número genérico).
    percent : bool
        Si ``True``, las etiquetas de la barra de escala llevan sufijo ``%``.
    """
    panel, (plot_left, plot_top, plot_width, plot_height, footer_top) = draw_panel_frame(
        x, y, width, height, title, subtitle, footer_height=74.0
    )
    ordered_rows = sorted(row_values)
    ordered_columns = sorted(column_values)
    all_values = [values[(row_value, column_value)] for row_value in ordered_rows for column_value in ordered_columns]
    domain_min = min(all_values)
    domain_max = max(all_values)
    plot_bottom = plot_top + plot_height
    row_label_width = 46.0
    grid_left = plot_left + row_label_width
    grid_top = plot_top + 6.0
    grid_width = plot_width - row_label_width
    grid_height = plot_height - 18.0
    cell_width = grid_width / max(1, len(ordered_columns))
    cell_height = grid_height / max(1, len(ordered_rows))

    for row_index, row_value in enumerate(ordered_rows):
        cell_y = grid_top + row_index * cell_height
        label_y = cell_y + cell_height * 0.58
        panel.append(
            svg_text(
                grid_left - 10.0,
                label_y,
                row_label_formatter(row_value),
                size=11,
                fill="#334155",
                anchor="end",
            )
        )
        for column_index, column_value in enumerate(ordered_columns):
            cell_x = grid_left + column_index * cell_width
            value = values[(row_value, column_value)]
            fill = heatmap_fill(value, domain_min, domain_max)
            panel.append(
                svg_rect(
                    cell_x + 1.0,
                    cell_y + 1.0,
                    max(10.0, cell_width - 2.0),
                    max(10.0, cell_height - 2.0),
                    fill=fill,
                    stroke="#ffffff",
                    stroke_width=1.0,
                    rx=6.0,
                )
            )
            if metric.endswith("_pct"):
                value_label = f"{value:.1f}%"
            elif metric.endswith("_s"):
                value_label = f"{value:.1f}"
            else:
                value_label = f"{value:.1f}" if not math.isclose(value, round(value)) else f"{value:.0f}"
            panel.append(
                svg_text(
                    cell_x + cell_width * 0.5,
                    cell_y + cell_height * 0.58,
                    value_label,
                    size=10,
                    weight="700",
                    fill=ideal_text_color(fill),
                    anchor="middle",
                )
            )

    for column_index, column_value in enumerate(ordered_columns):
        label_x = grid_left + cell_width * (column_index + 0.5)
        panel.append(
            svg_text(
                label_x,
                plot_bottom + 18.0,
                column_label_formatter(column_value),
                size=11,
                fill="#334155",
                anchor="middle",
            )
        )

    legend_left = x + 18.0
    legend_top = footer_top + 34.0
    legend_width = min(150.0, width - 180.0)
    steps = 36
    for step in range(steps):
        ratio_start = step / steps
        ratio_end = (step + 1) / steps
        step_x = legend_left + legend_width * ratio_start
        step_width = legend_width * (ratio_end - ratio_start) + 1.0
        panel.append(
            svg_rect(
                step_x,
                legend_top,
                step_width,
                12.0,
                fill=interpolate_color(HEATMAP_LOW_COLOR, HEATMAP_HIGH_COLOR, ratio_start),
                stroke="none",
                rx=0.0,
            )
        )
    panel.append(svg_rect(legend_left, legend_top, legend_width, 12.0, fill="none", stroke="#cbd5e1", stroke_width=0.8, rx=3.0))
    min_label = f"{domain_min:.1f}%" if percent else f"{domain_min:.1f}"
    max_label = f"{domain_max:.1f}%" if percent else f"{domain_max:.1f}"
    panel.append(svg_text(legend_left, legend_top + 26.0, min_label, size=10, fill="#64748b"))
    panel.append(svg_text(legend_left + legend_width, legend_top + 26.0, max_label, size=10, fill="#64748b", anchor="end"))
    panel.append(svg_text(legend_left + legend_width + 18.0, legend_top + 11.0, "bajo", size=10, fill="#64748b"))
    panel.append(svg_text(legend_left + legend_width + 18.0, legend_top + 24.0, "alto", size=10, fill="#64748b"))
    panel.append(svg_text(grid_left - row_label_width + 2.0, plot_top - 4.0, "TTL", size=10, fill="#64748b"))
    panel.append(svg_text(grid_left + grid_width * 0.5, plot_bottom + 34.0, "Buffer (paquetes)", size=10, fill="#64748b", anchor="middle"))

    return "".join(panel)
