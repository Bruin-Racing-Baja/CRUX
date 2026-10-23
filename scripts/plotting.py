import os

from dash import Dash, dcc, html, Input, Output, State, ctx, ALL
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import socket
import random
import sys
import json

VAR_NAMES = [
    "time_ms", "engine_count", "gear_count", "engine_rpm",
    "secondary_rpm", "filtered_engine_rpm", "filtered_secondary_rpm", "target_rpm",
    "engine_rpm_error", "ecvt_velocity_command", "ecvt_velocity", "ecvt_pos", "ecvt_iq", "ecvt_bus_voltage", "ecvt_bus_current",
    "ecvt_total_charge_used", "ecvt_total_power_used",
    "ecvt_inbound_limit_switch", "ecvt_outbound_limit_switch", "ecvt_engage_limit_switch",
    "centerlock_velocity_command", "centerlock_velocity", "centerlock_pos", "centerlock_iq", 
    "centerlock_bus_voltage", "centerlock_bus_current", 
    "centerlock_total_charge_used", "centerlock_total_power_used","centerlock_inbound_limit_switch", "centerlock_outbound_limit_switch"
]

X_COL = "time_ms"
PLOTTABLE = [v for v in VAR_NAMES if v != X_COL]

PRESET_GROUPS = {
    "RPM Overview": [
        "engine_rpm",
        "secondary_rpm",
        "filtered_engine_rpm",
        "filtered_secondary_rpm",
        "target_rpm",
        "target_engine_rpm_diff",
    ],
    "ECVT Velocity / Current": [
        "ecvt_velocity_command",
        "ecvt_velocity",
        "ecvt_iq",
        "ecvt_bus_current",
    ],
    "RPM Error": [
        "engine_rpm",
        "target_rpm",
        "engine_rpm_error",
    ],
    "Limit Switches": [
        "ecvt_inbound_limit_switch",
        "ecvt_outbound_limit_switch",
        "ecvt_engage_limit_switch",
    ],
    "Gear & Engine": [
        "engine_count",
        "gear_count",
    ],
}

DARK_THEME = {
    "bg": "#0d0f14",
    "panel": "#13161f",
    "border": "#1e2433",
    "accent": "#00e5ff",
    "accent2": "#ff4081",
    "text": "#e0e6f0",
    "subtext": "#6b7a99",
}

PLOTLY_TEMPLATE = {
    "layout": {
        "paper_bgcolor": "#13161f",
        "plot_bgcolor": "#0d0f14",
        "font": {"color": "#e0e6f0", "family": "JetBrains Mono, monospace"},
        "xaxis": {
            "gridcolor": "#1e2433",
            "linecolor": "#1e2433",
            "tickcolor": "#6b7a99",
            "title_font": {"color": "#6b7a99"},
        },
        "yaxis": {
            "gridcolor": "#1e2433",
            "linecolor": "#1e2433",
            "tickcolor": "#6b7a99",
            "title_font": {"color": "#6b7a99"},
        },
        "legend": {
            "bgcolor": "#13161f",
            "bordercolor": "#1e2433",
            "borderwidth": 1,
        },
        "title": {"font": {"color": "#00e5ff", "size": 14}},
        "margin": {"l": 60, "r": 20, "t": 50, "b": 50},
    }
}

COLORS = [
    "#00e5ff", "#ff4081", "#69ff47", "#ffd600", "#e040fb",
    "#ff6d00", "#00e676", "#2979ff", "#ff1744", "#ffea00",
]


def make_graph_panel(idx, files):
    panel_id = f"panel-{idx}"

def make_graph_panel(idx, files):
    panel_id = f"panel-{idx}"

    if idx == 0:
        default_preset = "RPM Overview"
    elif idx == 1:
        default_preset = "ECVT Velocity / Current"
    else:
        default_preset = list(PRESET_GROUPS.keys())[idx % len(PRESET_GROUPS)]
    
    default_vars = PRESET_GROUPS[default_preset]
    default_file = files[0] if files else None

    return html.Div(
        id=panel_id,
        children=[
            html.Div(
                [
                    html.Span(f"GRAPH {idx + 1}", style={
                        "fontFamily": "'JetBrains Mono', monospace",
                        "fontSize": "11px",
                        "letterSpacing": "3px",
                        "color": DARK_THEME["accent"],
                        "fontWeight": "700",
                    }),

                    dcc.Dropdown(
                        id={"type": "csv-file-dropdown", "index": idx},
                        options=[{"label": f, "value": f} for f in files],
                        value=[default_file] if default_file else [],
                        multi=True,
                        clearable=False,
                        placeholder="Select CSV files...",
                        style={
                            "width": "260px",
                            "fontSize": "11px",
                            "backgroundColor": DARK_THEME["bg"],
                            "border": f"1px solid {DARK_THEME['border']}",
                            "color": DARK_THEME["text"],
                        },
                    ),

                    dcc.Dropdown(
                        id={"type": "preset-dropdown", "index": idx},
                        options=[{"label": k, "value": k} for k in PRESET_GROUPS],
                        value=default_preset,
                        placeholder="Load preset...",
                        clearable=True,
                        style={
                            "width": "180px",
                            "fontSize": "11px",
                            "backgroundColor": DARK_THEME["bg"],
                            "border": f"1px solid {DARK_THEME['border']}",
                            "color": DARK_THEME["text"],
                        },
                    ),

                    dcc.Dropdown(
                        id={"type": "var-dropdown", "index": idx},
                        options=[{"label": v, "value": v} for v in PLOTTABLE],
                        multi=True,
                        value=default_vars,
                        placeholder="Select variables...",
                        style={
                            "flex": "1",
                            "fontSize": "11px",
                            "backgroundColor": DARK_THEME["bg"],
                            "border": f"1px solid {DARK_THEME['border']}",
                            "color": DARK_THEME["text"],
                            "minWidth": "260px",
                        },
                    ),

                    dcc.RadioItems(
                        id={"type": "chart-type", "index": idx},
                        options=[
                            {"label": "Line", "value": "line"},
                            {"label": "Scatter", "value": "scatter"},
                            {"label": "Area", "value": "area"},
                        ],
                        value="line",
                        inline=True,
                        style={"fontSize": "11px", "color": DARK_THEME["subtext"]},
                        inputStyle={"marginRight": "4px", "accentColor": DARK_THEME["accent"]},
                        labelStyle={"marginRight": "12px", "color": "white"},
                    ),
                ],
                style={
                    "display": "flex",
                    "alignItems": "center",
                    "gap": "12px",
                    "padding": "12px 16px",
                    "borderBottom": f"1px solid {DARK_THEME['border']}",
                    "flexWrap": "wrap",
                }
            ),

            dcc.Graph(
                id={"type": "graph", "index": idx},
                style={"height": "340px"},
                config={"displayModeBar": True, "modeBarButtonsToRemove": ["select2d", "lasso2d"]},
            ),
        ],
        style={
            "backgroundColor": DARK_THEME["panel"],
            "border": f"1px solid {DARK_THEME['border']}",
            "borderRadius": "8px",
            "overflow": "hidden",
            "marginBottom": "16px",
        }
    )


def run_app(csv_dir):
    files = sorted([f for f in os.listdir(csv_dir) if f.endswith(".csv")])

    if not files:
        raise FileNotFoundError("No CSV files found")

    selected_file = files[0]
    filepath = os.path.join(csv_dir, selected_file)

    df = pd.read_csv(filepath)

    # Compute derived channels
    if "target_rpm" in df.columns and "engine_rpm" in df.columns:
        df["target_engine_rpm_diff"] = df["target_rpm"] - df["engine_rpm"]
        df["target_engine_rpm_diff_sum"] = df["target_engine_rpm_diff"].cumsum()


    # Determine x-axis column
    x_col = X_COL if X_COL in df.columns else df.columns[0]

    app = Dash(__name__, suppress_callback_exceptions=True)

    app.index_string = '''
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>PLUNETTE</title>
        {%favicon%}
        {%css%}
        <link rel="preconnect" href="https://fonts.googleapis.com">
        <link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&family=Barlow+Condensed:wght@300;600;800&display=swap" rel="stylesheet">
        <style>
            * { box-sizing: border-box; margin: 0; padding: 0; }
            body { background: #0d0f14; color: #e0e6f0; font-family: 'JetBrains Mono', monospace; }
            ::-webkit-scrollbar { width: 6px; height: 6px; }
            ::-webkit-scrollbar-track { background: #0d0f14; }
            ::-webkit-scrollbar-thumb { background: #1e2433; border-radius: 3px; }
            .Select-control, .Select-menu-outer { background-color: #0d0f14 !important; border-color: #1e2433 !important; }
            .Select-value-label { color: #e0e6f0 !important; }
            .Select-option { background-color: #13161f !important; color: #e0e6f0 !important; }
            .Select-option:hover { background-color: #1e2433 !important; }
            .dash-dropdown .Select-multi-value-wrapper .Select-value { background: #1e2433 !important; border-color: #00e5ff33 !important; }
            .controls-bar input[type="number"],
            .controls-bar input[type="text"],
            .controls-bar [role="spinbutton"],
            .controls-bar .rc-input-number,
            .controls-bar .rc-input-number-input { color: #6b7a99; background: #13161f; border: 1px solid #1e2433; }
            .rc-slider-tooltip .rc-slider-tooltip-inner,
            .rc-tooltip .rc-tooltip-inner { color: #6b7a99; background: #13161f; border: 1px solid #1e2433; }
            .dash-dropdown-value-item { color: #13161f }
            .header-bar { 
                background: linear-gradient(90deg, #0d0f14 0%, #13161f 50%, #0d0f14 100%);
                border-bottom: 1px solid #1e2433;
                padding: 20px 32px;
                display: flex;
                align-items: center;
                justify-content: space-between;
            }
            .title-text {
                font-family: 'Barlow Condensed', sans-serif;
                font-weight: 800;
                font-size: 42px;
                letter-spacing: 8px;
                background: linear-gradient(90deg, #00e5ff, #2979ff);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
                background-clip: text;
            }
            .subtitle { font-size: 11px; color: #6b7a99; letter-spacing: 2px; margin-top: 4px; }
            .stat-pill {
                background: #13161f;
                border: 1px solid #1e2433;
                border-radius: 6px;
                padding: 8px 16px;
                font-size: 11px;
                color: #6b7a99;
                letter-spacing: 1px;
            }
            .stat-val { color: #00e5ff; font-size: 18px; font-weight: 700; display: block; }
            .controls-bar {
                background: #13161f;
                border-bottom: 1px solid #1e2433;
                padding: 12px 32px;
                display: flex;
                align-items: center;
                gap: 16px;
                flex-wrap: wrap;
            }
            .files-selection {
                background: #13161f;
                border-bottom: 1px solid #1e2433;
                padding: 12px 32px;
                display: flex;
                align-items: center;
             }
            .btn {
                background: transparent;
                border: 1px solid #1e2433;
                color: #6b7a99;
                padding: 7px 16px;
                border-radius: 4px;
                font-family: 'JetBrains Mono', monospace;
                font-size: 11px;
                letter-spacing: 1px;
                cursor: pointer;
                transition: all 0.15s;
            }
            .btn:hover { border-color: #00e5ff; color: #00e5ff; }
            .btn-accent {
                background: #00e5ff22;
                border-color: #00e5ff;
                color: #00e5ff;
            }
            .range-label { font-size: 11px; color: #6b7a99; letter-spacing: 1px; margin-right: 4px; }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
'''

    num_rows = len(df)
    duration_s = round((df[x_col].max() - df[x_col].min()) / 1e3, 2) if x_col in df.columns else "—"
    num_cols = len(df.columns)

    app.layout = html.Div([
        # Header
        html.Div([
            html.Div([
                html.Div("PLUNETTE", className="title-text"),
                html.Div("ECVT DATA VISUALIZER", className="subtitle"),
            ]),
            html.Div([
                html.Div([html.Span(f"{num_rows:,}", className="stat-val"), "SAMPLES"], className="stat-pill"),
                html.Div([html.Span(f"{duration_s}s", className="stat-val"), "DURATION"], className="stat-pill"),
                html.Div([html.Span(f"{num_cols}", className="stat-val"), "CHANNELS"], className="stat-pill"),
            ], style={"display": "flex", "gap": "12px"}),
        ], className="header-bar"),
        
        # Controls bar
        html.Div([
            html.Span("GRAPHS:", style={"fontSize": "11px", "color": DARK_THEME["subtext"], "letterSpacing": "2px"}),
            html.Button("＋ Add Graph", id="add-graph-btn", n_clicks=0, className="btn btn-accent"),
            html.Button("－ Remove Graph", id="remove-graph-btn", n_clicks=0, className="btn"),
            html.Div(style={"flex": "1"}),
            html.Span("X-AXIS RANGE:", className="range-label"),
            dcc.RangeSlider(
                id="x-range-slider",
                min=float(df[x_col].min()) if x_col in df.columns else 0,
                max=float(df[x_col].max()) if x_col in df.columns else 1,
                value=[float(df[x_col].min()), float(df[x_col].max())] if x_col in df.columns else [0, 1],
                tooltip={"placement": "bottom", "always_visible": False},
                marks=None,
                step=(df[x_col].max() - df[x_col].min()) / 100 if x_col in df.columns else 1,
                className="",
                updatemode="mouseup",
            ),
        ], className="controls-bar", style={"gap": "12px"}),

        # Graph count store
        dcc.Store(id="graph-count-store", data=2),

        # Main content
        html.Div(
            id="graphs-container",
            style={"padding": "20px 32px"},
        ),
    ], style={"backgroundColor": DARK_THEME["bg"], "minHeight": "100vh"})

    # --- Callbacks ---

    @app.callback(
        Output("graph-count-store", "data"),
        Input("add-graph-btn", "n_clicks"),
        Input("remove-graph-btn", "n_clicks"),
        State("graph-count-store", "data"),
        prevent_initial_call=True,
    )
    def update_graph_count(add, remove, current):
        try:
            current = int(current)
        except (TypeError, ValueError):
            current = 3

        triggered = ctx.triggered_id

        if triggered == "add-graph-btn":
            return min(current + 1, 8)

        if triggered == "remove-graph-btn":
            return max(current - 1, 1)

        return current


    @app.callback(
        Output("graphs-container", "children"),
        Input("graph-count-store", "data"),
    )
    def render_panels(count):
        return [make_graph_panel(i, files) for i in range(count)]

    @app.callback(
        Output({"type": "var-dropdown", "index": ALL}, "value"),
        Input({"type": "preset-dropdown", "index": ALL}, "value"),
        State({"type": "var-dropdown", "index": ALL}, "value"),
        prevent_initial_call=True,
    )
    def sync_presets(preset_vals, current_vals):
        out = list(current_vals)
        for i, pv in enumerate(preset_vals):
            if pv and pv in PRESET_GROUPS:
                out[i] = PRESET_GROUPS[pv]
        return out


    @app.callback(
        Output({"type": "graph", "index": ALL}, "figure"),
        Input({"type": "var-dropdown", "index": ALL}, "value"),
        Input({"type": "chart-type", "index": ALL}, "value"),
        Input("x-range-slider", "value"),
        Input({"type": "csv-file-dropdown", "index": ALL}, "value"),
    )
    def update_graphs(var_lists, chart_types, x_range, selected_files_by_graph):
        figs = []

        for vars_selected, chart_type, selected_files in zip(var_lists, chart_types, selected_files_by_graph):
            if not selected_files:
                selected_files = [files[0]]

            if isinstance(selected_files, str):
                selected_files = [selected_files]

            fig = go.Figure()

            if not vars_selected:
                fig.update_layout(**PLOTLY_TEMPLATE["layout"], title="— No variables selected —")
                figs.append(fig)
                continue

            for selected_file in selected_files:
                filepath = os.path.join(csv_dir, selected_file)
                df = pd.read_csv(filepath)

                if "target_rpm" in df.columns and "engine_rpm" in df.columns:
                    df["target_engine_rpm_diff"] = df["target_rpm"] - df["engine_rpm"]
                    df["target_engine_rpm_diff_sum"] = df["target_engine_rpm_diff"].cumsum()

                if "time_ms" in df.columns:
                    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0
                    x_col = "time_s"
                    min_x = x_range[0] / 1000.0
                    max_x = x_range[1] / 1000.0
                else:
                    x_col = df.columns[0]
                    min_x = x_range[0]
                    max_x = x_range[1]

                mask = (df[x_col] >= min_x) & (df[x_col] <= max_x)
                dff = df[mask].copy()

                valid_vars = [v for v in vars_selected if v in dff.columns]

                for i, v in enumerate(valid_vars):
                    if len(selected_files) == 1:
                        trace_name = v
                    else:
                        trace_name = f"{v} ({selected_file})"

                    if chart_type == "scatter":
                        fig.add_trace(go.Scatter(
                            x=dff[x_col],
                            y=dff[v],
                            mode="markers",
                            name=trace_name,
                            marker={"color": COLORS[i % len(COLORS)], "size": 3},
                        ))

                    elif chart_type == "area":
                        color = COLORS[i % len(COLORS)]
                        r, g, b = int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)
                        fill_color = f"rgba({r},{g},{b},0.15)"

                        fig.add_trace(go.Scatter(
                            x=dff[x_col],
                            y=dff[v],
                            mode="lines",
                            name=trace_name,
                            fill="tozeroy",
                            line={"color": color, "width": 1.5},
                            fillcolor=fill_color,
                        ))

                    else:
                        fig.add_trace(go.Scatter(
                            x=dff[x_col],
                            y=dff[v],
                            mode="lines",
                            name=trace_name,
                            line={"color": COLORS[i % len(COLORS)], "width": 1.5},
                        ))

            layout_kwargs = dict(PLOTLY_TEMPLATE["layout"])
            layout_kwargs["title"] = {
                "text": " · ".join(vars_selected[:4]) + ("..." if len(vars_selected) > 4 else ""),
                "font": {"color": DARK_THEME["accent"], "size": 12}
            }
            layout_kwargs["xaxis"] = {**PLOTLY_THEME_AXIS(), "title": "Time (s)"}
            layout_kwargs["yaxis"] = {**PLOTLY_THEME_AXIS(), "title": "Value"}

            fig.update_layout(**layout_kwargs)
            figs.append(fig)

        return figs

    port = 8051
    print(f"\n🚀  PLUNETTE running → http://0.0.0.0:{port}\n")
    app.run(
        host="0.0.0.0",
        port=port,
        debug=True,
        use_reloader=True,
        dev_tools_hot_reload=True,
        dev_tools_hot_reload_interval=1000,
    )
    return port


def PLOTLY_THEME_AXIS():
    return {
        "gridcolor": "#1e2433",
        "linecolor": "#1e2433",
        "tickcolor": "#6b7a99",
        "tickfont": {"color": "#6b7a99", "size": 10},
        "title_font": {"color": "#6b7a99"},
        "zerolinecolor": "#1e2433",
    }

if __name__ == "__main__":
    params = sys.argv

    base_dir = os.path.dirname(__file__)
    csv_dir = os.path.join(base_dir, "csv_files")

    port = run_app(csv_dir)

    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    print(f"Server: http://{ip}:{port}")