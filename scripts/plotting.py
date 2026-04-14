from dash import Dash, dcc, html, Input, Output, State, ctx, ALL
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import socket
import random
import sys
import json

VAR_NAMES = [
    "time_ms",
    "engine_count",
    "gear_count",
    "engine_rpm",
    "secondary_rpm",
    "filtered_engine_rpm",
    "filtered_secondary_rpm",
    "target_rpm",
    "engine_rpm_error",
    "velocity_command",
    "inbound_limit_switch",
    "outbound_limit_switch",
    "engage_limit_switch",
    "target_engine_rpm_diff",
    "target_engine_rpm_diff_sum",
]

X_COL = "time_ms"
PLOTTABLE = [v for v in VAR_NAMES if v != X_COL]

PRESET_GROUPS = {
    "RPM Overview":    ["engine_rpm", "secondary_rpm", "filtered_engine_rpm", "filtered_secondary_rpm", "target_rpm", "target_engine_rpm_diff", "target_engine_rpm_diff_sum"],
    "Filtered RPM":    ["filtered_engine_rpm", "filtered_secondary_rpm", "target_rpm"],
    "RPM Error":       ["engine_rpm", "target_rpm", "engine_rpm_error"],
    "ECVT Control":    ["velocity_command", "engine_rpm_error"],
    "Limit Switches":  ["inbound_limit_switch", "outbound_limit_switch", "engage_limit_switch"],
    "Gear & Engine":   ["engine_count", "gear_count"],
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


def make_graph_panel(idx):
    panel_id = f"panel-{idx}"
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
                    html.Div([
                        dcc.Dropdown(
                            id={"type": "preset-dropdown", "index": idx},
                            options=[{"label": k, "value": k} for k in PRESET_GROUPS],
                            placeholder="Load preset…",
                            clearable=True,
                            style={
                                "width": "160px",
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
                            value=list(PRESET_GROUPS.values())[idx % len(PRESET_GROUPS)],
                            placeholder="Select variables…",
                            style={
                                "flex": "1",
                                "fontSize": "11px",
                                "backgroundColor": DARK_THEME["bg"],
                                "border": f"1px solid {DARK_THEME['border']}",
                                "color": DARK_THEME["text"],
                                "minWidth": "200px",
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
                            labelStyle={"marginRight": "12px"},
                        ),
                    ], style={"display": "flex", "gap": "12px", "alignItems": "center", "flex": "1", "flexWrap": "wrap"}),
                ],
                style={
                    "display": "flex",
                    "alignItems": "center",
                    "gap": "16px",
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


def run_app(filepath):
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
        dcc.Store(id="graph-count-store", data=3),

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
        triggered = ctx.triggered_id
        if triggered == "add-graph-btn":
            return min(current + 1, 8)
        elif triggered == "remove-graph-btn":
            return max(current - 1, 1)
        return current

    @app.callback(
        Output("graphs-container", "children"),
        Input("graph-count-store", "data"),
    )
    def render_panels(count):
        return [make_graph_panel(i) for i in range(count)]

    # Sync preset → variable selection
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

    # Render each graph
    @app.callback(
        Output({"type": "graph", "index": ALL}, "figure"),
        Input({"type": "var-dropdown", "index": ALL}, "value"),
        Input({"type": "chart-type", "index": ALL}, "value"),
        Input("x-range-slider", "value"),
    )
    def update_graphs(var_lists, chart_types, x_range):
        figs = []
        for vars_selected, chart_type in zip(var_lists, chart_types):
            if not vars_selected:
                fig = go.Figure()
                fig.update_layout(**PLOTLY_TEMPLATE["layout"], title="— No variables selected —")
                figs.append(fig)
                continue

            # Filter by x range
            mask = (df[x_col] >= x_range[0]) & (df[x_col] <= x_range[1])
            dff = df[mask].copy()

            valid_vars = [v for v in vars_selected if v in dff.columns]

            if chart_type == "scatter":
                fig = go.Figure()
                for i, v in enumerate(valid_vars):
                    fig.add_trace(go.Scatter(
                        x=dff[x_col], y=dff[v], mode="markers",
                        name=v, marker={"color": COLORS[i % len(COLORS)], "size": 3},
                    ))
            elif chart_type == "area":
                fig = go.Figure()
                for i, v in enumerate(valid_vars):
                    color = COLORS[i % len(COLORS)]
                    r, g, b = int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)
                    fill_color = f"rgba({r},{g},{b},0.15)"
                    fig.add_trace(go.Scatter(
                        x=dff[x_col], y=dff[v], mode="lines", name=v,
                        fill="tozeroy",
                        line={"color": color, "width": 1.5},
                        fillcolor=fill_color,
                    ))
            else:  # line
                fig = go.Figure()
                for i, v in enumerate(valid_vars):
                    fig.add_trace(go.Scatter(
                        x=dff[x_col], y=dff[v], mode="lines", name=v,
                        line={"color": COLORS[i % len(COLORS)], "width": 1.5},
                    ))

            layout_kwargs = dict(PLOTLY_TEMPLATE["layout"])
            layout_kwargs["title"] = {"text": " · ".join(valid_vars[:4]) + ("…" if len(valid_vars) > 4 else ""), "font": {"color": DARK_THEME["accent"], "size": 12}}
            layout_kwargs["xaxis"] = {**PLOTLY_THEME_AXIS(), "title": f"{x_col} (ms)"}
            layout_kwargs["yaxis"] = {**PLOTLY_THEME_AXIS(), "title": "Value"}
            fig.update_layout(**layout_kwargs)
            figs.append(fig)

        return figs

    port = 8050
    print(f"\n🚀  PLUNETTE running → http://127.0.0.1:{port}\n")
    app.run(host="127.0.0.1", port=port, debug=True, use_reloader=False)
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
    if len(params) <= 1:
        print("Usage: python plunette_dashboard.py <logfile.csv>")
        sys.exit(1)

    filename = params[1]
    filepath = filename  # accept full or relative path directly
    print(f"Loading: {filepath}")

    port = run_app(filepath)

    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    print(f"Server: http://{ip}:{port}")