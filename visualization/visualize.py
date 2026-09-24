import json
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go


DATA_JSON = Path("data/short.json")
TRAJECTORY_CSV = Path("results/trajectory.csv")
CLEANED_AREA_CSV = Path("results/cleaned_area.csv")
RESULTS_JSON = Path("results/results.json")

ROBOT_STRIDE = 10
HEADING_STRIDE = 5
HEADING_LENGTH = 0.25


def transform_points(points, x, y, heading):
    """Transform points from robot frame into world frame."""
    points = np.asarray(points)

    c = np.cos(heading)
    s = np.sin(heading)

    rotation = np.array([
        [c, -s],
        [s,  c]
    ])

    return points @ rotation.T + np.array([x, y])


def add_polygon_trace(fig, points, name, color=None, showlegend=True):
    points = np.asarray(points)

    # close polygon
    points = np.vstack([points, points[0]])

    fig.add_trace(
        go.Scatter(
            x=points[:, 0],
            y=points[:, 1],
            mode="lines",
            fill="toself",
            name=name,
            showlegend=showlegend,
            opacity=0.3,
            line=dict(width=1),
        )
    )


def add_cleaned_area(fig, cleaned_area):
    first_polygon = True

    for polygon_id, polygon in cleaned_area.groupby(
        "polygon_id", sort=False
    ):
        path_parts = []

        for ring_id, ring in polygon.groupby(
            "ring_id", sort=False
        ):
            points = ring[["x_m", "y_m"]].to_numpy()

            if len(points) < 3:
                continue

            path = f"M {points[0, 0]} {points[0, 1]}"

            for x, y in points[1:]:
                path += f" L {x} {y}"

            path += " Z"
            path_parts.append(path)

        if not path_parts:
            continue

        fig.add_shape(
            type="path",
            path=" ".join(path_parts),
            fillcolor="rgba(34, 197, 94, 0.22)",
            line=dict(
                color="rgba(22, 163, 74, 0.85)",
                width=1.2,
            ),
            fillrule="evenodd",
            layer="below",
            name="Cleaned area",
            legendgroup="cleaned_area",
            showlegend=first_polygon,
        )

        first_polygon = False


def main():
    # ------------------------------------------------------------
    # Load input
    # ------------------------------------------------------------

    with open(DATA_JSON) as f:
        input_data = json.load(f)

    with open(RESULTS_JSON) as f:
        results = json.load(f)

    trajectory = pd.read_csv(TRAJECTORY_CSV)
    cleaned_area = pd.read_csv(CLEANED_AREA_CSV)

    raw_path = np.asarray(input_data["path"])
    robot = np.asarray(input_data["robot"])
    gadget = np.asarray(input_data["cleaning_gadget"])

    # ------------------------------------------------------------
    # Figure
    # ------------------------------------------------------------

    fig = go.Figure()

    # ------------------------------------------------------------
    # Raw trajectory
    # ------------------------------------------------------------

    fig.add_trace(
        go.Scatter(
            x=raw_path[:, 0],
            y=raw_path[:, 1],
            mode="lines+markers",
            name="Raw trajectory",
            line=dict(
                width=1,
                color="rgba(100, 116, 139, 0.65)",
            ),
            marker=dict(
                size=3,
                color="rgba(100, 116, 139, 0.65)",
            ),
        )
    )

    # ------------------------------------------------------------
    # Filtered / analyzed trajectory
    # ------------------------------------------------------------

    fig.add_trace(
        go.Scatter(
            x=trajectory["x_m"],
            y=trajectory["y_m"],
            mode="lines",
            name="Filtered trajectory",
            line=dict(
                width=3,
                color="#2563eb",
            ),
        )
    )

    # ------------------------------------------------------------
    # Speed visualization
    # ------------------------------------------------------------

    fig.add_trace(
        go.Scatter(
            x=trajectory["x_m"],
            y=trajectory["y_m"],
            mode="markers",
            name="Speed",
            visible="legendonly",
            marker=dict(
                size=7,
                color=trajectory["speed_m_per_s"],
                colorscale="Viridis",
                showscale=True,
                colorbar=dict(
                    title="Speed<br>[m/s]",
                    thickness=15,
                ),
            ),
            customdata=np.stack(
                [
                    trajectory["heading_rad"],
                    trajectory["curvature_per_m"],
                    trajectory["speed_m_per_s"],
                ],
                axis=1,
            ),
            hovertemplate=(
                "<b>Trajectory</b><br>"
                "x: %{x:.3f} m<br>"
                "y: %{y:.3f} m<br>"
                "heading: %{customdata[0]:.3f} rad<br>"
                "curvature: %{customdata[1]:.3f} 1/m<br>"
                "speed: %{customdata[2]:.3f} m/s"
                "<extra></extra>"
            ),
        )
    )

    # ------------------------------------------------------------
    # Headings
    # ------------------------------------------------------------

    heading_x = []
    heading_y = []

    for row in trajectory.iloc[::HEADING_STRIDE].itertuples():
        if not np.isfinite(row.heading_rad):
            continue

        dx = HEADING_LENGTH * np.cos(row.heading_rad)
        dy = HEADING_LENGTH * np.sin(row.heading_rad)

        heading_x += [
            row.x_m,
            row.x_m + dx,
            None,
        ]

        heading_y += [
            row.y_m,
            row.y_m + dy,
            None,
        ]

    fig.add_trace(
        go.Scatter(
            x=heading_x,
            y=heading_y,
            mode="lines",
            name="Heading",
            visible="legendonly",
            line=dict(
                width=1.5,
                color="#f59e0b",
            ),
        )
    )

    # ------------------------------------------------------------
    # Robot footprint
    # ------------------------------------------------------------

    first_robot = True

    for i in range(0, len(trajectory), ROBOT_STRIDE):
        row = trajectory.iloc[i]

        if not np.isfinite(row.heading_rad):
            continue

        footprint = transform_points(
            robot,
            row.x_m,
            row.y_m,
            row.heading_rad,
        )

        footprint = np.vstack(
            [footprint, footprint[0]]
        )

        fig.add_trace(
            go.Scatter(
                x=footprint[:, 0],
                y=footprint[:, 1],
                mode="lines",
                name="Robot footprint",
                legendgroup="robot",
                showlegend=first_robot,
                visible="legendonly",
                line=dict(
                    width=1.5,
                    color="rgba(30, 41, 59, 0.65)",
                ),
            )
        )

        first_robot = False

    # ------------------------------------------------------------
    # Cleaning gadget poses
    # ------------------------------------------------------------

    gadget_x = []
    gadget_y = []

    for i in range(0, len(trajectory), ROBOT_STRIDE):
        row = trajectory.iloc[i]

        if not np.isfinite(row.heading_rad):
            continue

        world_gadget = transform_points(
            gadget,
            row.x_m,
            row.y_m,
            row.heading_rad,
        )

        gadget_x += [
            world_gadget[0, 0],
            world_gadget[1, 0],
            None,
        ]

        gadget_y += [
            world_gadget[0, 1],
            world_gadget[1, 1],
            None,
        ]

    fig.add_trace(
        go.Scatter(
            x=gadget_x,
            y=gadget_y,
            mode="lines",
            name="Cleaning gadget",
            line=dict(
                width=3,
                color="#dc2626",
            ),
        )
    )

    # ------------------------------------------------------------
    # Cleaned-area polygon
    # ------------------------------------------------------------

    add_cleaned_area(
        fig,
        cleaned_area,
    )

    # ------------------------------------------------------------
    # Result cards
    # ------------------------------------------------------------

    cards = [
        (
            0.17,
            "PATH LENGTH",
            f"{results['path_length_m']:.3f} m",
        ),
        (
            0.50,
            "CLEANED AREA",
            f"{results['cleaned_area_m2']:.3f} m²",
        ),
        (
            0.83,
            "TRAVERSAL TIME",
            f"{results['traversal_time_s']:.3f} s",
        ),
    ]

    for x, title, value in cards:
        fig.add_annotation(
            x=x,
            y=1.08,
            xref="paper",
            yref="paper",
            text=(
                f"<span style='font-size:12px;color:#64748b'>"
                f"{title}</span>"
                f"<br>"
                f"<span style='font-size:22px'><b>{value}</b></span>"
            ),
            showarrow=False,
            align="center",
            bgcolor="white",
            bordercolor="rgba(148, 163, 184, 0.45)",
            borderwidth=1,
            borderpad=10,
        )

    # ------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------

    fig.update_layout(
        title=dict(
            text=(
                "<b>Cleaning Robot Path Analysis</b>"
                f"<br><span style='font-size:14px;color:#64748b'>"
                f"Matthias Jarsch</span>"
            ),
            x=0.5,
            xanchor="center",
            font=dict(size=24),
        ),
        template="plotly_white",
        height=850,
        margin=dict(
            l=70,
            r=170,
            t=180,
            b=70,
        ),
        hovermode="closest",
        legend=dict(
            title="Layers",
            x=1.01,
            y=1.0,
            xanchor="left",
            yanchor="top",
            groupclick="togglegroup",
            bgcolor="rgba(255,255,255,0.9)",
            bordercolor="rgba(148,163,184,0.4)",
            borderwidth=1,
        ),
    )

    fig.update_xaxes(
        title_text="x [m]",
        showgrid=True,
        gridcolor="rgba(148,163,184,0.20)",
        zeroline=False,
    )

    fig.update_yaxes(
        title_text="y [m]",
        showgrid=True,
        gridcolor="rgba(148,163,184,0.20)",
        zeroline=False,
        scaleanchor="x",
        scaleratio=1,
    )

    fig.write_html(
        "results/visualization.html"
    )

    fig.show()


if __name__ == "__main__":
    main()