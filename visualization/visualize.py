import json
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go


DATA_JSON = Path("data/short.json")
TRAJECTORY_CSV = Path("results/trajectory.csv")
CLEANED_AREA_CSV = Path("results/cleaned_area.csv")

ROBOT_STRIDE = 10
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
            fillcolor="rgba(0, 150, 100, 0.25)",
            line=dict(
                color="rgba(0, 120, 80, 0.8)",
                width=1,
            ),
            fillrule="evenodd",
            layer="below",
        )

def main():
    # ------------------------------------------------------------
    # Load input
    # ------------------------------------------------------------

    with open(DATA_JSON) as f:
        input_data = json.load(f)

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
            line=dict(width=1),
            marker=dict(size=3),
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
            line=dict(width=3),
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
            marker=dict(
                size=6,
                color=trajectory["speed_m_per_s"],
                colorscale="Viridis",
                showscale=True,
                colorbar=dict(
                    title="Speed [m/s]"
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
                "x=%{x:.3f} m<br>"
                "y=%{y:.3f} m<br>"
                "heading=%{customdata[0]:.3f} rad<br>"
                "curvature=%{customdata[1]:.3f} 1/m<br>"
                "speed=%{customdata[2]:.3f} m/s"
                "<extra></extra>"
            ),
        )
    )

    # ------------------------------------------------------------
    # Headings
    # ------------------------------------------------------------

    heading_x = []
    heading_y = []

    for row in trajectory.itertuples():
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
            line=dict(width=1),
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
                line=dict(width=1),
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
            line=dict(width=2),
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
    # Layout
    # ------------------------------------------------------------

    fig.update_layout(
        title="Cleaning Robot Path Analysis",
        xaxis_title="x [m]",
        yaxis_title="y [m]",
        hovermode="closest",
        legend=dict(
            title="Layers",
            groupclick="togglegroup",
        ),
    )

    # very important for geometric visualization
    fig.update_yaxes(
        scaleanchor="x",
        scaleratio=1,
    )

    fig.write_html(
        "results/visualization.html"
    )

    fig.show()


if __name__ == "__main__":
    main()