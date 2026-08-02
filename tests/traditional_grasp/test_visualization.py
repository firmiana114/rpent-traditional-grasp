from __future__ import annotations

import numpy as np

from rpent_traditional_grasp.visualization import (
    body_point_to_left_camera,
    project_rectified_point,
)


def test_body_point_projects_to_left_and_right_rectified_pixels() -> None:
    camera_to_body = np.eye(4)
    point = body_point_to_left_camera(
        np.array([0.1, 0.0, 1.0]),
        camera_to_body,
    )
    left_projection = np.array(
        [[100.0, 0.0, 50.0, 0.0], [0.0, 100.0, 40.0, 0.0], [0, 0, 1, 0]]
    )
    right_projection = left_projection.copy()
    right_projection[0, 3] = -10.0

    left_uv = project_rectified_point(point, left_projection)
    right_uv = project_rectified_point(point, right_projection)

    np.testing.assert_allclose(left_uv, [60.0, 40.0])
    np.testing.assert_allclose(right_uv, [50.0, 40.0])
