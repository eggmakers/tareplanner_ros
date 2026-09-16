#!/usr/bin/env python3
import argparse
import json
import os
import time

import airsim


PREFIX = "TARE_TUNNEL_"


def object_spec(name, center, size, role):
    return {
        "name": PREFIX + name,
        "center_ned": [float(value) for value in center],
        "size": [float(value) for value in size],
        "role": role,
    }


def build_specs(
    length,
    width,
    height,
    fixed_flight_height=1.5,
    world_to_map_z_offset=0.7,
):
    start_x = -4.0
    end_x = start_x + length
    center_x = (start_x + end_x) * 0.5
    wall_thickness = 0.5
    shell_width = width + wall_thickness * 2.0
    specs = [
        object_spec(
            "CEILING",
            (center_x, 0.0, -height - 0.2),
            (length, shell_width, 0.4),
            "ceiling",
        ),
        object_spec(
            "WALL_LEFT",
            (center_x, -(width + wall_thickness) * 0.5, -height * 0.5),
            (length, wall_thickness, height),
            "wall",
        ),
        object_spec(
            "WALL_RIGHT",
            (center_x, (width + wall_thickness) * 0.5, -height * 0.5),
            (length, wall_thickness, height),
            "wall",
        ),
        object_spec(
            "WALL_START",
            (start_x, 0.0, -height * 0.5),
            (wall_thickness, shell_width, height),
            "wall",
        ),
        object_spec(
            "WALL_END",
            (end_x, 0.0, -height * 0.5),
            (wall_thickness, shell_width, height),
            "wall",
        ),
    ]

    # Dense rows of sub-metre blocks exercise local replanning without turning
    # the course into a handful of oversized walls. Every row leaves at least
    # one flyable gap at the fixed 1.5 m test height.
    obstacle_rows = [
        (2.0, (-2.6, -1.8, -1.0)),
        (6.0, (1.0, 1.8, 2.6)),
        (10.0, (-2.6, -1.8, -1.0)),
        (14.0, (1.0, 1.8, 2.6)),
        (18.0, (-2.6, -1.8, -1.0)),
        (22.0, (1.0, 1.8, 2.6)),
    ]
    obstacles = []
    obstacle_index = 1
    world_flight_height = fixed_flight_height - world_to_map_z_offset
    if world_flight_height <= 0.0:
        raise ValueError("fixed flight height must exceed the AirSim-to-map z offset")
    for row_index, (forward, laterals) in enumerate(obstacle_rows):
        for column_index, lateral in enumerate(laterals):
            edge = 0.6 + 0.1 * ((row_index + column_index) % 3)
            # Put one block in every row exactly on the 1.5 m flight plane.
            # The second block alternates slightly above/below it while still
            # intersecting that plane, so the LiDAR cannot see over every row.
            if column_index == 0:
                height = world_flight_height
            else:
                height = (
                    world_flight_height - 0.35
                    if row_index % 2 == 0
                    else world_flight_height + 0.35
                )
            obstacles.append(
                (
                    "OBSTACLE_{:02d}".format(obstacle_index),
                    (forward, lateral, -height),
                    (edge, edge, 0.8),
                )
            )
            obstacle_index += 1

    overhead = [
        (5.0, -2.4, -2.8),
        (8.0, 0.8, -3.2),
        (11.0, 2.3, -2.7),
        (15.0, -2.4, -3.1),
        (18.0, 0.4, -2.8),
        (21.0, 2.3, -3.2),
    ]
    for overhead_index, center in enumerate(overhead):
        edge = 0.55 + 0.05 * (overhead_index % 3)
        obstacles.append(
            (
                "OBSTACLE_{:02d}".format(obstacle_index),
                center,
                (edge, edge, edge),
            )
        )
        obstacle_index += 1

    specs.extend(
        object_spec(name, center, size, "obstacle")
        for name, center, size in obstacles
    )
    return specs


def build_safety_model(
    specs,
    width,
    safety_margin,
    fixed_flight_height,
    world_to_map_z_offset,
    vehicle_radius=0.45,
    tracking_margin=0.20,
    pointcloud_margin=0.10,
    viewpoint_resolution=0.60,
):
    planner_clearance = vehicle_radius + tracking_margin + pointcloud_margin
    corridor_min_x = -width * 0.5 + safety_margin
    corridor_max_x = width * 0.5 - safety_margin
    rows = {}
    row_half_depths = {}
    obstacle_edges = []
    for item in specs:
        if item["role"] != "obstacle":
            continue
        obstacle_edges.extend(item["size"])
        center_height_map = -item["center_ned"][2] + world_to_map_z_offset
        if (
            abs(center_height_map - fixed_flight_height)
            > item["size"][2] * 0.5
        ):
            continue
        forward = round(item["center_ned"][0], 3)
        lateral = item["center_ned"][1]
        half_lateral = item["size"][1] * 0.5
        row_half_depths[forward] = max(
            row_half_depths.get(forward, 0.0), item["size"][0] * 0.5
        )
        rows.setdefault(forward, []).append(
            (
                max(corridor_min_x, lateral - half_lateral - planner_clearance),
                min(corridor_max_x, lateral + half_lateral + planner_clearance),
            )
        )

    row_clearances = []
    for forward, intervals in sorted(rows.items()):
        cursor = corridor_min_x
        free_widths = []
        for start, end in sorted(intervals):
            if start > cursor:
                free_widths.append(start - cursor)
            cursor = max(cursor, end)
        if cursor < corridor_max_x:
            free_widths.append(corridor_max_x - cursor)
        maximum_free_width = max(free_widths or [0.0])
        row_clearances.append(
            {
                "forward_enu_m": forward,
                "maximum_safe_center_corridor_m": round(maximum_free_width, 3),
            }
        )

    minimum_required_corridor = viewpoint_resolution + 0.20
    minimum_corridor = min(
        (row["maximum_safe_center_corridor_m"] for row in row_clearances),
        default=0.0,
    )
    if minimum_corridor < minimum_required_corridor:
        raise ValueError(
            "obstacle layout has only {:.2f} m safe center corridor; {:.2f} m required".format(
                minimum_corridor, minimum_required_corridor
            )
        )
    sorted_forwards = sorted(rows)
    longitudinal_gaps = [
        sorted_forwards[index + 1]
        - row_half_depths[sorted_forwards[index + 1]]
        - sorted_forwards[index]
        - row_half_depths[sorted_forwards[index]]
        for index in range(len(sorted_forwards) - 1)
    ]
    minimum_longitudinal_gap = min(longitudinal_gaps, default=0.0)
    minimum_required_longitudinal_gap = planner_clearance * 2.0
    if minimum_longitudinal_gap < minimum_required_longitudinal_gap:
        raise ValueError(
            "obstacle rows leave only {:.2f} m longitudinal turning gap; {:.2f} m required".format(
                minimum_longitudinal_gap, minimum_required_longitudinal_gap
            )
        )
    return {
        "vehicle_horizontal_radius_m": vehicle_radius,
        "vehicle_horizontal_diameter_m": vehicle_radius * 2.0,
        "tracking_margin_m": tracking_margin,
        "pointcloud_voxel_margin_m": pointcloud_margin,
        "planner_surface_clearance_m": planner_clearance,
        "viewpoint_resolution_m": viewpoint_resolution,
        "minimum_required_safe_center_corridor_m": minimum_required_corridor,
        "minimum_actual_safe_center_corridor_m": minimum_corridor,
        "minimum_required_longitudinal_turning_gap_m": minimum_required_longitudinal_gap,
        "minimum_actual_longitudinal_turning_gap_m": minimum_longitudinal_gap,
        "obstacle_edge_min_m": min(obstacle_edges),
        "obstacle_edge_max_m": max(obstacle_edges),
        "rows": row_clearances,
    }


def safety_geofence(length, width, margin):
    start_y = -4.0
    end_y = start_y + length
    half_width = width * 0.5
    if margin <= 0.0 or margin >= half_width:
        raise ValueError("safety margin must be positive and smaller than half the tunnel width")
    return {
        "min_x": -half_width + margin,
        "max_x": half_width - margin,
        "min_y": start_y + 0.25 + margin,
        "max_y": end_y - 0.25 - margin,
    }


def ned_to_enu(values):
    return [values[1], values[0], -values[2]]


def choose_asset(client, requested):
    assets = set(client.simListAssets())
    if requested != "auto":
        if requested not in assets:
            raise RuntimeError("AirSim asset is unavailable: " + requested)
        return requested
    for candidate in ("1M_Cube_Chamfer", "Cube"):
        if candidate in assets:
            return candidate
    raise RuntimeError("Blocks does not expose a spawnable cube asset")


def clear_scene(client):
    names = client.simListSceneObjects(PREFIX + ".*")
    removed = 0
    for name in names:
        if client.simDestroyObject(name):
            removed += 1
    return removed


def spawn_scene(client, specs, asset, world_to_map_z_offset=0.0):
    spawned = []
    for index, spec in enumerate(specs):
        center = spec["center_ned"]
        size = spec["size"]
        pose = airsim.Pose(
            airsim.Vector3r(center[0], center[1], center[2]),
            airsim.to_quaternion(0.0, 0.0, 0.0),
        )
        scale = airsim.Vector3r(size[0], size[1], size[2])
        result = client.simSpawnObject(
            spec["name"], asset, pose, scale, physics_enabled=False
        )
        if not result:
            raise RuntimeError("failed to spawn " + spec["name"])
        client.simSetSegmentationObjectID(result, 40 + index, False)
        item = dict(spec)
        item["spawned_name"] = result
        item["center_enu"] = ned_to_enu(center)
        item["center_enu"][2] += world_to_map_z_offset
        item["size_enu"] = [size[1], size[0], size[2]]
        spawned.append(item)
    return spawned


def configure_view_camera(client, vehicle_name, camera_name, camera_fov):
    # Camera 0 is also used by AirSim's Fpv viewport. Moving it behind and
    # above the vehicle keeps the tunnel shell and several obstacle rows in
    # frame instead of filling the screen with the nearest cube face.
    camera_pose = airsim.Pose(
        airsim.Vector3r(-3.0, 0.0, -2.0),
        airsim.to_quaternion(-0.2, 0.0, 0.0),
    )
    client.simSetCameraFov(camera_name, camera_fov, vehicle_name=vehicle_name)
    client.simSetCameraPose(
        camera_name,
        camera_pose,
        vehicle_name=vehicle_name,
    )
    return {
        "camera_name": camera_name,
        "fov_degrees": camera_fov,
        "position_ned_relative": [-3.0, 0.0, -2.0],
        "pitch_radians": -0.2,
    }


def write_boundary(path, bounds):
    if not path:
        return
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    vertices = [
        (bounds["max_x"], bounds["min_y"], 0.0),
        (bounds["min_x"], bounds["min_y"], 0.0),
        (bounds["min_x"], bounds["max_y"], 0.0),
        (bounds["max_x"], bounds["max_y"], 0.0),
        (bounds["max_x"], bounds["min_y"], 0.0),
    ]
    with open(path, "w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\nelement vertex {}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "end_header\n".format(len(vertices))
        )
        for vertex in vertices:
            stream.write("{:.3f} {:.3f} {:.3f}\n".format(*vertex))


def write_report(
    path,
    asset,
    specs,
    removed,
    bounds=None,
    camera=None,
    fixed_flight_height=None,
    world_to_map_z_offset=None,
    safety_model=None,
):
    report = {
        "asset": asset,
        "removed_previous_objects": removed,
        "object_count": len(specs),
        "objects": specs,
    }
    if bounds is not None:
        report["safety_geofence_enu"] = bounds
    if camera is not None:
        report["view_camera"] = camera
    if fixed_flight_height is not None and world_to_map_z_offset is not None:
        report["fixed_flight_height_map"] = fixed_flight_height
        report["world_to_map_translation_enu"] = [
            0.0,
            0.0,
            world_to_map_z_offset,
        ]
    if safety_model is not None:
        report["safety_model"] = safety_model
    if path:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))


def parse_args():
    parser = argparse.ArgumentParser(description="Build a compact cube tunnel in AirSim Blocks")
    parser.add_argument("--host", default="host.docker.internal")
    parser.add_argument("--port", type=int, default=41452)
    parser.add_argument("--length", type=float, default=30.0)
    parser.add_argument("--width", type=float, default=7.0)
    parser.add_argument("--height", type=float, default=4.5)
    parser.add_argument("--safety-margin", type=float, default=0.75)
    parser.add_argument("--fixed-flight-height", type=float, default=1.5)
    parser.add_argument("--world-to-map-z-offset", type=float, default=0.7)
    parser.add_argument("--asset", default="auto")
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="0")
    parser.add_argument("--camera-fov", type=float, default=110.0)
    parser.add_argument("--skip-camera-setup", action="store_true")
    parser.add_argument("--report", default="/data/tunnel_scene.json")
    parser.add_argument("--boundary-report", default="/data/tunnel_boundary.ply")
    parser.add_argument("--clear-only", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    client = airsim.VehicleClient(ip=args.host, port=args.port)
    client.confirmConnection()
    removed = clear_scene(client)
    if args.clear_only:
        write_report(args.report, None, [], removed)
        if args.boundary_report and os.path.exists(args.boundary_report):
            os.remove(args.boundary_report)
        return

    asset = choose_asset(client, args.asset)
    specs = build_specs(
        args.length,
        args.width,
        args.height,
        args.fixed_flight_height,
        args.world_to_map_z_offset,
    )
    bounds = safety_geofence(args.length, args.width, args.safety_margin)
    safety_model = build_safety_model(
        specs,
        args.width,
        args.safety_margin,
        args.fixed_flight_height,
        args.world_to_map_z_offset,
    )
    spawned = spawn_scene(
        client, specs, asset, args.world_to_map_z_offset
    )
    camera = None
    if not args.skip_camera_setup:
        camera = configure_view_camera(
            client,
            args.vehicle_name,
            args.camera_name,
            args.camera_fov,
        )
    time.sleep(0.5)
    write_boundary(args.boundary_report, bounds)
    write_report(
        args.report,
        asset,
        spawned,
        removed,
        bounds,
        camera,
        args.fixed_flight_height,
        args.world_to_map_z_offset,
        safety_model,
    )


if __name__ == "__main__":
    main()
