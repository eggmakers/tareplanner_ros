#!/usr/bin/env python3
import sys

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

try:
    import airsim
except ImportError as exc:
    airsim = None
    AIRSIM_IMPORT_ERROR = exc
else:
    AIRSIM_IMPORT_ERROR = None


class AirSimLidarBridge:
    def __init__(self):
        self.host = rospy.get_param("~host", "host.docker.internal")
        self.port = int(rospy.get_param("~port", 41452))
        self.vehicle_name = rospy.get_param("~vehicle_name", "PX4")
        self.lidar_name = rospy.get_param("~lidar_name", "LidarSensor1")
        self.output_topic = rospy.get_param("~output_topic", "/airsim/registered_scan")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.input_frame = rospy.get_param("~input_frame", "ned_world")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.rpc_timeout = float(rospy.get_param("~rpc_timeout", 3.0))
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 2.0))
        self.world_to_map_offset = np.asarray(
            [
                float(rospy.get_param("~world_to_map_x_offset", 0.0)),
                float(rospy.get_param("~world_to_map_y_offset", 0.0)),
                float(rospy.get_param("~world_to_map_z_offset", 0.0)),
            ],
            dtype=np.float32,
        )

        if airsim is None:
            rospy.logfatal("airsim_lidar_bridge: cannot import airsim: %s", AIRSIM_IMPORT_ERROR)
            raise RuntimeError("AirSim Python client is not installed")
        if self.input_frame not in ("ned_world", "enu_world"):
            raise ValueError("~input_frame must be 'ned_world' or 'enu_world'")
        if self.publish_rate <= 0.0:
            rospy.logwarn("airsim_lidar_bridge: invalid publish_rate %.3f, using 10 Hz", self.publish_rate)
            self.publish_rate = 10.0

        self.client = None
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        rospy.loginfo(
            "airsim_lidar_bridge: AirSim %s:%d vehicle=%s lidar=%s -> %s (%s)",
            self.host,
            self.port,
            self.vehicle_name,
            self.lidar_name,
            self.output_topic,
            self.frame_id,
        )
        rospy.logwarn(
            "airsim_lidar_bridge: AirSim world -> map translation "
            "ENU [%.3f, %.3f, %.3f] m",
            self.world_to_map_offset[0],
            self.world_to_map_offset[1],
            self.world_to_map_offset[2],
        )

    def connect(self):
        try:
            client = airsim.MultirotorClient(
                ip=self.host,
                port=self.port,
                timeout_value=self.rpc_timeout,
            )
            if not client.ping():
                raise RuntimeError("AirSim ping returned false")
            self.client = client
            rospy.loginfo("airsim_lidar_bridge: connected to AirSim RPC server")
            return True
        except Exception as exc:
            self.client = None
            rospy.logwarn_throttle(10.0, "airsim_lidar_bridge: waiting for AirSim: %s", exc)
            return False

    def read_points(self):
        lidar_data = self.client.getLidarData(self.lidar_name, self.vehicle_name)
        flat_points = np.asarray(lidar_data.point_cloud, dtype=np.float32)
        valid_length = flat_points.size - (flat_points.size % 3)
        if valid_length == 0:
            return np.empty((0, 3), dtype=np.float32)
        if valid_length != flat_points.size:
            rospy.logwarn_throttle(
                10.0,
                "airsim_lidar_bridge: dropping %d malformed lidar values",
                flat_points.size - valid_length,
            )
        points = flat_points[:valid_length].reshape((-1, 3))
        points = points[np.isfinite(points).all(axis=1)]

        if self.input_frame == "ned_world":
            # AirSim world NED (north, east, down) -> ROS ENU (east, north, up).
            points = points[:, [1, 0, 2]].copy()
            points[:, 2] *= -1.0
        points += self.world_to_map_offset
        return points

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.client is None and not self.connect():
                rospy.sleep(self.reconnect_delay)
                continue
            try:
                points = self.read_points()
                if points.shape[0] == 0:
                    rospy.logwarn_throttle(
                        10.0,
                        "airsim_lidar_bridge: lidar '%s' returned no points; check settings.json",
                        self.lidar_name,
                    )
                else:
                    header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
                    self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            except Exception as exc:
                rospy.logwarn_throttle(10.0, "airsim_lidar_bridge: lidar read failed: %s", exc)
                self.client = None
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("airsim_lidar_bridge")
    try:
        AirSimLidarBridge().spin()
    except (RuntimeError, ValueError) as exc:
        rospy.logfatal("airsim_lidar_bridge: %s", exc)
        sys.exit(1)
