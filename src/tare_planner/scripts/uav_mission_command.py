#!/usr/bin/env python3

import sys

import rospy
from std_srvs.srv import Trigger


SERVICES = {
    "explore": "/uav_mission_mux/select_tare",
    "tare": "/uav_mission_mux/select_tare",
    "plan": "/uav_mission_mux/select_fixed_start",
    "saved": "/uav_mission_mux/select_saved_map",
    "replay": "/uav_mission_mux/select_replay",
    "hold": "/uav_mission_mux/hold",
    "clear-emergency": "/uav_mission_mux/clear_emergency",
}


def call(service_name, timeout):
    rospy.wait_for_service(service_name, timeout=timeout)
    response = rospy.ServiceProxy(service_name, Trigger)()
    if not response.success:
        raise RuntimeError(response.message)
    rospy.loginfo(response.message)


def main():
    rospy.init_node("uav_mission_command", anonymous=True)
    command = rospy.get_param("~command", sys.argv[1] if len(sys.argv) > 1 else "")
    if command not in SERVICES and command not in ("replay-stop", "validate", "export"):
        raise SystemExit(
            "usage: uav_mission_command.py "
            "{explore|plan|saved|replay|replay-stop|hold|clear-emergency|validate|export}")
    timeout = float(rospy.get_param("~timeout", 8.0))
    try:
        if command == "replay":
            call(SERVICES[command], timeout)
            call("/uav_path_replayer/start", timeout)
        elif command == "replay-stop":
            call("/uav_path_replayer/stop", timeout)
            call(SERVICES["hold"], timeout)
        elif command == "validate":
            call("/uav_bundle_validator/validate", timeout)
        elif command == "export":
            call("/uav_path_exporter/save", timeout)
        else:
            call(SERVICES[command], timeout)
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        raise SystemExit(str(error))


if __name__ == "__main__":
    main()
