#!/usr/bin/env python3
import sys

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


MODES = {
    "land": "AUTO.LAND",
    "rtl": "AUTO.RTL",
    "hold": "POSCTL",
}


def main():
    rospy.init_node("uav_mode_command", anonymous=True)
    command = rospy.get_param("~command", sys.argv[1] if len(sys.argv) > 1 else "")
    if command not in MODES:
        raise SystemExit("usage: uav_mode_command.py {land|rtl|hold}")

    state_topic = rospy.get_param("~state_topic", "/mavros/state")
    service_name = rospy.get_param("~set_mode_service", "/mavros/set_mode")
    timeout = float(rospy.get_param("~timeout", 8.0))
    try:
        state = rospy.wait_for_message(state_topic, State, timeout=timeout)
    except rospy.ROSException as exc:
        raise SystemExit("no MAVROS state: {}".format(exc))
    if not state.connected:
        raise SystemExit("MAVROS is not connected to the flight controller")
    if not state.armed and command in ("land", "rtl"):
        rospy.loginfo("vehicle is already disarmed; %s is not required", command)
        return

    rospy.wait_for_service(service_name, timeout=timeout)
    mode = MODES[command]
    response = rospy.ServiceProxy(service_name, SetMode)(
        base_mode=0, custom_mode=mode
    )
    if not response.mode_sent:
        raise SystemExit("PX4 rejected mode request: " + mode)

    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            state = rospy.wait_for_message(state_topic, State, timeout=1.0)
        except rospy.ROSException:
            continue
        if state.mode == mode:
            rospy.loginfo("PX4 mode confirmed: %s", mode)
            return
    raise SystemExit("PX4 did not confirm mode: " + mode)


if __name__ == "__main__":
    main()
