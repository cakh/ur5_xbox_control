"""Gripper-only test: toggles the Hand-E twice and shows the gripper's replies on the pendant.
Keep fingers clear. Usage: python gripper_test.py [robot_ip]"""
import sys
import time

from rtde_control import RTDEControlInterface

from ur_control.gripper import Gripper

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.3.3"
c = RTDEControlInterface(ip)
try:
    g = Gripper(ip, rtde_c=c, debug=True)
    for _ in range(2):
        g.toggle()
        time.sleep(3)
finally:
    c.stopScript()
