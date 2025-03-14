from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import os
import psutil
import sys

# Define the workspace (motion limits) for the robot in meters
MOTION_LIMITS = {
                "xmin": -0.3, "xmax": 0.3,
                "ymin": -0.3, "ymax": 0.3,
                "zmin": -0.5, "zmax": 0.5,
                }

class UR5Controller:
    """
    A class to control the UR5 robot using RTDE interface.

    This controller sets up RTDE communication, applies real-time priority settings,
    and allows motion control.
    """
    def __init__(self, robot_ip="localhost", rtde_frequency=500.0, ur_cap_port=50002):
        """
        Initializes the UR5 controller with RTDE communication.

        :param robot_ip: IP address of the robot controller (default is "localhost")
        :param rtde_frequency: RTDE communication frequency in Hz (500 for e series and 125 for CB3)
        :param ur_cap_port: Port number for URCap communication
        """
        self.robot_ip = robot_ip
        self.rtde_frequency = rtde_frequency
        self.ur_cap_port = ur_cap_port
        self.dt = 1.0 / rtde_frequency  # Time step (2ms)

        self.vel = 0.01
        self.acc = 0.01
        self.lookahead_time = 0.1   
        self.gain = 600
        self.time_counter = 0.0     # Internal time counter
        self.flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT

        # Real-time priority
        self.rt_receive_priority = 90
        self.rt_control_priority = 85

        # Initialize RTDE Interfaces
        self.rtde_r = RTDEReceive(self.robot_ip, self.rtde_frequency, [], True, False, self.rt_receive_priority)
        self.rtde_c = RTDEControl(self.robot_ip, self.rtde_frequency, self.flags, self.ur_cap_port, self.rt_control_priority)

        # Set real-time priority
        self.set_real_time_priority()

        # Move to initial position
        self.init_position()

    def set_real_time_priority(self):
        """
        Sets the process to real-time priority based on the operating system.

        - Windows: Uses `REALTIME_PRIORITY_CLASS`
        - Linux: Uses `SCHED_FIFO` scheduling with priority 80
        """
        os_used = sys.platform
        process = psutil.Process(os.getpid())

        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)
    
    def init_position(self):
        """
        Moves the robot to an initial safe position.
        """
        self.actual_tcp_pose = self.rtde_r.getActualTCPPose()   # Get the current TCP pose
        motion_input = (0,0,0,0,0)
        init_pose = self.getTarget(self.actual_tcp_pose, self.time_counter, motion_input)   # Compute target pose
        self.rtde_c.moveL(init_pose, self.vel, self.acc)     # Move to initial position

    def getTarget(self, pose, timestep, motion_input, freq = 10.0):
        """
        Calculates the next target pose based on joystick input.

        :param pose: Current TCP pose [x, y, z, rx, ry, rz]
        :param timestep: Current time step
        :param motion_input: Tuple (dx, dy, dz_up, d_z_down, button_A) from joystick
        :param freq: Frequency scaling factor
        :return: New target pose within motion limits
        """
        target = pose[:]    # Copy current pose

        # Update X and Y positions based on joystick input
        for index in range (0,2):
            target[index]+=0.01*freq*timestep*motion_input[index]
        
        # Update Z positions based on joystick input
        target[3]+=0.01*freq*timestep*(motion_input[3])

        # Ensure the target is within defined motion limits
        target[0] = max(MOTION_LIMITS["xmin"], min(target[0], MOTION_LIMITS["xmax"]))
        target[1] = max(MOTION_LIMITS["ymin"], min(target[1], MOTION_LIMITS["ymax"]))
        target[2] = max(MOTION_LIMITS["zmin"], min(target[2], MOTION_LIMITS["zmax"]))

        return target
    
    def move_robot(self, xbox_input):
        """
        Moves the robot based on Xbox controller input.

        :param xbox_input: Tuple (dx, dy, dz_up, d_z_down, button_A) from joystick
        """
        self.actual_tcp_pose = self.rtde_r.getActualTCPPose()   # Get the latest TCP pose from the robot
        t_start = self.rtde_c.initPeriod()  # Initialize the RTDE motion period
        #print(xbox_input)
        servo_target = self.getTarget(self.actual_tcp_pose, self.time_counter, xbox_input)        # Compute the target position based on joystick input
        self.rtde_c.servoL(servo_target, self.vel, self.acc, self.dt, self.lookahead_time, self.gain)        # Send the servo motion command
        self.rtde_c.waitPeriod(t_start)        # Wait for the RTDE period to synchronize execution
        self.time_counter += self.dt        # Increment the internal time counter


