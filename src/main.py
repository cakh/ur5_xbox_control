from ur5_control.ur5_controller import UR5Controller
from xbox.xbox_control import XboxController
import time
import threading

class UR5JoyControl:
    """
    A class to control a UR5 robot using an Xbox controller.
    
    This class reads joystick inputs continuously and sends motion commands 
    to the UR5 robot.
    """
    def __init__(self):

        self.ur5e = UR5Controller(robot_ip="localhost")
        self.joy = XboxController()
        self.motion_input = (0,0,0,0,0)
        self.running = True
        
    def read_joystick(self):
        """
        Continuously reads input from the Xbox controller.
        Updates `self.motion_input` with the latest joystick values.
        """
        while self.running:
            self.motion_input = self.joy.read()
            time.sleep(0.1)

    def control_robot(self):
        """
        Continuously sends joystick input to move the UR5 robot.
        """
        try:
            while self.running:
                self.ur5e.move_robot(self.motion_input)  # Send motion input to the robot
                time.sleep(0.1)
        except KeyboardInterrupt:
            # Stop controller and robot motion
            self.running = False
            print("Control Interrupted!")
            self.ur5e.rtde_c.servoStop()
            self.ur5e.rtde_c.stopScript()

    def start(self):
        """
        Starts joystick input reading and robot control in separate threads.
        Keeps the main thread alive.
        """
        joystick_thread = threading.Thread(target=self.read_joystick, daemon=True)     
        robot_thread = threading.Thread(target=self.control_robot, daemon=True)       
        joystick_thread.start() 
        robot_thread.start()    
        while True:             # Keep the main thread running to prevent exit
            time.sleep(0.5)

if __name__ == "__main__":
    controller = UR5JoyControl()
    controller.start()
