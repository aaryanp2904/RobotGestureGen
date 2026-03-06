# nao_server.py - RUN IN PYTHON 2.7
from SimpleXMLRPCServer import SimpleXMLRPCServer
from naoqi import ALProxy
import sys

class NaoBridge:
    def __init__(self, robot_ip, robot_port):
        print("Connecting to NAO at {}:{}...".format(robot_ip, robot_port))
        self.motion = ALProxy("ALMotion", robot_ip, robot_port)
        self.posture = ALProxy("ALRobotPosture", robot_ip, robot_port)
        self.life = ALProxy("ALAutonomousLife", robot_ip, robot_port)

        # Disable autonomous life to prevent it from interfering with our animation
        if self.life.getState() != "disabled":
            self.life.setState("disabled")

        self.motion.wakeUp()
        self.posture.goToPosture("StandInit", 0.5)
        print("NAO is initialized and listening for commands on port 8000.")

    def play_trajectory(self, names, angles, times):
        """Receives trajectory arrays from Python 3 and executes them."""
        print("Received trajectory payload for {} joints. Executing...".format(len(names)))
        # The 'True' flag means these are absolute angles, not relative
        self.motion.angleInterpolation(names, angles, times, True)
        print("Trajectory complete.")
        return True
    
    def rest(self):
        """Returns the robot to a resting state."""
        print("Returning to rest state.")
        self.posture.goToPosture("StandInit", 0.5)
        self.motion.rest()
        return True

if __name__ == "__main__":
    # Allows passing a custom port via command line: python nao_server.py 31559
    robot_port = int(sys.argv[1]) if len(sys.argv) > 1 else 31559
    robot_ip = "127.0.0.1"

    # Start the XML-RPC server on localhost port 8000
    server = SimpleXMLRPCServer(("localhost", 8000), allow_none=True)
    server.register_introspection_functions() # <--- ADD THIS LINE
    server.register_instance(NaoBridge(robot_ip, robot_port))
    
    print("Python 2 Bridge Server running. Waiting for Python 3 client...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")