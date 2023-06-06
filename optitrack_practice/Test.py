import socket
import time
import math 
import sys 
import numpy as np
import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

# connecting to robot
IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


# code from optitrack_test
positions = {}
rotations = {}


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

# objectives:
#  1. compute the distance between the robot pos and desired
#  pos (any arbitrary point in the environment, e.g., 
# g = [2.0,3.0])

def comp_dis():
    global positions, rotations, robot_id
    
    x = positions[robot_id][0] # assuming x val
    y = positions[robot_id][1] # assuming y val
    
    # get initial pos 
    
    
    pass
    # pos = 
    # return pos



#  2.  Compute desired orientation towards the desired position 
#  (this is the desired orientation). Clue: the arctangent 
#  function might be useful.

#  3. Compute the difference between the
#  desired orientation and the robot orientation 
#  (we call it orientation error).

#  4. Implement a proportional controller to 
# make the robot head toward the desired position, 
# i.e., drive the orientation error to zero. Use the 
# angular velocity as a control input. The following code 
# transforms linear and angular velocities (v, omega) into
# motor inputs:

    # u = np.array([v - omega, v + omega])
    # u[u > 1500] = 1500
    # u[u < -1500] = -1500
    # # Send control input to the motors
    # command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])

#  5. Implement a proportional controller to change 
#  the linear velocity v proportionally to the distance 
#  towards the goal.

#  6. Implement the position controller by 
#  combining the last two previous steps.





if __name__ == "__main__":
    clientAddress = "192.168.0.32"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 205

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    while is_running:
        if robot_id in positions:
            print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
            time.sleep(1)

try:
    while True:
        # Send control input to the motors
        move = None
        print("move: ",move)
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(move[0], move[0], move[1], move[1])
        # command = 'CMD_MOTOR#1500#1500#1500#1500\n'
        s.send(command.encode('utf-8'))

        # Wait for 1 second
        time.sleep(1)
except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    # Close the connection
    s.shutdown(2)
    s.close()
    sys.exit("Exiting Program!")

    
    

