import socket
import time
import math 
import sys 
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


positions = {}
rotations = {}
distanceList = []
orientationError = []

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    # print()
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz




def funcTest(rotations,robot_id,positions):
    # initial pos
    x = positions[robot_id][0]
    y = positions[robot_id][1]
    print(f"current: ({x}, {y})")
    print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
    # desired pos 
    x_d = -2
    y_d = -1
    
    print(f'Difference between desired and robot positions: x: {abs(x_d - x):.2f}, y: {abs(y_d - y):.2f}')
    
    # gain for angular and linear velocity
    k_w = 280
    k_v = 1700
    
    # orientation
    theta = math.radians(rotations[robot_id])
    
    # eq for linear velocity
    distance = math.sqrt(((x_d - x)**2) + ((y_d - y)**2))
    print("distance: ",distance)
    v = k_v * distance
    print("linear velocity: ",v)
    
    # eq for alpha
    alpha = (math.atan2((y_d - y), (x_d - x)))
    print()
    # eq for angular velocity
    w = k_w * math.degrees(math.atan2((math.sin(alpha - theta)) , math.cos(alpha - theta)))
    # position controller
    u = np.array([ v - w, v + w])
    u[u > 1500] = 1500
    u[u < -1500] = -1500
    distanceList.append(distance)
    orientationError.append(w)
    # print("position x: ", positions[robot_id][0])
    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
    s.send(command.encode('utf-8'))

    time.sleep(0.1)
    


try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.42"
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
        try:
            while is_running:
                if robot_id in positions:

                    # print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                    # Send control input to the motors
                    funcTest(rotations,robot_id,positions)
                    # funcTest()
                    # Wait for 1 second
                    # time.sleep(1)
                    # last position
        except KeyboardInterrupt:
            
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
            s.shutdown(2)
            s.close()
            print("\n\n")
            print("----------------------------------------")
            print("\n\n")
            print("distance list: ", distanceList)
            print("\n\n")
            print("----------------------------------------")
            print("\n\n")
            print("orientation error (omega): ", orientationError)
            
            sys.exit("Exiting Program!")
            

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    # Close the connection
    s.shutdown(2)
    s.close()
    sys.exit("Exiting Program!")

    
    
# Close the connection
s.shutdown(2)
s.close()


