import socket
import time
import math 
import sys 
import numpy as np

IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

import sys
import time
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
desiredX = []
desiredY = []
RegX = []
RegY = []

t = 0

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    # print()
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz



# def controller(x,y):
#     global rotations,robot_id,positions
    
#     # gain 
#     k_w = 200
#     k_v = 900
    
#     # desired position
#     x_d = -2
#     y_d = -1
#     # x_d = [0, 0, 1, 1, 0]
#     # y_d = [0, 1, 1, 0, 0]
    
#     # angle
#     theta = math.radians(rotations[robot_id]) 
    
#     # linear velocity
#     distance = math.sqrt(((x_d - x)**2) + ((y_d - y)**2))
#     v = k_v * distance

#     # angular velocity
#     alpha = (math.atan2((y_d - y), (x_d - x)))
#     w = k_w * math.degrees(math.atan2((math.sin(alpha - theta)) , math.cos(alpha - theta)))

#     # p controller  
#     u = np.array([ v - w , v + w])
#     u[u > 1500] = 1500
#     u[u < -1500] = -1500

#     return u


def funcTest():
    global rotations,robot_id,positions,t
    
    # gain 
    k_w = 120
    k_v = 2000    

    c = (math.pi/180) * 3.5
    
    x_c = -2
    y_c = 0
    radius = 1
    while True:
        x = positions[robot_id][0]
        y = positions[robot_id][1]
        print(f"current: ({x}, {y})")
        
        x_d = radius*math.cos((c*t)) + x_c
        y_d = radius*math.sin((c*t)) + y_c
        
        print("Time: ", c*t)
        
        theta = math.radians(rotations[robot_id])

        distance = math.sqrt(((x_d - x)**2) + ((y_d - y)**2))
        print("distance: ",distance)
        v = k_v * distance
        # print("linear velocity: ",v)
        

        alpha = (math.atan2((y_d - y), (x_d - x)))
        print()
        
        # eq for angular velocity
        w = k_w * math.degrees(math.atan2((math.sin(alpha - theta)) , math.cos(alpha - theta)))
        
        desiredX.append(x_d)
        desiredY.append(y_d)
        RegX.append(x)
        RegY.append(y)
        
        u = np.array([ v - w, v + w])
        u[u > 1500] = 1500
        u[u < -1500] = -1500
        
        # print("position x: ", positions[robot_id][0])
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
        s.send(command.encode('utf-8'))
        t += 1 
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
                    # funcTest(rotations,robot_id,positions)
                    funcTest()
                
                    # Wait for 1 second
                    # time.sleep(1)
                    # last position
        except KeyboardInterrupt:
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
            s.shutdown(2)
            s.close()
            t=0
            print("\n\n")
            print(f"X_D: {desiredX}")
            print("\n\n")
            print("\n\n")
            print(f"Y_D: {desiredY}")
            print("\n\n")
            print("\n\n")
            print(f"REG X: {RegX}")
            print("\n\n")
            print("\n\n")
            print(f"REG Y: {RegY}")
            print("\n\n")
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






    # # waypoints
    # p1 = [0.,0]
    # p2 = [2,3]
    # p3 = [4,5]
    # p4 = [5,-1]
    # p5 = [1,-1]
    
    
    # '''
    # # waypoints for rectangle 
    # # p1 = [0.,0]
    # # p2 = [0,4]
    # # p3 = [5,4]
    # # p4 = [5,0]
    # # p5 = [0.,0]
    # '''
    
    
    # # velocities:
    # v1 = [0,-0.001]
    # v2 = [-1,1]
    # v3 = [1,0]
    # v4 = [-1,1]
    # v5 = [0,-0.001]
    
    # v1 = [0,-0.001]
    # v2 = [1,1]
    # v3 = [1,0]
    # v4 = [1,1]
    # v5 = [0,-0.001] 
    

    # # Time:
    # t1 = 0
    # t2 = 12
    # t3 = 22
    # t4 = 32
    # t5 = 42   
    # #time:
    # t = 1