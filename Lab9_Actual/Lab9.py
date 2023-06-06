
import numpy as np
import sys
import math
import time
import socket
from util import quaternion_to_euler_angle_vectorized1
from NatNetClient import NatNetClient
import networkx as nx
import matplotlib.pyplot as plt


newDict = {

    1: (-3.87, 3.22),
    2: (-3.85, 1.84),
    3: (-3.05, 1.39),
    4: (-1.91, 1.34),
    5: (-1.75, 2.44),
    6: (-1.77, 3.54),
    7: (-0.61, 2.31),
    8: (0.64, 2.54),
    9: (0.20, 1.54),
    10: (0.72, 3.95)

}

newList = list(newDict.values())


G3 = nx.Graph()

for i in (newList):
    G3.add_node((i))


G3.add_edge(newList[0], newList[1])
G3.add_edge(newList[0], newList[5])
G3.add_edge(newList[1], newList[2])
G3.add_edge(newList[2], newList[3])
G3.add_edge(newList[3], newList[4])
G3.add_edge(newList[3], newList[6])
G3.add_edge(newList[4], newList[5])
G3.add_edge(newList[4], newList[6])
G3.add_edge(newList[5], newList[0])
G3.add_edge(newList[6], newList[7])
G3.add_edge(newList[6], newList[8])
G3.add_edge(newList[7], newList[9])
G3.add_edge(newList[8], newList[7])


pos = {
    newList[0]: (1, 5),
    newList[1]: (1, 1),
    newList[2]: (3, 0),
    newList[3]: (5, 0),
    newList[4]: (5, 3),
    newList[5]: (5, 5),
    newList[6]: (7, 2),
    newList[7]: (9, 2),
    newList[8]: (8, 0),
    newList[9]: (9, 5),

}


# print(newList[0])

nx.draw(G3, pos=pos, with_labels=True, node_color="red",
        node_size=2500, font_color="white", font_size=8, font_weight="bold", width=4)
plt.margins(0.2)

# shows undirected graph:
plt.show()

# print shortest path:
path = nx.shortest_path(G3, source=newList[0], target=newList[9])
print(path)


IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


positions = {}
rotations = {}
distanceList = []
orientationError = []
time_taken = []
index = 0

x_d = [(path[i][0]) for i in range(len(path))]
y_d = [(path[i][1]) for i in range(len(path))]


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    # print()
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(
        rotation_quaternion)

    rotations[robot_id] = rotz


def funcTest():
    global rotations, robot_id, positions, index

    # print("index: ", index)

    # gain
    k_w = 35
    k_v = 1450

    x = positions[robot_id][0]
    y = positions[robot_id][1]
    print(f"current: ({x}, {y})")

    theta = math.radians(rotations[robot_id])

    distance = math.sqrt(((x_d[index] - x)**2) + ((y_d[index] - y)**2))
    print("distance: ", distance)
    v = k_v * distance
    # print("linear velocity: ",v)

    alpha = (math.atan2((y_d[index] - y), (x_d[index] - x)))
    print()

    w = k_w * \
        math.degrees(math.atan2(
            (math.sin(alpha - theta)), math.cos(alpha - theta)))

    u = np.array([v - w, v + w])
    u[u > 1500] = 1500
    u[u < -1500] = -1500

    command = 'CMD_MOTOR#%d#%d#%d#%d\n' % (u[0], u[0], u[1], u[1])
    s.send(command.encode('utf-8'))
    # t += 0.5
    time.sleep(0.1)
    if distance < 0.45 and index < len(x_d):
        command = 'CMD_MOTOR#00#00#00#00\n'
        index += 1
        s.send(command.encode('utf-8'))
    if (index == len(x_d)):
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        sys.exit("Program has finished!")


try:
    if __name__ == "__main__":
        clientAddress = "192.168.0.115"
        optitrackServerAddress = "192.168.0.172"
        robot_id = 300

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
                    # for i in range(len(x_d)):
                    funcTest()
                # break

        except KeyboardInterrupt:
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
            s.shutdown(2)
            s.close()


except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    # Close the connection
    s.shutdown(2)
    s.close()
    sys.exit("Exiting Program!")
