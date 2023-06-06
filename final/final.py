
import numpy as np
import sys
import math
import time
import socket
from util import quaternion_to_euler_angle_vectorized1
from NatNetClient import NatNetClient
import networkx as nx
import matplotlib.pyplot as plt
import random
import matplotlib.path as mpath
import networkx as nx
import matplotlib.pyplot as plt
from shapely import Point, LineString
import numpy as np
import cv2 as cv

# Create an empty graph
G = nx.Graph()

# Define the number of rows and columns
num_rows = 6
num_columns = 6

# Define the x and y ranges
# yr = (3.78, -2.38)
# xr = (5.649, -4.15)
xr = (3.88, 5.56)
yr = (-2.38, 3.78)
# Calculate the increment for x and y
xi = (xr[1] - xr[0]) / num_columns
yi = (yr[1] - yr[0]) / num_rows

# Generate nodes and edges for the organized pattern
for row in range(num_rows):
    for col in range(num_columns):
        # Generate the node label as a tuple with coordinate values formatted as floats
        node_label = (round(xr[0] + xi * col, 2), round(yr[0] + yi * row, 2))
        G.add_node(node_label)

        if col > 0:
            left_node_label = (
                round(xr[0] + xi * (col - 1), 2), round(yr[0] + yi * row, 2))
            G.add_edge(node_label, left_node_label)

        if col < num_columns - 1:
            right_node_label = (
                round(xr[0] + xi * (col + 1), 2), round(yr[0] + yi * row, 2))
            G.add_edge(node_label, right_node_label)

        if row > 0:
            top_node_label = (round(xr[0] + xi * col, 2),
                              round(yr[0] + yi * (row - 1), 2))
            G.add_edge(node_label, top_node_label)

        if row < num_rows - 1:
            bottom_node_label = (
                round(xr[0] + xi * col, 2), round(yr[0] + yi * (row + 1), 2))
            G.add_edge(node_label, bottom_node_label)

        if col > 0 and row > 0:
            top_left_node_label = (
                round(xr[0] + xi * (col - 1), 2), round(yr[0] + yi * (row - 1), 2))
            G.add_edge(node_label, top_left_node_label)

        if col > 0 and row < num_rows - 1:
            bottom_left_node_label = (
                round(xr[0] + xi * (col - 1), 2), round(yr[0] + yi * (row + 1), 2))
            G.add_edge(node_label, bottom_left_node_label)

        if col < num_columns - 1 and row > 0:
            top_right_node_label = (
                round(xr[0] + xi * (col + 1), 2), round(yr[0] + yi * (row - 1), 2))
            G.add_edge(node_label, top_right_node_label)

        if col < num_columns - 1 and row < num_rows - 1:
            bottom_right_node_label = (
                round(xr[0] + xi * (col + 1), 2), round(yr[0] + yi * (row + 1), 2))
            G.add_edge(node_label, bottom_right_node_label)

# Set the positions of the nodes based on their labels
pos = {node_label: node_label for node_label in G.nodes()}
print(G.nodes())

# Add new nodes to the graph
new_nodes = ['A', 'B', 'C']
G.add_nodes_from(new_nodes)

# Specify positions for the new nodes
pos.update({'A': (1.5, -1.5), 'B': (-2.5, -2.5), 'C': (3.5, 3.5)})

# Add edges between new nodes to form a triangle
G.add_edge('A', 'B')
G.add_edge('B', 'C')
G.add_edge('C', 'A')
# Visualize the updated graph
nx.draw(G, pos, with_labels=True, node_size=500,
        node_color='lightblue', font_size=8)
# plt.axis('equal')
# plt.xlim(*(-5, 5))
# plt.ylim(*(-5, 5))
plt.show()


# Create a list to store the nodes to be removed
nodes_to_remove = []
edges_to_remove = []

# Check for intersection of edges with the line connecting 'A', 'B', and 'C'
line_ab = LineString([pos['A'], pos['B']])
line_bc = LineString([pos['B'], pos['C']])
line_ca = LineString([pos['C'], pos['A']])

for u, v in G.edges:
    line_uv = LineString([pos[u], pos[v]])
    if line_uv.intersects(line_ab) or line_uv.intersects(line_bc) or line_uv.intersects(line_ca):
        edges_to_remove.append((u, v))

# Check for intersection of nodes with the line connecting 'A', 'B', and 'C'
for node_label, coord in pos.items():
    if node_label in new_nodes:
        continue  # Skip the target nodes 'A', 'B', and 'C'

    line_node = LineString([pos['A'], pos['C']])
    if line_node.contains(Point(coord)):
        nodes_to_remove.append(node_label)

# Remove the edges and nodes from the graph
G.remove_edges_from(edges_to_remove)
G.remove_nodes_from(nodes_to_remove)

# Remove nodes that have a degree <2
remove_nodes = [node for node, degree in dict(
    G.degree()).items() if degree < 2]
print(f"Removed nodes Degree < 2: {remove_nodes}")
G.remove_nodes_from(remove_nodes)
# Remove the corresponding coordinates from pos
pos = {node: coord for node, coord in pos.items() if node not in remove_nodes}


# if nodes are isolated:
G.remove_nodes_from(list(nx.isolates(G)))
# Remove the corresponding coordinates from pos
pos = {node: coord for node, coord in pos.items() if node not in remove_nodes}

# given values for obstacles in the evironment
# remove nodes and edges that are within the box
A = (1.5, -1.5)
B = (-2.5, -2.5)
C = (3.5, 3.5)

# Create the polygon path using A, B, and C
polygon_path = mpath.Path([A, B, C, A])


# Iterate over the nodes and their positions
for node_label, node_position in pos.items():
    if polygon_path.contains_point(node_position):
        nodes_to_remove.append(node_label)
        edges_to_remove.extend(G.edges(node_label))


# Remove the edges within the shape formed by A, B, and C
G.remove_edges_from(edges_to_remove)

# Remove the nodes within the shape formed by A, B, and C
G.remove_nodes_from(nodes_to_remove)


# get the coords in a list
co = nx.get_node_attributes(G, 'pos')
for node, coord in pos.items():
    print(f"node {node}: coord {coord}")
coordList = list(pos.values())
print(coordList)

# xList = [(coordList[i][0]) for i in range(len(coordList))]
# print(xList)
# yList = [(coordList[i][1]) for i in range(len(coordList))]
# print(yList)


print(f'GNODES: {G.nodes}')

gList = list(G.nodes())


source = (4.72, 2.75)
target = (4.16, -2.38)
# source = (4.72, 2.75)
# target = (5.0, 2.75)


def random_path_dfs(graph, start, end):
    # Create a stack to store the nodes during the depth-first search
    stack = [(start, [start])]

    while stack:
        node, path = stack.pop()

        # Check if the current node is the target node
        if node == end:
            return path

        # Get the neighbors of the current node
        neighbors = list(graph.neighbors(node))

        # Shuffle the neighbors to visit them in a random order
        random.shuffle(neighbors)

        # Iterate over the neighbors
        for neighbor in neighbors:
            # Check if the neighbor is not already visited
            if neighbor not in path:
                # Push the neighbor and the updated path to the stack
                stack.append((neighbor, path + [neighbor]))

    # If no path is found, return an empty path
    return []

# Usage:
# source = coordList[0]
# target = coordList[9]


# Find a random path using DFS
path = random_path_dfs(G, source, target)


print(f'PATHS: {path}')

# once x y lens == max then def () global xd yd path
# path = random_path_dfs(G, start, end)
x_d = [(path[i][0]) for i in range(len(path))]
y_d = [(path[i][1]) for i in range(len(path))]

# Visualize the updated graph
nx.draw(G, pos, with_labels=True, node_size=700,
        node_color='lightblue', font_size=6)

plt.show()


IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')


positions = {}
rotations = {}
index = 0

# x_d = [(path[i][0]) for i in range(len(path))]
# y_d = [(path[i][1]) for i in range(len(path))]

global world_vec
world_vec = [0, 0]


def SbLinApprox(s):
    # 1/sb vs dist
    # y = 0.0025x - 0.0002
    Sb = 1/s
    res = (0.0025 * Sb) - 0.0002
    # print("Sb approx: ", res)
    return res * -1 * 55108.012


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    # print()
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(
        rotation_quaternion)

    rotations[robot_id] = rotz


# replace with picam2

cap = cv.VideoCapture('http://192.168.0.205:3000/stream.mjpg')
cnt_swap = 0

# List of coordinates
# coordinates = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)]

# Variables to store the previous and current random coordinates
previous_coordinate = None
current_coordinate = None


def get_random_coordinate():
    global current_coordinate, previous_coordinate

    # If all coordinates have been used, reset the previous_coordinate variable
    if previous_coordinate is not None and len(path) == 1:
        previous_coordinate = None

    # Get a random coordinate
    while True:
        current_coordinate = random.choice(path)
        if current_coordinate != previous_coordinate:
            break

    return current_coordinate


def detect_duck():

    # incorporate the optitrack system:
    global rotations, robot_id, positions, index, path

    # print("index: ", index)

    # gain
    k_w = 35
    k_v = 1450

    x = positions[robot_id][0]
    y = positions[robot_id][1]
    print(f"current: ({x}, {y})")

    theta = math.radians(rotations[robot_id])

    found_duck = False

    # if is_running and (robot_id in positions):

    # ============================ Setting Up the Mask and Gray Image ============================#
    # After recieving the camera data the mask is used to filter out everything that is not yello (since the ducks are yellow)
    # the difficult part is how do you decipher the difference between yellow ducks and yello tap or other objects
    # whatever system we use need a way to decipher ducks from everything else

    ret, img = cap.read()
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # upper and lower bounds for the mask
    lower = np.array([20, 100, 100])
    upper = np.array([30, 255, 255])

    # creating mask
    mask = cv.inRange(hsv, lower, upper)

    # masking image
    masked_img = cv.bitwise_and(img, img, mask=mask)

    # converting original image back gray for the blob detector
    gray_img = cv.cvtColor(masked_img, cv.COLOR_BGR2GRAY)

    # =========================== Setting Up Detector ============================#

    # nb: using RETR_EXTERNAL this disregards the countors found within countors and focuses on
    # the outside. th CHAIN_APPROX_SIMPLE runs faster than the alternative
    cnts = cv.findContours(gray_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    # =================== Calculating Ducks px and py Vectors =====================#
    for c in cnts:
        # create rectangular shape on the object
        x, y, w, h = cv.boundingRect(c)

        area = cv.contourArea(c)
        # print("area: " + str(area) + " w: " + str(w) + " h: " + str(h))
        # small area father away
        # big area closer
        portion = abs(w - h)
        # filter
        if area < 2000 or area > 8000 or portion > 3:
            # if portion > 3 or area < 2000 or area > 9000: TRY ME
            # if area >  9000:
            continue
        found_duck = True
        u_value = x + (w / 2)
        est_dist = SbLinApprox(area)
        print("u value == " + str(u_value) + " area == " +
              str(area) + "est dist == " + str(est_dist))
        # Now we need to find px and py

        # these values need to be fixed
        # in some units need to scale into the proper units
        px = (u_value - 320) / 96
        theta = np.arcsin(px / est_dist) * 57.2958

        # print("x distance == " + str(px))
        # theta = np.arcsin(px / est_dist)# * 57.2958
        # print("theta == " + str(theta))
        py = px / (np.tan(theta * 0.0174533))

        print("px == " + str(px) + "py == " + str(py))

        p_vec = [px,
                 py]
        # change this value to optitrack recieved value
        world_theta = rotations[robot_id]
        rot_matrix = [[np.cos(world_theta), np.sin(world_theta) * -1],
                      [np.sin(world_theta), np.cos(world_theta)]]
        temp_world_vec = np.cross(p_vec, rot_matrix)
        # world_vec

        # world vec are x and u position of the duck
        world_vec[0] = (world_vec[0] * 0.0254) + positions[robot_id][0]
        world_vec[1] = (world_vec[1] * 0.0254) + positions[robot_id][1]
        print(world_vec)

        # finally take the robots x and y and add it to the world_vec tor get the ducks world coordinates
        x_d = world_vec[0]
        y_d = world_vec[1]
        # now that we have the px and py vector need to convert it to the world coordinates

        # If it makes it to this point then the robot should go towards the duck
        cv.rectangle(img, (x, y), (x+w, y+h), (36, 255, 12), 2)

    if detect_duck:

        distance = math.sqrt(((x_d[index] - x)**2) + ((y_d[index] - y)**2))
        print("distance: ", distance)
        v = k_v * distance
        # print("linear velocity: ",v)

        alpha = (math.atan2((y_d - y), (x_d - x)))
        # print()

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
        if distance < 0.45:

            # move on to next desired position
            # index += 1
            x_d = 4.72
            y_d = 2.75

            index = 0

            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
    else:
        distance = math.sqrt(
            ((x_d[index] - x)**2) + ((y_d[index] - y)**2))
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

            # move on to next desired position
            index += 1

            s.send(command.encode('utf-8'))

    if (index == len(x_d)):

        if cnt_swap == 0:
            source = (4.72, 2.75)
            target = get_random_coordinate()
            cnt_swap = 1
        elif cnt_swap == 1:
            target = (4.72, 2.75)
            source = get_random_coordinate()
            cnt_swap = 0

        path = random_path_dfs(G, source, target)
        x_d = [(path[i][0]) for i in range(len(path))]
        y_d = [(path[i][1]) for i in range(len(path))]
        index = 0


try:
    if __name__ == "__main__":
        # clientAddress = "192.168.0.115"
        # optitrackServerAddress = "192.168.0.172"
        # robot_id = 300

        clientAddress = "192.168.0.31"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 203

        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

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
                    # funcTest()
                    detect_duck()
                    # break
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
