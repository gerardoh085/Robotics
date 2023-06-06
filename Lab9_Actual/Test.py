
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

plt.show()

# print shortest path:
path = nx.shortest_path(G3, source=newList[0], target=newList[9])
print(path)

x_d = [(path[i][0]) for i in range(len(path))]
y_d = [(path[i][1]) for i in range(len(path))]
# print(path[0][0])
print(x_d)
print(y_d)
