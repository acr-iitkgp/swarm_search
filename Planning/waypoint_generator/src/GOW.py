import AHP
import numpy as np
import math
from geopy.distance import geodesic
from geometry_msgs.msg import Point


class Node(object):

	def __init__(self, name):
	    self.name = name
	    self.neighbour = []

	def addNeighbour(self, obj):
	    self.neighbour.append(obj)


def generateOrderedWaypoints(pos_array, curr_pos, path):

	node_array = []
	for i in range(len(pos_array)):
	    node_array.append(Node(i))

	for i in range(len(path)):
	    node_array[path[i][0]].addNeighbour(node_array[path[i][1]])
	    node_array[path[i][1]].addNeighbour(node_array[path[i][0]])

	min_dist = 1e7
	min_node = 0
	end_nodes = []
	
	order_waypt = []
	order_waypt.append(curr_pos)

	if len(pos_array) > 1:
		for i in range(len(pos_array)):
			#distance = math.sqrt(math.pow(pos_array[i].x - curr_pos.x, 2) + math.pow(pos_array[i].y - curr_pos.y, 2))
			distance = geodesic((pos_array[i].x, pos_array[i].y), (curr_pos.x, curr_pos.y)).m
			if(len(node_array[i].neighbour) == 1):
				end_nodes.append(i)
				if(distance < min_dist):
					min_dist = distance
					min_node = i

		end_nodes.remove(min_node)
		end_node = end_nodes[0]

		order_waypt.append(pos_array[min_node])
		prev_node = min_node
		next_node = node_array[prev_node].neighbour[0].name
		while(next_node != end_node):
			# print(prev_node)
			order_waypt.append(pos_array[next_node])
			node_array[next_node].neighbour.remove(node_array[prev_node])
			prev_node = next_node
			next_node = node_array[prev_node].neighbour[0].name

		order_waypt.append(pos_array[end_node])
	elif len(pos_array) == 1:
		order_waypt.append(pos_array[0])

	final_point = Point()
	final_point.x = 0;
	final_point.y = 0;
	order_waypt.append(final_point)

	return order_waypt


# # array of random positions
# curr_pos = np.random.rand(2)
# pos_array = np.random.rand(4, 2)
# print(curr_pos)
# print(pos_array)

# edge = [  # u, v, w
#     AHP.Edge(0, 1, 1),
#     AHP.Edge(0, 2, 1),
#     AHP.Edge(0, 3, 1),
#     AHP.Edge(1, 2, 2),
#     AHP.Edge(1, 3, 3),
#     AHP.Edge(2, 3, 2)
# ]

# for e in edge:
#     print(e.u, "<->", e.v, ", W:", e.w)

# path, w = AHP.AHP(edge, 4)
# print("Final result", path, "TW:", w)

# order_waypt = generateorder_waypt(pos_array, curr_pos, path)
# print(order_waypt)
