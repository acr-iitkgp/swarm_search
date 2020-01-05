
from math import sin, cos, sqrt, atan2, radians
import copy
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from swarm_search.msg import sip_goal
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix


R1 = 6373000
sip_const = 2.5  # depends on the height and FOV of the camera

input_square = [[22.314583, 87.308080], [22.314583, 87.308276], [22.314765, 87.308278], [22.314764, 87.308082]]
global drone_input
global drone_start
drone_input = [[22.314577, 87.308265], [22.314579, 87.308355], [22.314577, 87.308243], [22.314577, 87.308205]]

drone_start = drone_input[0]

global connected0
global armed0
global state0

global connected1
global armed1
global state1

global connected2
global armed2
global state2

global connected3
global armed3
global state3

global start_time

global i
i = 0

global j
j = 0

global target_0
global target_1
global target_2
global target_3

target_0 = sip_goal()
target_1 = sip_goal()
target_2 = sip_goal()
target_3 = sip_goal()


###############################
connected0 = 1
connected1 = 0
connected2 = 1
connected3 = 1

armed0 = 1
armed1 = 0
armed2 = 1
armed3 = 1
###############################


def gps_corner_sort(input_square, drone_start):
    """
    Code returns a list containing the sorted gps coordinates from drone 0
    Input the 4 gps vertices and the gps coordinates of drone 0
    """
    sorted_gps = []
    new_list = []

    for i in range(0, len(input_square)):

        sorted_gps.append(gps_dist(drone_start[0], drone_start[1], input_square[i][0], input_square[i][1]))

    sorted_gps_m = copy.deepcopy(sorted_gps)

    while sorted_gps_m:
        minimum = sorted_gps_m[0]
        j = 0
        for i in range(0, len(sorted_gps_m)):
            if sorted_gps_m[i] < minimum:
                minimum = sorted_gps_m[i]

        j = sorted_gps.index(minimum)
        new_list.append(input_square[j])
        sorted_gps_m.remove(minimum)

    return(new_list)


def coordinate_axes(isl):
    """
    returns the origin, x axes, y axes and vertice distance of the local coordinate system
    Input is sorted list of gps vertices
    """
    x = [0, 0]
    y = [0, 0]
    dist1 = gps_dist(isl[0][0], isl[0][1], isl[1][0], isl[1][1])
    dist2 = gps_dist(isl[0][0], isl[0][1], isl[2][0], isl[2][1])

    origin = isl[0]

    x[0] = (isl[1][0] - isl[0][0]) / dist1
    x[1] = (isl[1][1] - isl[0][1]) / dist1

    y[0] = (isl[2][0] - isl[0][0]) / dist2
    y[1] = (isl[2][1] - isl[0][1]) / dist2

    return(origin, x, y, dist1)


def transform_to_gps(point, origin, x, y):
    """
    Transforms the input point from the local frame to gps coordinates
    """
    tr_point = [0, 0]

    tr_point[0] = origin[0] + point[0] * x[0] + point[1] * y[0]
    tr_point[1] = origin[1] + point[0] * x[1] + point[1] * y[1]

    return(tr_point)


def gps_dist(lat1, lon1, lat2, lon2):
    """
    Returns the distance in metres between 2 global points
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R1 * c
    return (distance)


def euclidean_dist(x1, y1, x2, y2):
    """
    returns the euclidean distance between 2 points
    """
    return (sqrt(pow((x1 - x2), 2) + pow(y1 - y2, 2)))


def find_SIP(x1, y1, x2, y2, i):
    """
    Function called from quadrant_SIP function
    """
    x = (x1 + x2) / 2.0
    y = (y1 + y2) / 2.0
    if(i == 0):
        x += sip_const
    if(i == 1):
        y -= sip_const
    if(i == 2):
        x -= sip_const
    if(i == -1):
        y += sip_const

    return(x, y)


def quadrant_SIP(sq_c):
    """
    Input list containing 4 corners of a sub quadrant
    Returns a list containg the 4 SIPs of that sub quadrant
    Sub-quadrant corners in format - 1X2
                                                                                                                                     XXX
                                                                                                                                     0X3
    SIP format - X2X
                                                     1X3
                                                     X0X
    """
    sip = []
    for i in range(4):
        sip.append([0, 0])

    for i in range(-1, len(sq_c) - 1):
        sip[i + 1] = find_SIP(sq_c[i][0], sq_c[i][1], sq_c[i + 1][0], sq_c[i + 1][1], i)

    return (sip)


def find_quad_corners(quad_origin, d):
    """
    Input - origin of the coordinate system (0,0) and the distance between 2 vertices
    """
    sub_quad_corner = []
    c_quad_origin = copy.deepcopy(quad_origin)

    for i in range(4):
        temp = []
        for j in range(len(c_quad_origin)):
            temp.append(c_quad_origin[j])
        sub_quad_corner.append(temp)

    for i in range(4):

        if(i == 1 or i == 2):
            sub_quad_corner[i][1] = sub_quad_corner[i][1] + d / 2.0

        if(i == 3 or i == 2):
            sub_quad_corner[i][0] = sub_quad_corner[i][0] + d / 2.0

    return(sub_quad_corner)


def plot(in_array, in_SIP, drone_pos):
    """
    Function to plot the SIP using matplotlib
    """
    x1 = []
    y1 = []

    for i in range(0, 4):
        for j in range(0, 4):
            x1.append(in_array[i][j][0])
            y1.append(in_array[i][j][1])

    plt.scatter(x1, y1, color=(0.0, 1.0, 0.0))
    j = 4
    for i in range(0, len(in_SIP)):
        plt.scatter(in_SIP[i][0][0], in_SIP[i][0][1], color=(i / 4.0, 0.0, j / 4.0))
        plt.scatter(in_SIP[i][1][0], in_SIP[i][1][1], color=(i / 4.0, 0.0, j / 4.0))
        plt.scatter(drone_pos[i][0], drone_pos[i][1], color=(i / 4.0, 0.0, j / 4.0))
        j = j - 1
    # plt.show()


def plot1(in_array, in_SIP, drone_pos):
    """
    Function to plot the SIP using matplotlib
    """
    x1 = []
    y1 = []
    x2 = []
    y2 = []

    plt.scatter(drone_start[0], drone_start[1], color=(0.0, 0.0, 0.0))

    for i in range(0, 4):
        for j in range(0, 4):
            x1.append(in_array[i][j][0])
            y1.append(in_array[i][j][1])

    plt.scatter(x1, y1, color=(0.0, 1.0, 0.0))

    for i in range(0, 4):
        x2.append(input_square[i][0])

    for i in range(0, 4):
        y2.append(input_square[i][1])

    plt.scatter(x2, y2, color=(1.0, 0.0, 0.0))
    plt.scatter(drone_start[0], drone_start[1], color=(0.0, 0.0, 0.0))

    j = 4
    for i in range(0, len(in_SIP)):
        plt.scatter(in_SIP[i][0][0], in_SIP[i][0][1], color=(i / 4.0, 0.0, j / 4.0))
        plt.scatter(in_SIP[i][1][0], in_SIP[i][1][1], color=(i / 4.0, 0.0, j / 4.0))
        plt.scatter(drone_pos[i][0], drone_pos[i][1], color=(i / 4.0, 0.0, j / 4.0))
        j = j - 1
    plt.show()


def compute_gps_sip(input_square, drone_start):
    """
    This is the function which returns the GPS_SIPs when input is 4 GPS vertices of
    the arena and the location of drone 0
    """
    local_origin = [0, 0]
    sorted_input_square = gps_corner_sort(input_square, drone_start)
    origin, x, y, dist = coordinate_axes(sorted_input_square)

    quad_corner = []
    sip = []
    quad_corner.append(find_quad_corners(local_origin, dist))

    for i in range(1, 4):
        quad_corner.append(find_quad_corners(quad_corner[0][i], dist))

    for i in range(0, len(quad_corner)):
        sip.append(quadrant_SIP(quad_corner[i]))

    gps_sip = copy.deepcopy(sip)

    for i in range(0, 4):
        for j in range(0, 4):
            gps_sip[i][j] = transform_to_gps(sip[i][j], origin, x, y)

    return(gps_sip)


def find_start_end_SIP(quad_sip, drone_pos):

    sip_dist = []
    new_list = []

    for i in range(0, len(quad_sip)):
        sip_dist.append(gps_dist(drone_pos[0], drone_pos[1], quad_sip[i][0], quad_sip[i][1]))

    sorted_dist_m = copy.deepcopy(sip_dist)
    # Use these if need to calculate original SIP
    # while sorted_dist_m:
    # 	minimum = sorted_dist_m[0]
    # 	j = 0
    # 	for i in range(0, len(sorted_dist_m)):
    # 		if sorted_dist_m[i] < minimum:
    # 			minimum = sorted_dist_m[i]

    # 	j = sip_dist.index(minimum)
    # 	new_list.append(quad_sip[j])
    # 	sorted_dist_m.remove(minimum)

    # return([new_list[0], new_list[3]])

    # Use this for fixed SIP in orientation

    return([quad_sip[0], quad_sip[2]])


def assign_sip(gps_sip, drone_input):

    target_sip = []

    for i in range(0, len(drone_input)):
        target_sip.append(find_start_end_SIP(gps_sip[i], drone_input[i]))

    plot(gps_sip, target_sip, drone_input)
    return(target_sip)


def assign_sip1(gps_sip, drone_input):

    target_sip = []

    for i in range(0, len(drone_input)):
        target_sip.append(find_start_end_SIP(gps_sip[i], drone_input[i]))

    plot1(gps_sip, target_sip, drone_input)
    return(target_sip)


def callback0(data):

    global drone_input
    global drone_start

    drone_input[0][0] = data.latitude
    drone_input[0][1] = data.longitude
    drone_start = drone_input[0]


def callback1(data):

    global drone_input
    print(data.latitude)
    drone_input[1][0] = data.latitude
    drone_input[1][1] = data.longitude


def callback2(data):

    global drone_input
    drone_input[2][0] = data.latitude
    drone_input[2][1] = data.longitude


def callback3(data):

    global drone_input
    drone_input[3][0] = data.latitude
    drone_input[3][1] = data.longitude


def execute():

    global start_time
    global i

    global target_0
    global target_1
    global target_2
    global target_3

    if (i == 0):
        start_time = rospy.get_time()
        i = 1

    # target_2.takeoff_flag.data = 1 ####### only for testing
    if((rospy.get_time() - start_time) > 5):
        target_1.takeoff_flag.data = 1

    if((rospy.get_time() - start_time) > 10):
        target_2.takeoff_flag.data = 1

    if((rospy.get_time() - start_time) > 15):
        target_3.takeoff_flag.data = 1

    if((rospy.get_time() - start_time) > 20):
        target_0.takeoff_flag.data = 1

    pub0.publish(target_0)
    pub1.publish(target_1)
    pub2.publish(target_2)
    pub3.publish(target_3)


def calculate_execute():

    global target_0
    global target_1
    global target_2
    global target_3

    target_0.takeoff_flag.data = 0
    target_1.takeoff_flag.data = 0
    target_2.takeoff_flag.data = 0
    target_3.takeoff_flag.data = 0

    gps_sip = compute_gps_sip(input_square, drone_start)
    target_sip = assign_sip1(gps_sip, drone_input)

    target_0.sip_start.x = target_sip[0][0][0]
    target_0.sip_start.y = target_sip[0][0][1]
    target_0.sip_end.x = target_sip[0][1][0]
    target_0.sip_end.y = target_sip[0][1][1]

    target_1.sip_start.x = target_sip[1][0][0]
    target_1.sip_start.y = target_sip[1][0][1]
    target_1.sip_end.x = target_sip[1][1][0]
    target_1.sip_end.y = target_sip[1][1][1]

    target_2.sip_start.x = target_sip[2][0][0]
    target_2.sip_start.y = target_sip[2][0][1]
    target_2.sip_end.x = target_sip[2][1][0]
    target_2.sip_end.y = target_sip[2][1][1]

    target_3.sip_start.x = target_sip[3][0][0]
    target_3.sip_start.y = target_sip[3][0][1]
    target_3.sip_end.x = target_sip[3][1][0]
    target_3.sip_end.y = target_sip[3][1][1]


def state0(data):
    global connected0
    global armed0

    connected0 = data.connected
    armed0 = data.armed


def state1(data):
    global connected1
    global armed1
    global j

    connected1 = data.connected
    armed1 = data.armed

    if(connected0 and connected1 and connected2 and connected3 and armed0 and armed1 and armed2 and armed3 and (j < 10)):
        print("Entered: " + str(j))
        calculate_execute()
        j += 1

    if(connected0 and connected1 and connected2 and connected3 and armed0 and armed1 and armed2 and armed3 and (j >= 10)):
        print("Staring to execute")
        execute()


def state2(data):
    global connected2
    global armed2

    connected2 = data.connected
    armed2 = data.armed


def state3(data):
    global connected3
    global armed3
    global start_mission

    connected3 = data.connected
    armed3 = data.armed


def main():

    global pub0
    global pub1
    global pub2
    global pub3

    global start_time

    rospy.init_node('initial_ground_publish', anonymous=True)
    start_time = rospy.get_time()

    rospy.Subscriber("/drone0/mavros/global_position/global", NavSatFix, callback0)
    rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, callback1)
    rospy.Subscriber("/drone2/mavros/global_position/global", NavSatFix, callback2)
    rospy.Subscriber("/drone3/mavros/global_position/global", NavSatFix, callback3)

    ##########################################
    # rospy.Subscriber("/drone2/mavros/global_position/global", NavSatFix, callback2)
    # rospy.Subscriber("/drone2/mavros/state", State, state2)
    ##########################################

    pub0 = rospy.Publisher('master/drone0/ground_msg', sip_goal, queue_size=5)
    pub1 = rospy.Publisher('master/drone1/ground_msg', sip_goal, queue_size=5)
    pub2 = rospy.Publisher('master/drone2/ground_msg', sip_goal, queue_size=5)
    pub3 = rospy.Publisher('master/drone3/ground_msg', sip_goal, queue_size=5)

    rospy.Subscriber("/drone0/mavros/state", State, state0)
    rospy.Subscriber("/drone1/mavros/state", State, state1)
    rospy.Subscriber("/drone2/mavros/state", State, state2)
    rospy.Subscriber("/drone3/mavros/state", State, state3)

    # calculate_execute()
    # execute()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
