import rospy
import math
import geopy
from geometry_msgs.msg import PoseStamped, Pose2D
from geopy.distance import VincentyDistance, vincenty
from sensor_msgs.msg import NavSatFix

global local_target
local_target = Pose2D()

def global_to_local(data):
    # TODO: refactor

	lat = data.x
	lon = data.y

	position_global = rospy.wait_for_message('/drone1/mavros/global_position/global', NavSatFix, timeout=5)
	pose = rospy.wait_for_message('/drone1/mavros/local_position/pose', PoseStamped, timeout=5)

	d = math.hypot(pose.pose.position.x, pose.pose.position.y)

	bearing = math.degrees(math.atan2(-pose.pose.position.x, -pose.pose.position.y))
	if bearing < 0:
		bearing += 360

	cur = geopy.Point(position_global.latitude, position_global.longitude)
	origin = VincentyDistance(meters=d).destination(cur, bearing)

	_origin = origin.latitude, origin.longitude
	olat_tlon = origin.latitude, lon
	tlat_olon = lat, origin.longitude

	global N
	N = vincenty(_origin, tlat_olon)
	if lat < origin.latitude:
		N = -N

	global E
	E = vincenty(_origin, olat_tlon)
	if lon < origin.longitude:
		E = -E

	local_target.x = E.meters # DOUBT IN COORDINATE SYSTEM
	local_target.y = N.meters


def main():

    global pub1

    rospy.init_node('global_to_local_drone1', anonymous=True)
    start_time = rospy.get_time()

    rospy.Subscriber('/drone1/convert_global_to_local', Pose2D, global_to_local)
    pub1 = rospy.Publisher('/drone1/global_to_local_converted', Pose2D, queue_size=10)
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
		pub1.publish(local_target)
		r.sleep()
		rospy.spin()

if __name__ == '__main__':
    main()

