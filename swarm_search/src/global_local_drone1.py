import rospy
import math
import geopy
from geometry_msgs.msg import PoseStamped, Pose2D
from geopy.distance import VincentyDistance, vincenty
from sensor_msgs.msg import NavSatFix

# global local_target
local_target = Pose2D()
position_global = NavSatFix()
pose = PoseStamped()


def global_to_local(data):
	# TODO: refactor
	global local_target
	# local_target = Pose2D()
	lat = data.x
	lon = data.y
	print data

	d = math.hypot(pose.pose.position.x, pose.pose.position.y)

	bearing = math.degrees(math.atan2(-pose.pose.position.x, -pose.pose.position.y))
	if bearing < 0:
		bearing += 360

	cur = geopy.Point(position_global.latitude, position_global.longitude)
	origin = VincentyDistance(meters=d).destination(cur, bearing)

	_origin = origin.latitude, origin.longitude
	olat_tlon = origin.latitude, lon
	tlat_olon = lat, origin.longitude

	# global N
	N = vincenty(_origin, tlat_olon)
	if lat < origin.latitude:
		N = -N

	# global E
	E = vincenty(_origin, olat_tlon)
	if lon < origin.longitude:
		E = -E

	local_target.x = E.meters # DOUBT IN COORDINATE SYSTEM
	local_target.y = N.meters
	# print E.meters #x
	# print N.meters #y
	# print local_target

def globalCallback(data):
	global position_global
	position_global = data
	#print "global me data aaya"

def localCallback(data):
	global pose
	pose = data
	#print "local me data aaya"

def main():

	# global pub1
	# global local_target

	rospy.init_node('global_to_local_drone1', anonymous=True)
	start_time = rospy.get_time()

	rospy.Subscriber('/drone1/convert_global_to_local', Pose2D, global_to_local)
	rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, globalCallback ) 
	rospy.Subscriber('/drone1/mavros/local_position/pose', PoseStamped, localCallback )


	# global_to_local(data)
	pub1 = rospy.Publisher('/drone1/global_to_local_converted', Pose2D, queue_size=10)


	r = rospy.Rate(20.0)

	while not rospy.is_shutdown():
		# print local_target
		pub1.publish(local_target)
		r.sleep()
		# rospy.spin()

if __name__ == '__main__':
	main()

