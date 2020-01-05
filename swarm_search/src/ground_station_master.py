
from math import sin, cos, sqrt, atan2, radians
import copy 
import matplotlib.pyplot as plt 

R1 = 6373000
sip_const = 12.5



def gps_corner_sort(input_square,drone_start):
	"""
	Code returns a list containing the sorted gps coordinates from drone 0
	Input the 4 gps vertices and the gps coordinates of drone 0
	"""
	sorted_gps = []
	new_list = []

	for i in range (0,len(input_square)):

		sorted_gps.append(gps_dist(drone_start[0],drone_start[1],input_square[i][0],input_square[i][1]))

	sorted_gps_m = copy.deepcopy(sorted_gps)  

	while sorted_gps_m:
		minimum = sorted_gps_m[0]
		j = 0
		for i in range (0,len(sorted_gps_m)):
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
	x = [0,0]
	y = [0,0]
	dist1 = gps_dist(isl[0][0],isl[0][1],isl[1][0],isl[1][1])
	dist2 = gps_dist(isl[0][0],isl[0][1],isl[2][0],isl[2][1])

	origin = isl[0]

	x[0] = (isl[1][0]-isl[0][0])/dist1
	x[1] = (isl[1][1]-isl[0][1])/dist1

	y[0] = (isl[2][0]-isl[0][0])/dist2
	y[1] = (isl[2][1]-isl[0][1])/dist2
	
	return(origin,x,y,dist1)

def transform_to_gps(point,origin,x,y):
	"""
	Transforms the input point from the local frame to gps coordinates
	"""
	tr_point = [0,0]

	tr_point[0] = origin[0] + point[0]*x[0] + point[1]*y[0]
	tr_point[1] = origin[1] + point[0]*x[1] + point[1]*y[1]

	return(tr_point)


def gps_dist(lat1,lon1,lat2,lon2):
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

def euclidean_dist(x1,y1,x2,y2):
	"""
	returns the euclidean distance between 2 points
	"""
	return (sqrt(pow((x1-x2),2) + pow(y1-y2,2)))

def find_SIP(x1,y1,x2,y2,i):
	"""
	Function called from quadrant_SIP function
	"""
	x = (x1+x2)/2.0
	y = (y1+y2)/2.0
	if(i==0):
		x+=sip_const
	if(i==1):
		y-=sip_const
	if(i==2):
		x-=sip_const
	if(i==-1):
		y+=sip_const

	return(x,y)



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
	for i in range (4):
		sip.append([0,0])

	for i in range (-1,len(sq_c)-1):
		sip[i+1] = find_SIP(sq_c[i][0],sq_c[i][1],sq_c[i+1][0],sq_c[i+1][1],i)

	return (sip)

def find_quad_corners(quad_origin,d):
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

		if(i==1 or i==2):
			sub_quad_corner[i][1] = sub_quad_corner[i][1]+d/2.0

		if(i==3 or i==2):
			sub_quad_corner[i][0] = sub_quad_corner[i][0]+d/2.0

	return(sub_quad_corner)

def plot(in_array,in_SIP,drone_pos):
	"""
	Function to plot the SIP using matplotlib
	"""
	x1 = []
	y1 = []

	for i in range(0,4):
	 	for j in range(0,4):
	 		x1.append(in_array[i][j][0])
	 		y1.append(in_array[i][j][1])

	plt.scatter(x1,y1,color = (0.0,1.0,0.0))
	j = 4
	for i in range(0,len(in_SIP)):
		plt.scatter(in_SIP[i][0][0],in_SIP[i][0][1],color = (i/4.0,0.0,j/4.0))
		plt.scatter(in_SIP[i][1][0],in_SIP[i][1][1],color = (i/4.0,0.0,j/4.0))
		plt.scatter(drone_pos[i][0],drone_pos[i][1],color = (i/4.0,0.0,j/4.0))
		j=j-1
	plt.show() 

def compute_gps_sip(input_square,drone_start):
	"""
	This is the function which returns the GPS_SIPs when input is 4 GPS vertices of 
	the arena and the location of drone 0
	"""
	local_origin = [0,0]
	sorted_input_square = gps_corner_sort(input_square,drone_start)
	origin,x,y,dist = coordinate_axes(sorted_input_square)

	quad_corner = []
	sip = []
	quad_corner.append(find_quad_corners(local_origin,dist))
	
	for i in range(1,4):
		quad_corner.append(find_quad_corners(quad_corner[0][i],dist))

	for i in range(0,len(quad_corner)):	
		sip.append(quadrant_SIP(quad_corner[i]))

	gps_sip = copy.deepcopy(sip)

	for i in range(0,4):
	 	for j in range(0,4):
	 		gps_sip[i][j] = transform_to_gps(sip[i][j],origin,x,y)

	return(gps_sip)


def find_start_end_SIP(quad_sip,drone_pos):

	sip_dist = []
	new_list = []

	for i in range(0,len(quad_sip)):
		sip_dist.append(gps_dist(drone_pos[0],drone_pos[1],quad_sip[i][0],quad_sip[i][1]))

	sorted_dist_m = copy.deepcopy(sip_dist)  

	while sorted_dist_m:
		minimum = sorted_dist_m[0]
		j = 0
		for i in range (0,len(sorted_dist_m)):
			if sorted_dist_m[i] < minimum:
				minimum = sorted_dist_m[i]
		
		j = sip_dist.index(minimum)
		new_list.append(quad_sip[j])
		sorted_dist_m.remove(minimum)  

	return([new_list[0],new_list[3]])


def main():

	target_sip = []

	input_square = [[-35.362588,149.165051],[-35.362588,149.166153],[-35.363485,149.166155],[-35.363485,149.165051]]


	drone_input = [[-35.362383,149.164811],[-35.362424,149.164817],[-35.362459,149.164819],[-35.362498,149.164825]]
	drone_start = drone_input[0]

	gps_sip = compute_gps_sip(input_square,drone_start)

	for i in range (0,len(drone_input)):
		target_sip.append(find_start_end_SIP(gps_sip[i],drone_input[i]))

	plot(gps_sip,target_sip,drone_input)
	
	
if __name__ == '__main__':
	main()