

from math import sin, cos, sqrt, atan2, radians
import copy 
import matplotlib.pyplot as plt 

R = 6373000
sip_const = 1.25

input_square = [[22.317494, 87.309866],[22.317608,87.310572],[22.318193, 87.310467],[22.318106, 87.309797]]

drone_start = [22.317406, 87.310183]


def gps_dist(lat1,lon1,lat2,lon2):

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

	return (sqrt(pow((x1-x2),2) + pow(y1-y2,2)))

def find_SIP(x1,y1,x2,y2,i):

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

	sip = []
	for i in range (4):
		sip.append([0,0])

	for i in range (-1,len(sq_c)-1):
		sip[i+1] = find_SIP(sq_c[i][0],sq_c[i][1],sq_c[i+1][0],sq_c[i+1][1],i)
	

	return (sip)

def find_quad_corners(quad_origin,d):

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

def main():

	corner = [[0,0],[0,10],[10,10],[10,0]]
	dist = 10
	quad_corner = []
	x = []
	y = []
	sip = []
	quad_corner.append(find_quad_corners(corner[0],dist))
	
	for i in range(1,len(corner)):
		quad_corner.append(find_quad_corners(quad_corner[0][i],dist))

	for i in range(0,len(quad_corner)):	
		sip.append(quadrant_SIP(quad_corner[i]))
		print (sip[i])
	  
	# Function to plot scatter 
	for i in range(0,4):
	 	for j in range(0,4):
	 		x.append(sip[i][j][0])
	 		y.append(sip[i][j][1])

	plt.scatter(x,y) 
	plt.ylim(0, 10)
	plt.xlim(0, 10)
	  
	# function to show the plot 
	plt.show() 

if __name__ == '__main__':
	main()