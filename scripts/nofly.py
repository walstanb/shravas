import sys
from math import sqrt
import csvio
'''imported csv to read and write to csv file'''
#from csvio import csvread,csvwrite

class pt:
	'''
	Function Name:__init__
	Input:        x, y coordinates
	Output:       initiates all the variables in the class with coordinate data 
	Logic:        basic data structure for coordinate data
	'''
	def __init__(self,x,y):
		self.x=x
		self.y=y

class nofl:
	'''
	Function Name:	__init__
	Input:        	x, y, radius coordinates
	Output:       	initiates all the variables in the class with no fly zone center coordinates and radius
	Logic:        	basic data structure for no fly zone area
	'''
	def __init__(self,x,y,r):
		self.x=x
		self.y=y
		self.r=r

def dist(s, d):
	'''
	Function Name:	dist
	Input:        	source and destination coordinates
	Output:       	returns the distance two points (source and destination)
	Logic:        	calculates coordinate distance between two points
	'''
	q=sqrt(pow((d.x-s.x),2)+pow((d.y-s.y),2))
	return q

def intersectsCir( s, d, o):
	'''
	Function Name:	intersectsCir
	Input:        	source, destination and NoFlyZone
	Output:       	boolean True or False
	Logic:        	Checks whether the direct path between two points intersects NoFlyZone Circle
	'''
	a = d.y-s.y
	b = s.x-d.x
	c = (d.x*s.y - s.x*d.y)
	dq = sqrt(pow((a*o.x + b*o.y + c),2)/(a*a + b*b))
	if (dq <= o.r):
		return True
	else:
		return False

def getWayP(s, d, cir):
	'''
	Function Name:	getWayP
	Input:        	source, destination and NoFlyZone Circle
	Output:       	Single WayPoint
	Logic:        	Calculates optimal WayPoint for any two source destination and circular nofly region
	Notes:			Returns Source coordinates if no path is possible
					Returns Destination coordinates if direct path possible
					Is Divided into two Stages for code optimization
	'''
	flag=0
	m,n=0,0
	p=pt(0.0,0.0)
	wp=pt(0.0,0.0)
	#s.x,s.y=s.x*10.0,s.y*10.0
	#d.x,d.y=d.x*10.0,d.y*10.0
	#cir.x,cir.y=cir.x*10.0,cir.y*10.0
	#cir.r=cir.r*1.3

	if(intersectsCir(s,d,cir)==False):
		return d
	else:

		if(dist(s,pt(cir.x,cir.y))<=cir.r or dist(s,pt(cir.x,cir.y))<=cir.r):
			return s

		#Math Bitch 
		import numpy as np
		from math import acos, atan2, sin, cos

		b = sqrt((s.x - cir.x)**2 + (s.y - cir.y)**2)  	# hypot() also works here
		th = acos(cir.r / b)  								# angle theta
		g = atan2(s.y - cir.y, s.x - cir.x)  			# direction angle of point P from C
		g1 = g + th  									# direction angle of point T1 from C
		g2 = g - th  									# direction angle of point T2 from C

		s1x = cir.x + cir.r* cos(g1)
		s1y = cir.y + cir.r* sin(g1)
		s2x = cir.x + cir.r* cos(g2)
		s2y = cir.y + cir.r* sin(g2)

		#print(s1x,s1y,s2x,s2y)

		b = sqrt((d.x - cir.x)**2 + (d.y - cir.y)**2)  	# hypot() also works here
		th = acos(cir.r / b)  								# angle theta
		g = atan2(d.y - cir.y, d.x - cir.x)  			# direction angle of point P from C
		g1 = g + th  									# direction angle of point T1 from C
		g2 = g - th  									# direction angle of point T2 from C

		d1x = cir.x + cir.r* cos(g1)
		d1y = cir.y + cir.r* sin(g1)
		d2x = cir.x + cir.r* cos(g2)
		d2y = cir.y + cir.r* sin(g2)

		#print(d1x,d1y,d2x,d2y)

		ms1=(s.y-s1y)/float(s.x-s1x)
		ms2=(s.y-s2y)/float(s.x-s2x)
		md1=(d.y-d1y)/float(d.x-d1x)
		md2=(d.y-d2y)/float(d.x-d2x)

		cs1=s.y-ms1*s.x 								#y-ms1*x=s.y-ms1*s.x
		cs2=s.y-ms2*s.x 								#y-ms2*x=s.y-ms2*s.x
		cd1=d.y-md1*d.x 								#y-md1*x=d.y-md1*d.x
		cd2=d.y-md2*d.x 								#y-md2*x=d.y-md2*d.x

		#print(ms1,cs1)
		#print(ms2,cs2)
		#print(md1,cd1)
		#print(md2,cd2)


		j=np.array([[1.0,-ms1],[1.0,-md1]])
		k=np.array([cs1,cd1])
		l=np.linalg.solve(j,k)
		tm0=pt(l[1],l[0])

		j=np.array([[1.0,-ms1],[1.0,-md2]])
		k=np.array([cs1,cd2])
		l=np.linalg.solve(j,k)
		tm1=pt(l[1],l[0])

		j=np.array([[1.0,-ms2],[1.0,-md1]])
		k=np.array([cs2,cd1])
		l=np.linalg.solve(j,k)
		tm2=pt(l[1],l[0])

		j=np.array([[1.0,-ms2],[1.0,-md2]])
		k=np.array([cs2,cd2])
		l=np.linalg.solve(j,k)
		tm3=pt(l[1],l[0])

		ll=[tm0,tm1,tm2,tm3]

		#for i in range(len(ll)):
		#	print(ll[i].x,ll[i].y)

		ar=[(dist(s,ll[0])+dist(ll[0],d)),(dist(s,ll[1])+dist(ll[1],d)),(dist(s,ll[2])+dist(ll[2],d)),(dist(s,ll[3])+dist(ll[3],d))]
		indx=np.argmin(ar)

		if(-100.0<ll[indx].x<100.0 or -100.0<ll[indx].y<100.0):
			return ll[indx]
		else:
			return s


def main(wp):
	'''
	Function Name:	main
	Input:        	Reads coordinate data from input csv file
	Output:       	Writes coordinate data to outpur csv file
	Logic:        	Code to structure output csv file 
	'''
	rwp=[]
	
	#NO FLY ZONE
	cir=nofl(0.0,0.9,0.3)

	rwp.append({'y':float(wp[0]['y']),'x':float(wp[0]['x']),'delivery':int(wp[0]['delivery']),'z':float(wp[0]['z']),'qr':wp[0]['qr']})
	for i in range(1,len(wp)):
		s=pt(float(wp[i-1]['x']),float(wp[i-1]['y']))
		d=pt(float(wp[i]['x']),float(wp[i]['y']))
		co=getWayP(s, d, cir)
		#print("Source :",s.x,s.y)
		#print("Dest :",d.x,d.y)
		#print("WayP :",co.x,co.y)
		if(co.x==s.x and co.y==s.y):
			print("path plannning not possible")
			sys.exit(1)
		elif (co.x==d.x and co.y==d.y):
			rwp.append({'y':d.y,'x':d.x,'delivery':int(wp[i]['delivery']),'z':float(wp[i]['z']),'qr':wp[i]['qr']})
		elif((co.x!=s.x and co.y!=s.y) or (co.x!=d.x and co.y!=d.y)):
			#print("inserting")
			rwp.append({'y':co.y,'x':co.x,'delivery':-2,'z':float(wp[i]['z']),'qr':''})
			rwp.append({'y':d.y,'x':d.x,'delivery':int(wp[i]['delivery']),'z':float(wp[i]['z']),'qr':wp[i]['qr']})
			
	return rwp
	#csvwrite(rwp,"outputcsv.csv")
	#print(wp.x,wp.y)

if __name__ == '__main__':
	'''
	Function Name:	__main__
	Logic:        	For testing purposes only 
	'''
	#wp=[{"x":-8.52,"y":0.14,"z":0,"qr":0,"delivery":0},{"x":-4.4,"y":2.94,"z":20,"qr":"QuadDrop","delivery":2},{"x":0.7,"y":-0.63,"z":20,"qr":"WANK","delivery":1},{"x":0,"y":0,"z":0,"qr":0,"delivery":-1}]
	#cir=nofl(-6.7,1.5,2.0)
	wp=csvio.csvread('/home/walst/catkin_ws/src/shravas/src/coordinates.csv')
	rs=main(wp)

	for i in range(len(wp)):
		print(wp[i])
	print("\n\n")

	for i in range(len(rs)):
		print(rs[i])
	#print(rs[1]['x'],rs[1]['y'])