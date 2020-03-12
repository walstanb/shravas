from math import sqrt

class pt:
	def __init__(self,x,y):
		self.x=x
		self.y=y

class nofl:
	def __init__(self,x,y,r):
		self.x=x
		self.y=y
		self.r=r

def dist(s, d):
	q=sqrt(pow((d.x-s.x),2)+pow((d.y-s.y),2))
	return q

def dist(s, d):
	q=sqrt(pow((d.x-s.x),2)+pow((d.y-s.y),2))
	return q


def checkObs(s, d, o):
	a = d.y-s.y
	b = s.x-d.x
	c = a*s.y + b*s.x
	r=a*o.x+b*o.y
	if(abs(r-c)<0.0001):
		return 1
	else:
		return 0

def checkObs( s, d, x3, y3):
	a = d.y-s.y
	b = s.x-d.x
	c = a*s.y + b*s.x
	r=a*x3+b*y3
	if(abs(r-c)<0.0001):
		return 1
	else:
		return 0

def intersectsCir( s, d, o):
	a = d.y-s.y
	b = s.x-d.x
	c = (d.x*s.y - s.x*d.y)
	dq = sqrt(pow((a*o.x + b*o.y + c),2)/(a*a + b*b))
	if (dq <= o.r):
		return True
	else:
		return False

def getWayP(s, d, cir):
	flag=0
	m,n=0,0
	p=pt(0.0,0.0)
	wp=pt(0.0,0.0)
	s.x,s.y=s.x*10.0,s.y*10.0
	d.x,d.y=d.x*10.0,d.y*10.0
	cir.x,cir.y=cir.x*10.0,cir.y*10.0
	cir.r=cir.r*10.0

	if(intersectsCir(s,d,cir)==False):
		d.x,d.y=d.x/10.0,d.y/10.0
		return d
		
	for i in range(-142,55):
		for j in range(-50,95):
			#cout<<j<<","<<i<<"     ";
			
			#if(j==cir.x&&i==cir.y || j==cir.x&&i==cir.y+cir.r || j==cir.x&&i==cir.y-cir.r || j==cir.x-cir.r&&i==cir.y || j==cir.x+cir.r&&i==cir.y)
			#	cout<<"x ";
			
			if((j==s.x and i==s.y) or (j==d.x and i==d.y)):
				#cout<<"o ";
				continue
			elif(((j>=s.x and j<=d.x) and (i>=s.y and i<=d.y)) or ((j<=s.x and j>=d.x) and (i<=s.y and i>=d.y)) or ((j<=s.x and j>=d.x) and (i>=s.y and i<=d.y)) or ((j>=s.x and j<=d.x) and (i<=s.y and i>=d.y))):
				p= pt(j,i)
				if(intersectsCir(s,p,cir) or intersectsCir(d,p,cir)):
					#cout<<"  ";
					continue
				else:	
					#cout<<". ";
					#cout<<p.x<<", "<<p.y<<endl;
					t=dist(s,p)+dist(p,d)
					u=abs(1-(dist(s,p)/dist(p,d)))
					#cout<<p.x<<", "<<p.y<<"  "<<t<<" "<<u<<endl;
					
					if(flag==0):
						m,n=t,u
						wp=pt(p.x,p.y)
						flag=1
						#cout<<p.x<<", "<<p.y<<"  "<<m<<" "<<n<<endl;
					elif(t<m and u>n):
						m,n=t,u
						wp=pt(p.x,p.y)
						#cout<<p.x<<", "<<p.y<<"  "<<m<<" "<<n<<endl;
						flag=1
					else:
						continue		
			else:
				continue
				#cout<<"_ ";
		#cout<<endl;
	if(flag==1):
		wp.x,wp.y=wp.x/10.0,wp.y/10.0
		return wp
	else: 
		#cout<<"Switching to Stage 2"<<endl;
		for i in range(-142,55):
			for j in range(-50,95):
				if((j==s.x and i==s.y) or (j==d.x and i==d.y)):
					continue
				else:
					p=pt(j,i)
					if(intersectsCir(s,p,cir) or intersectsCir(d,p,cir)):
						continue
					else:
						t=dist(s,p)+dist(p,d)
						u=abs(1-(dist(s,p)/dist(p,d)))
						if(flag==0):
							m,n=t,u
							wp=pt(p.x,p.y)
							flag=1
						elif(t<m and u>n):
							m,n=t,u
							wp=pt(p.x,p.y)
							flag=1
						else:
							continue
		if(flag==1):
			wp.x,wp.y=wp.x/10.0,wp.y/10.0
			return wp
		else:
			#cout<<"No possible flight path"<<endl;
			print("No possible flight path")
			s.x,s.y=s.x/10.0,s.y/10.0
			return s

def main():
	s=pt(-12.0,8.8)
	d=pt(4.9,-4.0)
	cir=nofl(0.0,0.0,4.3)

	wp=getWayP(s, d, cir)

	print(wp.x,wp.y)

if __name__ == '__main__':
	main()