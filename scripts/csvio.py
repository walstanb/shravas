import csv

def csvread():
	dictt={}
	i=1
	with open('/home/walst/catkin_ws/src/shravas/scripts/coordinates.csv', mode='r') as infile:
		reader = csv.DictReader(infile)
		for row in reader:
			dictt[i]=dict(row);
			i+=1
	ls=list(dictt.values())
	return ls
	
def csvwrite(ls):
	#l=list(dictt.values())
	arr=[tuple(d.values()) for d in ls]
	k=[tuple(d.keys()) for d in ls]
	with open('/home/walst/catkin_ws/src/shravas/scripts/coords.csv','wb') as out:
		csv_out=csv.writer(out)
		csv_out.writerow(k[0])
		for row in arr:
			csv_out.writerow(row)

if __name__ == '__main__':
	hq=csvread()
	print(hq)
	csvwrite(hq)