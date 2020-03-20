import csv

def csvread(fname):
	dictt={}
	i=1
	with open(fname, mode='r') as infile:
		reader = csv.DictReader(infile)
		for row in reader:
			dictt[i]=dict(row);
			i+=1
	ls=list(dictt.values())
	return ls
	
def csvwrite(ls,fname):
	#l=list(dictt.values())
	arr=[tuple(d.values()) for d in ls]
	k=[tuple(d.keys()) for d in ls]
	with open(fname,'wb') as out:
		csv_out=csv.writer(out)
		csv_out.writerow(k[0])
		for row in arr:
			csv_out.writerow(row)

#if __name__ == '__main__':
	#hq=csvread()
	#print(hq)
	#ls=[{"x":0,"y":0,"Z":0,"qr":0,"delivery":0},{"x":-8.0,"y":4.0,"Z":20,"qr":"QuadDrop","delivery":2},{"x":0.7,"y":-0.63,"Z":20,"qr":"WANK","delivery":1},{"x":0,"y":0,"Z":0,"qr":0,"delivery":-1}]
	#csvwrite(ls)