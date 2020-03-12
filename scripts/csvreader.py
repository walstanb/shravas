import csv

class csvio():
	def __init__(self):
		self.wp={}
	def read(self):
		i=1
		with open('coordinates.csv', mode='r') as infile:
			reader = csv.DictReader(infile)
			for row in reader:
				self.wp[i]=dict(row);
				i=i+1
		
	def csv_write(self):
		
		self.wp{{x:33,y:44},{x:56,y:78},{x:44,y:77}}
		
		with open('coords.csv', 'w') as csv_file:  
			writer = csv.writer(csv_file)
			for data in self.wp.items():
				writer.writerow(data)
		print(self.wp)

if __name__ == '__main__':
	test = csvio()
	test.read()
	test.csv_write()