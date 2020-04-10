import csvio
import getpass

def encrypt(msg):
	result=""
	count = -1
	a,r = 17,7
	for index in msg:
		count+=1
		temp = ord(index)
		update = temp*(a*(r**count))
		update = update % 26
		update += 97
		temp = chr(update)
		result+=temp
	return result

def auth(uname,password):
	password = encrypt(password)
	list = csvio.csvread('/home/'+getpass.getuser()+'/catkin_ws/src/shravas/src/dat.csv')
	for index in list :
		if(index['uname'] == uname and index['password'] == password):
			return 1
		else:
			return -1



#auth('Walstan','password')
