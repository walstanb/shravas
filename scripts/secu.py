import csvio
import getpass

def encrypt(msg,index):
	return msg

'''
Logic : Returns 1 if authenticated , -1 if password is incorrect and 0 for invalid user name
'''

def auth(uname,password):
	list = csvio.csvread('/home/'+getpass.getuser()+'/catkin_ws/src/shravas/src/dat.csv')
	for index in list :
		if(index['uname'] == uname and index['password'] == password):
			return 1
		else:
			return -1


#auth('Walstan','password')
