import serial
import time

port_name1 = "/dev/cu.usbserial-595E5A8E5C"
#port_name2 = "/dev/cu.wchusbserial5322029451"
port_name2 = "/dev/ttyACM0"


class myAGVBase(object):

	time_out = 1 #sec
	baud_rate = 115200
	serial_time_out = 0.01
	header  = 115

	data_len = 16

	xyw_max = 1
	xyw_min = -1

	data_list = []

	error_2 = "Rec data len wrong"
	error_3 = "Rec data verifcation wrong"
	error_4 = "Rec data too long time"

	def __init__(self, COM):
		self.port_name = COM
		self.ser = serial.Serial(self.port_name, self.baud_rate, timeout = self.time_out)
		time.sleep(self.serial_time_out)

	def processData(self, _data_list):
		if len(_data_list) == self.data_len:
			ver = 0
			for i in range(self.data_len-1):
				ver += _data_list[i]

			if (ver%256) == _data_list[self.data_len-1] :
				_x = (_data_list[0] - 128) / 100
				_y = (_data_list[1] - 128) / 100
				_w = (_data_list[2] - 128) / 100

				ax = ((_data_list[3] + _data_list[4]*256 ) - 10000) / 1000.0
				ay = ((_data_list[5] + _data_list[6]*256 ) - 10000) / 1000.0
				az = ((_data_list[7] + _data_list[8]*256 ) - 10000) / 1000.0

				wx = ((_data_list[9] + _data_list[10]*256 ) - 10000) / 10.0
				wy = ((_data_list[11] + _data_list[12]*256 ) - 10000) / 10.0
				wz = ((_data_list[13] + _data_list[14]*256 ) - 10000) / 10.0

				return ([_x, _y , _w],[ax,ay,az,wx,wy,wz])
			else:
				pass
		else:
			pass
		return 0


	def getXYW(self):
		entry_time = time.time()

		while 1:
			rec = self.ser.read()
			data_1 = int.from_bytes(rec,'big')

			if data_1 == self.header:
				data_2 = int.from_bytes(self.ser.read(),'big')
				if data_2 == self.header:
					for i in range(self.data_len):
						self.data_list.append(int.from_bytes(self.ser.read(),'big'))
					res = self.processData(self.data_list)
					self.data_list = []
					return res
					break
			else:
				return (0)
				break

			if (time.time() - entry_time ) >self.time_out:
				print (self.error_4)
				return (-1)
				break

		pass

	def sendXYW(self,_x,_y,_w):		# units: m/s  m/s  rad/s ; max 1m/s 1rad/s
		xyw = [_x,_y,_w]
		for i in range(3):
			if xyw[i] > self.xyw_max:
				xyw[i] = self.xyw_max
			if xyw[i] < self.xyw_min:
				xyw[i] = self.xyw_min

		x_send = int(xyw[0]*100) + 128
		y_send = int(xyw[1]*100) + 128
		w_send = int(xyw[2]*100) + 128
		ver = x_send + y_send + w_send

		if ver>255:
			ver -= 256

		command = bytearray()
		command.append(self.header)
		command.append(self.header)
		command.append(x_send)
		command.append(y_send)
		command.append(w_send)
		command.append(ver)

		self.ser.write(command)

		time.sleep(self.serial_time_out)



base = myAGVBase(port_name2)

base.sendXYW(0.0,0,0)	# units: m/s  m/s  rad/s ; max 1m/s 1rad/s

while 1:
	# get xyw data
	_data =base.getXYW() 
	if _data != 0:
		print(_data)  
	#base.getXYW()






