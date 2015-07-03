"""
A basic GPS NMEA reader for UART on micropython.

Based on the servicepunkten.com FGPMMOPA6H breakout board but should probably work on others as well as they all follow the same standards.
http://www.servicepunkten.com/?p=62

Code by xix xeaon @ XIXIT.

See usage at the bottom.
"""


def nmea_checksum(d):
	c = 0
	for x in d:
		c = c ^ x
	return c

def verify_data(data):
	if not (
		data.startswith(b"$") and
		data[-5:].startswith(b"*") and
		data.endswith(b"\r\n")
	):
		return None
	
	cs = int(data[-4:-2], 16)
	msg = data[1:-5]
	if "$" in msg or "*" in msg:
		return False
	if not cs == nmea_checksum(msg):
		return False
	
	return msg.split(b',')

def deg2dec(d):
	return int(d[:-7]) + float(d[-7:].decode())/60


FXT_NONE   = 0
FXT_NORMAL = 1
FXT_DIFFER = 2
FXT_dict = {0:"NONE", 1:"NORMAL", 2:"DIFFER"}

FXD_NONE = 1
FXD_2D   = 2
FXD_3D   = 3

FXD_dict = {1:"NONE", 2:"2D", 3:"3D"}

ANT_ACTIVE_SHORT = 1
ANT_INTERNAL     = 2
ANT_ACTIVE       = 3
ANT_dict = {1:"ACTIVE_SHORT", 2:"INTERNAL", 3:"ACTIVE"}

class NMEAGPS():
	def __init__(self, uart):
		self.uart = uart
		
		self.date = None
		self.time = None
		
		self.fix_type = None
		self.fix_dim = None
		self.satellites = None
		self.antenna = None
		
		self.latitude = None
		self.longitude = None
		self.altitude = None
		
		self.pdop = None
		self.hdop = None
		self.vdop = None
		
		self.heading = None
		self.speed = None
	
	def read_sentences(self):
		while self.uart.any():
			data = uart.readline()
			msg = verify_data(data)
			if not msg: continue
			
			if msg[0] == b'GPGGA': # Global Positioning System Fixed Data
				self.handle_GPGGA(msg)
			elif msg[0] == b'GPGSA': # GNSS DOP and Active Satellites
				self.handle_GPGSA(msg)
			elif msg[0] == b'GPGSV': # GNSS Satellites in View
				pass
			elif msg[0] == b'GPRMC': # Recommended Minimum Navigation Information
				self.handle_GPRMC(msg)
			elif msg[0] == b'GPVTG': # Course and speed information relative to the ground
				self.handle_GPVTG(msg)
			elif msg[0] == b'PGTOP': # Status of antenna
				self.handle_PGTOP(msg)
			#else:
				#print(msg)
	
	def handle_GPGGA(self, msg):
		if msg[1]:
			self.time = (int(msg[1][:2]), int(msg[1][2:4]), float(msg[1][4:].decode()))
		else: self.time = None
		
		fix = int(msg[6])
		if fix in FXT_dict:
			self.fix_type = fix
		else:
			self.fix_type = None
			print("debug: unknown fix_type: %s" % fix) # seen 6?
		
		self.satellites = int(msg[7])
		
		if msg[2]:
			self.latitude = deg2dec(msg[2]) * (1 if msg[3] == b'N' else -1)
		else: self.latitude = None
		
		if msg[4]:
			self.longitude = deg2dec(msg[4]) * (1 if msg[5] == b'E' else -1)
		else: self.longitude = None
		
		if msg[9]:
			self.altitude = float(msg[9].decode())
			if msg[11]:
				self.altitude += float(msg[11].decode())
		else: self.altitude = None
		
		if msg[8]:
			self.hdop = float(msg[8].decode())
		else: self.hdop = None
	
	def handle_GPGSA(self, msg):
		fix = int(msg[2])
		if fix in FXD_dict:
			self.fix_dim = fix
		else:
			self.fix_dim = None
			print("debug: unknown fix_dim: %s" % fix)
		
		if msg[15]:
			self.pdop = float(msg[15].decode())
		else: self.pdop = None
		
		if msg[16]:
			self.hdop = float(msg[16].decode())
		else: self.hdop = None
		
		if msg[17]:
			self.vdop = float(msg[17].decode())
		else: self.vdop = None
	
	def handle_PGTOP(self, msg):
		ant = int(msg[2])
		if ant in ANT_dict:
			self.antenna = ant
		else:
			self.antenna = None
			print("debug: unknown antenna: %s" % ant)
	
	def handle_GPRMC(self, msg):
		if msg[4]:
			self.date = (int(msg[4][:2]), int(msg[4][2:4]), int(msg[4][4:]))
		else: self.date = None
	
	def handle_GPVTG(self, msg):
		if msg[1]:
			self.heading = float(msg[1].decode())
		else: self.heading = None
		
		if msg[7]:
			self.speed = float(msg[7].decode()) /60/60 *1000
		else: self.speed = None



if __name__ == "__main__":
	import pyb
	
	uart = pyb.UART(3, baudrate=9600, bits=8, stop=1, parity=None)
	
	gps = NMEAGPS(uart)
	
	while True:
		gps.read_sentences()
		
		print("%sT%s | %s, %s, %sm | %s, %s, %s | %s*, %sm/s | type:%s, dim:%s, sats:%s, ant:%s" % (
			gps.date, gps.time,
			gps.latitude, gps.longitude, gps.altitude,
			gps.pdop, gps.hdop, gps.vdop,
			gps.heading, gps.speed,
			FXT_dict.get(gps.fix_type), FXD_dict.get(gps.fix_dim), gps.satellites, ANT_dict.get(gps.antenna),
		))
		
		pyb.delay(100)


