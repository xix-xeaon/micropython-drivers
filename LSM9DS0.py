"""
Simple access to the g-force linear acceleration, gauss magnetic and dps angular rate sensors of the LSM9DS0 through I2C for micropython.

Values are normalized to g-force, gauss and degrees per second for the default sensitivities, but no options are possible yet.

Based on SparkFun 9 Degrees of Freedom IMU Breakout: LSM9DS0.
https://www.sparkfun.com/products/12636

Code by xix xeaon @ XIXIT.

Thanks to Jim Lindblom @ SparkFun Electronics and his SFE_LSM9DS0.cpp letting me know where to start

See usage at the bottom.
"""


# thanks travc @ stackoverflow
def twos_comp(val, bits=8):
	if (val & (1 << (bits - 1))) != 0:
		val = val - (1 << bits)
	return val


G = 0 # Angular rate sensor
WHO_AM_I_G = 0x0F
CTRL_REG1_G = 0x20
CTRL_REG2_G = 0x21
CTRL_REG3_G = 0x22
CTRL_REG4_G = 0x23
CTRL_REG5_G = 0x24
REFERENCE_G = 0x25
STATUS_REG_G = 0x27
OUT_X_L_G = 0x28
OUT_X_H_G = 0x29
OUT_Y_L_G = 0x2A
OUT_Y_H_G = 0x2B
OUT_Z_L_G = 0x2C
OUT_Z_H_G = 0x2D
FIFO_CTRL_REG_G = 0x2E
FIFO_SRC_REG_G = 0x2F
INT1_CFG_G = 0x30
INT1_SRC_G = 0x31
INT1_TSH_XH_G = 0x32
INT1_TSH_XL_G = 0x33
INT1_TSH_YH_G = 0x34
INT1_TSH_YL_G = 0x35
INT1_TSH_ZH_G = 0x36
INT1_TSH_ZL_G = 0x37
INT1_DURATION_G = 0x38


XM = 1 # Linear acceleration and magnetic sensor
OUT_TEMP_L_XM = 0x05
OUT_TEMP_H_XM = 0x06
STATUS_REG_M = 0x07
OUT_X_L_M = 0x08
OUT_X_H_M = 0x09
OUT_Y_L_M = 0x0A
OUT_Y_H_M = 0x0B
OUT_Z_L_M = 0x0C
OUT_Z_H_M = 0x0D
WHO_AM_I_XM = 0x0F
INT_CTRL_REG_M = 0x12
INT_SRC_REG_M = 0x13
INT_THS_L_M = 0x14
INT_THS_H_M = 0x15
OFFSET_X_L_M = 0x16
OFFSET_X_H_M = 0x17
OFFSET_Y_L_M = 0x18
OFFSET_Y_H_M = 0x19
OFFSET_Z_L_M = 0x1A
OFFSET_Z_H_M = 0x1B
REFERENCE_X = 0x1C
REFERENCE_Y = 0x1D
REFERENCE_Z = 0x1E
CTRL_REG0_XM = 0x1F
CTRL_REG1_XM = 0x20
CTRL_REG2_XM = 0x21
CTRL_REG3_XM = 0x22
CTRL_REG4_XM = 0x23
CTRL_REG5_XM = 0x24
CTRL_REG6_XM = 0x25
CTRL_REG7_XM = 0x26
STATUS_REG_A = 0x27
OUT_X_L_A = 0x28
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2A
OUT_Y_H_A = 0x2B
OUT_Z_L_A = 0x2C
OUT_Z_H_A = 0x2D
FIFO_CTRL_REG = 0x2E
FIFO_SRC_REG = 0x2F
INT_GEN_1_REG = 0x30
INT_GEN_1_SRC = 0x31
INT_GEN_1_THS = 0x32
INT_GEN_1_DURATION = 0x33
INT_GEN_2_REG = 0x34
INT_GEN_2_SRC = 0x35
INT_GEN_2_THS = 0x36
INT_GEN_2_DURATION = 0x37
CLICK_CFG = 0x38
CLICK_SRC = 0x39
CLICK_THS = 0x3A
TIME_LIMIT = 0x3B
TIME_LATENCY = 0x3C
TIME_WINDOW = 0x3D
Act_THS = 0x3E
Act_DUR = 0x3F


g_addr=0x6B
xm_addr=0x1D

class LSM9DS0():
	def __init__(self, i2c, g_addr=0x6B, xm_addr=0x1D):
		self.i2c = i2c
		self.g_addr = g_addr
		self.xm_addr = xm_addr
		
		self.gyro = LSM9DS0.SensorInterface(self, G, OUT_X_L_G, OUT_Y_L_G, OUT_Z_L_G, 8.75/1000)
		self.mag = LSM9DS0.SensorInterface(self, XM, OUT_X_L_M, OUT_Y_L_M, OUT_Z_L_M, 0.08/1000)
		self.accel = LSM9DS0.SensorInterface(self, XM, OUT_X_L_A, OUT_Y_L_A, OUT_Z_L_A, 0.061/1000)
	
	def init(self):
		# init gyro
		self.write_reg(G, CTRL_REG1_G, 0b00001111)
		self.write_reg(G, CTRL_REG2_G, 0b00000000)
		self.write_reg(G, CTRL_REG3_G, 0b10001000)
		self.write_reg(G, CTRL_REG4_G, 0b00000000)
		self.write_reg(G, CTRL_REG5_G, 0b00000000)
		
		# init accel
		self.write_reg(XM, CTRL_REG0_XM, 0b00000000)
		self.write_reg(XM, CTRL_REG1_XM, 0b01010111)
		self.write_reg(XM, CTRL_REG2_XM, 0b00000000)
		self.write_reg(XM, CTRL_REG3_XM, 0b00000100)
		
		# init mag
		self.write_reg(XM, CTRL_REG4_XM, 0b00000100)
		self.write_reg(XM, CTRL_REG5_XM, 0b10010100)
		self.write_reg(XM, CTRL_REG6_XM, 0b00000000)
		self.write_reg(XM, CTRL_REG7_XM, 0b00000000)
		
		return (
			self.read_reg(G, WHO_AM_I_G),
			self.read_reg(XM, WHO_AM_I_XM),
		)
	
	def read_reg(self, slave, reg, data=1):
		n_bytes = data if type(data) == int else len(data)
		return self.i2c.mem_read(
			data = data,
			addr = self.g_addr if not slave else self.xm_addr,
			memaddr = reg | 0x80 if n_bytes > 1 else reg,
		)
	
	def write_reg(self, slave, reg, data=0):
		self.i2c.mem_write(
			data = data,
			addr = self.g_addr if not slave else self.xm_addr,
			memaddr = reg,
		)
	
	
	
	class SensorInterface():
		def __init__(self, lsm9ds0, slave, x_reg, y_reg, z_reg, bit_value):
			self.lsm9ds0 = lsm9ds0
			self.slave = slave
			self.x_reg = x_reg
			self.y_reg = y_reg
			self.z_reg = z_reg
			self.bit_value = bit_value
		
		def x(self):
			b = self.lsm9ds0.read_reg(self.slave, self.x_reg, 2)
			return twos_comp(b[1]*256+b[0], 16)*self.bit_value
		
		def y(self):
			b = self.lsm9ds0.read_reg(self.slave, self.y_reg, 2)
			return twos_comp(b[1]*256+b[0], 16)*self.bit_value
		
		def z(self):
			b = self.lsm9ds0.read_reg(self.slave, self.z_reg, 2)
			return twos_comp(b[1]*256+b[0], 16)*self.bit_value
		
		def all(self):
			b = self.lsm9ds0.read_reg(self.slave, self.x_reg, 6)
			return (
				twos_comp(b[1]*256+b[0], 16)*self.bit_value,
				twos_comp(b[3]*256+b[2], 16)*self.bit_value,
				twos_comp(b[5]*256+b[4], 16)*self.bit_value,
			)



if __name__ == "__main__":
	import pyb
	
	i2c = pyb.I2C(2, mode=pyb.I2C.MASTER, baudrate=100000)
	
	lsm9ds0 = LSM9DS0(i2c)
	g, xm = lsm9ds0.init()
	
	while True:
		print(lsm9ds0.accel.all())
		mag_x = lsm9ds0.mag.x()
		gyro_z = lsm9ds0.gyro.z()
		pyb.delay(100)


