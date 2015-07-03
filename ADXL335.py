"""
Based on the SparkFun Triple Axis Accelerometer Breakout: ADXL335.
https://www.sparkfun.com/products/9269

Code by xix xeaon @ XIXIT.

See usage at the bottom.
"""

class ADXL335():
	"""ADXL335 object allows you to easily read the acceleration in G."""
	def __init__(self, adc_x, adc_y, adc_z, v_in=3.3):
		"""initialize with pyb.ADC objects, and optional measured voltage"""
		self.adc_x = adc_x
		self.adc_y = adc_y
		self.adc_z = adc_z
		self.v_in = v_in
		self.g0 = self.v_in/2
		self.g1 = self.v_in/10
	
	def x(self):
		"""returns current G-normalized force in X direction"""
		return (self.v_in * self.adc_x.read() / (4096-1) - self.g0) / self.g1
	
	def y(self):
		"""returns current G-normalized force in Y direction"""
		return (self.v_in * self.adc_y.read() / (4096-1) - self.g0) / self.g1
	
	def z(self):
		"""returns current G-normalized force in Z direction"""
		return (self.v_in * self.adc_z.read() / (4096-1) - self.g0) / self.g1
	
	def all(self):
		"""returns current G-normalized force in all 3 directions"""
		return (self.x(), self.y(), self.z())



if __name__ == "__main__":
	import pyb
	
	adxl335 = ADXL335(
		pyb.ADC(pyb.Pin.board.X2),
		pyb.ADC(pyb.Pin.board.X3),
		pyb.ADC(pyb.Pin.board.X4),
	)
	
	while True:
		#print((adxl335.x(), adxl335.y(), adxl335.z()))
		print(adxl335.all())
		pyb.delay(100)


