from Adafruit_I2C import Adafruit_I2C
import time
import array
from struct import *

class BME280:
	
	T1 = 0.0
	T2 = 0.0
	T3 = 0.0
	P1 = 0.0
	P2 = 0.0
	P3 = 0.0
	P4 = 0.0
	P5 = 0.0
	P6 = 0.0
	P7 = 0.0
	P8 = 0.0
	H1 = 0.0
	H2 = 0.0
	H3 = 0.0
	H4 = 0.0
	H5 = 0.0
	H6 = 0.0
	T_FINE = 0.0

	Temperature = 0.0;
	Humidity = 0.0;
	Pressure = 0.0;
	
	CHIP_ID_REG =                  (0xD0)  #/*Chip ID Register */
	RST_REG =                      (0xE0)  #/*Softreset Register */
	STAT_REG =                     (0xF3)  #/*Status Register */
	CTRL_MEAS_REG =                (0xF4)  #/*Ctrl Measure Register */
	CTRL_HUMIDITY_REG =            (0xF2)  #/*Ctrl Humidity Register*/
	CONFIG_REG =                   (0xF5)  #/*Configuration Register */
	PRESSURE_MSB_REG =             (0xF7)  #/*Pressure MSB Register */
	PRESSURE_LSB_REG =             (0xF8)  #/*Pressure LSB Register */
	PRESSURE_XLSB_REG =            (0xF9)  #/*Pressure XLSB Register */
	TEMPERATURE_MSB_REG =          (0xFA)  #/*Temperature MSB Reg */
	TEMPERATURE_LSB_REG =          (0xFB)  #/*Temperature LSB Reg */
	TEMPERATURE_XLSB_REG =         (0xFC)  #/*Temperature XLSB Reg */
	HUMIDITY_MSB_REG =             (0xFD)  #/*Humidity MSB Reg */
	HUMIDITY_LSB_REG =             (0xFE)  #/*Humidity LSB Reg */
	TEMPERATURE_CALIB_DIG_T1_LSB_REG =            (0x88)
	TEMPERATURE_CALIB_DIG_T1_MSB_REG =            (0x89)
	TEMPERATURE_CALIB_DIG_T2_LSB_REG =            (0x8A)
	TEMPERATURE_CALIB_DIG_T2_MSB_REG =            (0x8B)
	TEMPERATURE_CALIB_DIG_T3_LSB_REG =            (0x8C)
	TEMPERATURE_CALIB_DIG_T3_MSB_REG =            (0x8D)
	PRESSURE_CALIB_DIG_P1_LSB_REG =               (0x8E)
	PRESSURE_CALIB_DIG_P1_MSB_REG =               (0x8F)
	PRESSURE_CALIB_DIG_P2_LSB_REG =               (0x90)
	PRESSURE_CALIB_DIG_P2_MSB_REG =               (0x91)
	PRESSURE_CALIB_DIG_P3_LSB_REG =               (0x92)
	PRESSURE_CALIB_DIG_P3_MSB_REG =               (0x93)
	PRESSURE_CALIB_DIG_P4_LSB_REG =               (0x94)
	PRESSURE_CALIB_DIG_P4_MSB_REG =               (0x95)
	PRESSURE_CALIB_DIG_P5_LSB_REG =               (0x96)
	PRESSURE_CALIB_DIG_P5_MSB_REG =               (0x97)
	PRESSURE_CALIB_DIG_P6_LSB_REG =               (0x98)
	PRESSURE_CALIB_DIG_P6_MSB_REG =               (0x99)
	PRESSURE_CALIB_DIG_P7_LSB_REG =               (0x9A)
	PRESSURE_CALIB_DIG_P7_MSB_REG =               (0x9B)
	PRESSURE_CALIB_DIG_P8_LSB_REG =               (0x9C)
	PRESSURE_CALIB_DIG_P8_MSB_REG =               (0x9D)
	PRESSURE_CALIB_DIG_P9_LSB_REG =               (0x9E)
	PRESSURE_CALIB_DIG_P9_MSB_REG =               (0x9F)
	HUMIDITY_CALIB_DIG_H1_REG     =               (0xA1)
	HUMIDITY_CALIB_DIG_H2_LSB_REG =               (0xE1)
	HUMIDITY_CALIB_DIG_H2_MSB_REG =               (0xE2)
	HUMIDITY_CALIB_DIG_H3_REG     =               (0xE3)
	HUMIDITY_CALIB_DIG_H4_MSB_REG =               (0xE4)
	HUMIDITY_CALIB_DIG_H4_LSB_REG =               (0xE5)
	HUMIDITY_CALIB_DIG_H5_MSB_REG =               (0xE6)
	HUMIDITY_CALIB_DIG_H6_REG     =               (0xE7)
	
	DATA_FRAME_PRESSURE_MSB_BYTE	=   	(0)
	DATA_FRAME_PRESSURE_LSB_BYTE	=		(1)
	DATA_FRAME_PRESSURE_XLSB_BYTE =		(2)
	DATA_FRAME_TEMPERATURE_MSB_BYTE =	(3)
	DATA_FRAME_TEMPERATURE_LSB_BYTE =	(4)
	DATA_FRAME_TEMPERATURE_XLSB_BYTE =	(5)
	DATA_FRAME_HUMIDITY_MSB_BYTE	=		(6)
	DATA_FRAME_HUMIDITY_LSB_BYTE	=		(7)
	
	TEMPERATURE_CALIB_DIG_T1_LSB=		(0)
	TEMPERATURE_CALIB_DIG_T1_MSB=		(1)
	TEMPERATURE_CALIB_DIG_T2_LSB=		(2)
	TEMPERATURE_CALIB_DIG_T2_MSB=		(3)
	TEMPERATURE_CALIB_DIG_T3_LSB=		(4)
	TEMPERATURE_CALIB_DIG_T3_MSB=		(5)
	PRESSURE_CALIB_DIG_P1_LSB=       (6)
	PRESSURE_CALIB_DIG_P1_MSB=       (7)
	PRESSURE_CALIB_DIG_P2_LSB=       (8)
	PRESSURE_CALIB_DIG_P2_MSB=       (9)
	PRESSURE_CALIB_DIG_P3_LSB=       (10)
	PRESSURE_CALIB_DIG_P3_MSB=       (11)
	PRESSURE_CALIB_DIG_P4_LSB=       (12)
	PRESSURE_CALIB_DIG_P4_MSB=       (13)
	PRESSURE_CALIB_DIG_P5_LSB=       (14)
	PRESSURE_CALIB_DIG_P5_MSB=       (15)
	PRESSURE_CALIB_DIG_P6_LSB=       (16)
	PRESSURE_CALIB_DIG_P6_MSB=       (17)
	PRESSURE_CALIB_DIG_P7_LSB=       (18)
	PRESSURE_CALIB_DIG_P7_MSB=       (19)
	PRESSURE_CALIB_DIG_P8_LSB=       (20)
	PRESSURE_CALIB_DIG_P8_MSB=       (21)
	PRESSURE_CALIB_DIG_P9_LSB=       (22)
	PRESSURE_CALIB_DIG_P9_MSB=       (23)
	HUMIDITY_CALIB_DIG_H1=           (25)
	HUMIDITY_CALIB_DIG_H2_LSB=		(0)
	HUMIDITY_CALIB_DIG_H2_MSB=		(1)
	HUMIDITY_CALIB_DIG_H3=			(2)
	HUMIDITY_CALIB_DIG_H4_MSB=		(3)
	HUMIDITY_CALIB_DIG_H4_LSB=		(4)
	HUMIDITY_CALIB_DIG_H5_MSB=		(5)
	HUMIDITY_CALIB_DIG_H6=			(6)
	MASK_DIG_H4=		(0x0F)
	
	def __init__(self, address):
		#Setup the I2C component
		self.i2c = Adafruit_I2C(address=0x76)
		#Verify the ID
		if(self.ID() != 0x60):
			print "ERROR!  Got an invalid ID: "+hex(self.ID())

		#Perform a soft reset
		self.SoftReset()
		#set data acquisition options
		self.i2c.write8(self.CTRL_HUMIDITY_REG, 0x01)  #x1 humidity oversampling
		self.i2c.write8(self.CTRL_MEAS_REG, 0xAB)      # 10101011 - 16x pressure, 2x temperature, normal mode
		self.i2c.write8(self.CONFIG_REG, 0x08)        # 0001000 - filter 16, t_standby 0.5ms
		#Read in calibration data
		print "Initialization complete.  Waiting for first valid measurement."
		time.sleep(0.007125)  #sleep 7.125ms for acquisition
		self.GetCalibrationValues()
		[p, t, h] = self.GetUncompensatedValues()
		while p == t:
			time.sleep(0.5)
			[p, t, h] = self.GetUncompensatedValues()
		print "Sensor ready."

	def ID(self):
		return self.i2c.readS8(self.CHIP_ID_REG)

	def SoftReset(self):
		self.i2c.write8(self.RST_REG, 0xB6)
		time.sleep(0.003)

	def Measuring(self):
		#status = self.i2c.readS8(STAT_REG)
		#measuring = status & 0x04
		#if(measuring > 0):
		#	print "Measuring"
		#	return True
		#return False
		return False

	def CopyingData(self):
		status = self.i2c.readS8(self.STAT_REG)
		im_update = status & 0x01
		if(im_update > 0):
			#print "Copying"
			return True
		return False

	def GetCalibrationValues(self):
		data = array.array('B', self.i2c.readList(self.TEMPERATURE_CALIB_DIG_T1_LSB_REG, 26))
		#print "CalibValues: "
		#print data
		dS = data.tostring()
		self.T1 = unpack(">H", dS[self.TEMPERATURE_CALIB_DIG_T1_MSB]+dS[self.TEMPERATURE_CALIB_DIG_T1_LSB])[0]
		self.T2 = unpack(">h", dS[self.TEMPERATURE_CALIB_DIG_T2_MSB]+dS[self.TEMPERATURE_CALIB_DIG_T2_LSB])[0]
		self.T3 = unpack(">h", dS[self.TEMPERATURE_CALIB_DIG_T3_MSB]+dS[self.TEMPERATURE_CALIB_DIG_T3_LSB])[0]
		self.P1 = unpack(">H", dS[self.PRESSURE_CALIB_DIG_P1_MSB]+dS[self.PRESSURE_CALIB_DIG_P1_LSB])[0]
		self.P2 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P2_MSB]+dS[self.PRESSURE_CALIB_DIG_P2_LSB])[0]
		self.P3 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P3_MSB]+dS[self.PRESSURE_CALIB_DIG_P3_LSB])[0]
		self.P4 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P4_MSB]+dS[self.PRESSURE_CALIB_DIG_P4_LSB])[0]
		self.P5 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P5_MSB]+dS[self.PRESSURE_CALIB_DIG_P5_LSB])[0]
		self.P6 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P6_MSB]+dS[self.PRESSURE_CALIB_DIG_P6_LSB])[0]
		self.P7 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P7_MSB]+dS[self.PRESSURE_CALIB_DIG_P7_LSB])[0]
		self.P8 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P8_MSB]+dS[self.PRESSURE_CALIB_DIG_P8_LSB])[0]
		self.P9 = unpack(">h", dS[self.PRESSURE_CALIB_DIG_P9_MSB]+dS[self.PRESSURE_CALIB_DIG_P9_LSB])[0]
		self.H1 = unpack(">B", dS[self.HUMIDITY_CALIB_DIG_H1])[0]
		
		data = array.array('B', self.i2c.readList(self.HUMIDITY_CALIB_DIG_H2_LSB_REG, 7))
		#print "CalibValues2: "
		#print data
		dS = data.tostring()
		self.H2 = unpack(">h", dS[self.HUMIDITY_CALIB_DIG_H2_MSB]+dS[self.HUMIDITY_CALIB_DIG_H2_LSB])[0]
		self.H3 = unpack(">B", dS[self.HUMIDITY_CALIB_DIG_H3])[0]

		#I'm not sure this works ax expected, but the datasheet calls this a signed short
		#even though the algorithm only fills the lower 12 bits, meaning it will always be unsigned
		self.H4 = unpack(">b", dS[self.HUMIDITY_CALIB_DIG_H4_MSB])[0] << 4 | (0x0F & unpack(">b", dS[self.HUMIDITY_CALIB_DIG_H4_LSB])[0])
		self.H5 = unpack(">b", dS[self.HUMIDITY_CALIB_DIG_H5_MSB])[0] << 4 | (unpack(">b", dS[self.HUMIDITY_CALIB_DIG_H4_LSB])[0] >> 4)
		self.H6 = unpack(">b", dS[self.HUMIDITY_CALIB_DIG_H6])[0]

		#self.PrintCalibrationValues();

	def PrintCalibrationValues(self):
		print "Calibration values:"
		print "T1 = "+str(self.T1)
		print "T2 = "+str(self.T2)
		print "T3 = "+str(self.T3)
		print "P1 = "+str(self.P1)
		print "P2 = "+str(self.P2)
		print "P3 = "+str(self.P3)
		print "P4 = "+str(self.P4)
		print "P5 = "+str(self.P5)
		print "P6 = "+str(self.P6)
		print "P7 = "+str(self.P7)
		print "P8 = "+str(self.P8)
		print "P9 = "+str(self.P9)
		print "H1 = "+str(self.H1)
		print "H2 = "+str(self.H2)
		print "H3 = "+str(self.H3)
		print "H4 = "+str(self.H4)
		print "H5 = "+str(self.H5)
		print "H6 = "+str(self.H6)

	def GetUncompensatedValues(self):
		while self.Measuring() or self.CopyingData():
			time.sleep(0.005)

		data = array.array('B', self.i2c.readList(self.PRESSURE_MSB_REG, 8))
		#print "Got:"
		#print data

		#/*Pressure*/
		uncomp_pressure = data[self.DATA_FRAME_PRESSURE_MSB_BYTE] << 12 | data[self.DATA_FRAME_PRESSURE_LSB_BYTE] << 4 | data[self.DATA_FRAME_PRESSURE_XLSB_BYTE] >> 4

		#/* Temperature */
		uncomp_temperature = data[self.DATA_FRAME_TEMPERATURE_MSB_BYTE] << 12 | data[self.DATA_FRAME_TEMPERATURE_LSB_BYTE] << 4 | data[self.DATA_FRAME_TEMPERATURE_XLSB_BYTE] >> 4

		#/*Humidity*/
		uncomp_humidity = data[self.DATA_FRAME_HUMIDITY_MSB_BYTE] << 8 | data[self.DATA_FRAME_HUMIDITY_LSB_BYTE]

		#print "Pressure: " + str(uncomp_pressure) + " Temp: "+str(uncomp_temperature)+" Humidity: "+str(uncomp_humidity)
		return [uncomp_pressure, uncomp_temperature, uncomp_humidity]

	def CompensateAll(self, upress, utemp, uhumidity):
		temperature = self.CompensateTemperature(utemp)
		pressure = self.CompensatePressure(upress)
		humidity = self.CompensateHumidity(uhumidity)
		return [pressure, temperature, humidity]

	def CompensateTemperature(self, utemp):
		x1 = 0.0
		x2 = 0.0
		self.Temperature = 0.0

		x1 = (utemp / 16384.0 - self.T1 / 1024.0) * self.T2
		x2 = ((utemp / 131072.0 - self.T1 / 8192.0) * (utemp / 131072.0 - self.T1 / 8192.0)) * self.T3;
		self.T_FINE = x1 + x2
		self.Temperature  = (x1 + x2) / 5120.0;

		return self.Temperature

	def CompensateHumidity(self, uhumid):
		self.Humidity = 0.0

		self.Humidity = self.T_FINE - 76800.0;
		if self.Humidity != 0.0:
			self.Humidity = (uhumid - (self.H4 * 64.0 +	self.H5 / 16384.0 * self.Humidity))*(self.H2 / 65536.0 * (1.0 + self.H6	/ 67108864.0 * self.Humidity * (1.0 + self.H3 / 67108864.0 * self.Humidity)))
		else:
			return -1.0;
		self.Humidity = self.Humidity * (1.0 - self.H1*self.Humidity / 524288.0)
		if (self.Humidity > 100.0):
			self.Humidity = 100.0
		elif (self.Humidity < 0.0):
			self.Humidity = 0.0
		return self.Humidity

	def CompensatePressure(self, upress):
		x1 = 0.0
		x2 = 0.0
		self.Pressure = 0.0

		x1 = (self.T_FINE /	2.0) - 64000.0
		x2 = x1 * x1 * self.P6 / 32768.0
		x2 = x2 + x1 * self.P5 * 2.0
		x2 = (x2 / 4.0) + (self.P4 * 65536.0)
		x1 = (self.P3 *	x1 * x1 / 524288.0 + self.P2 * x1) / 524288.0
		x1 = (1.0 + x1 / 32768.0) *	self.P1
		self.Pressure = 1048576.0 - upress
		#/* Avoid exception caused by division by zero */
		if (x1 != 0.0):
			self.Pressure = (self.Pressure - (x2 / 4096.0)) * 6250.0 / x1
		else:
			return -1.0
		x1 = self.P9 * self.Pressure * self.Pressure / 2147483648.0
		x2 = self.Pressure * self.P8 / 32768.0
		self.Pressure = self.Pressure + (x1 + x2 + self.P7) / 16.0
	
		return self.Pressure;


