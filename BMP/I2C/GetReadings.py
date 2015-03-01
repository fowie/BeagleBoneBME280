import BME280
import time
import sys
from influxdb import client as influxdb


#dbs = [influxdb.InfluxDBClient('192.168.123.200', 8086, "USERNAME", "PASSWORD", "DATABASE"),
#	influxdb.InfluxDBClient('50.34.26.244', 8086, "USERNAME", "PASSWORD", "DATABASE")]
#DB contains simply an array called dbs like that shown above
from DB import *

bme = BME280.BME280(0x76)

while True:
	[upress, utemp, uhumid] = bme.GetUncompensatedValues()
	[pressure, temperature, humidity] = bme.CompensateAll(upress, utemp, uhumid)
	
	if pressure != -1.0 and temperature != -1.0 and humidity != -1.0:
		print "Got valid data, submitting."
		
		for db in dbs:
			print "To "+db._host
			try:
				data = [
					{"fields":{"value":pressure},
					"measurement":"Barometric Pressure",
					"tags":{"Sensor":"BME280"},
					}
				]
				db.write_points(data)
				data = [
					{"fields":{"value":temperature},
					"measurement":"Temperature",
					"tags":{"Sensor":"BME280"},
					}
				]
				db.write_points(data)
				data = [
					{"fields":{"value":humidity},
					"measurement":"Humidity",
					"tags":{"Sensor":"BME280"},
					}
				]
				db.write_points(data)
				print "Submission to "+db._host+" successful"
			except:
				print sys.exc_info()[0]
				print "Error submitting to db "+db._host
	print "Pressure: "+str(pressure)
	print "Temperature: "+str(temperature)
	print "Humidity: "+str(humidity)

	time.sleep(10)

