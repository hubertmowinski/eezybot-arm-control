from easyEEZYbotARM.serial_communication import arduinoController

# Insert your Arduino serial port here
myArduino = arduinoController(port="COM6")

# Create some test data.
# The angles used for this test data are 'raw' servo angles (i.e. not calibrated)
# against the kinematic model for the EEZYbotARM. This example moves the EzzyBot base (q1)
testData = []
testData.append(myArduino.composeMessage(servoAngle_q1=90,
                                         servoAngle_q2=90,
                                         servoAngle_q3=90,
                                         servoAngle_EE=10))

testData.append(myArduino.composeMessage(servoAngle_q1=30,
                                         servoAngle_q2=90,
                                         servoAngle_q3=100,
                                         servoAngle_EE=90))


# The connection should be managed in this sequence (will be simplified in future)
# Open a serial port and connect to the Arduino
myArduino.openSerialPort()
# Send the test data which is managed by the 'run test' function'
myArduino.communicate(data=testData, delay_between_commands=5)
myArduino.closeSerialPort()  # Close the serial port