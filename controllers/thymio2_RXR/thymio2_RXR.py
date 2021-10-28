"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
from controller import Compass

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

#compass = robot.getDevice("compass")
#compass.enable(timeStep)

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.8 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

head = 0

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
    #compassValues = compass.getValues()
    if head == 0:
        #initialCompassValue = compassValues[0]
        head = 1
    
    if centralRightSensorValue + outerRightSensorValue + centralLeftSensorValue + outerLeftSensorValue == 0:
        leftMotorVelocity = leftMotor.getVelocity()
        rightMotorVelocity = rightMotor.getVelocity()
        
        if compassValues[0] < initialCompassValue:
             rightMotor.setVelocity(rightMotorVelocity + 1)
            
        elif compassValues[0] > initialCompassValue:
             leftMotor.setVelocity(leftMotorVelocity + 1)
            
    else:
        leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue))
        rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) - centralSensorValue)