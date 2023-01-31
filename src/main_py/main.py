#!/usr/bin/env pybricks-micropython
#arber è stato qui
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
R_sensor = ColorSensor(Port.S3)
L_sensor = ColorSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Inserisci la soglia massima per il valore dei colori. In base ai valori sperimentali.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Velocità standard in mm/s.
DRIVE_SPEED = 100

# Set the gain of the proportional line controller.per ogni x percentuale di luce il robot
# gira di questi percentuale, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# per esempio se la percentuale di luce riflessa è 10 , the robot
# sterza a 10*1.2 = 12 gradi al secondo.
PROPORTIONAL_GAIN = 1.2

# Start following the line endlessly.
while True:
    # Calculate the deviation from the threshold.
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)