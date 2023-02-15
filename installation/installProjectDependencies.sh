#!/bin/bash

#A Python library that enables the use of Jetson's GPIOs
sudo -H python3 -m pip install inputs

#Install driver for the MotorDriver, I2C and PowerSupply
sudo -H python3 -m pip install Adafruit-MotorHAT

#Install support of "Typed Attributes" enforcement framework
sudo -H python3 -m pip install traitlets

#Install support of 'PyGame"
sudo -H python3 -m pip install pygame

#Add User to Input Group
sudo gpasswd -a $USER input