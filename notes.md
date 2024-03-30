# Balloon tracking using particle filter

## Overview

This mini project aims to track a high altitude balloon using a particle filter. 
The balloon is tracked using a GPS receiver and an accelerometer.
The GPS receiver provides the position of the balloon and the accelerometer provides the acceleration of the balloon.
The particle filter uses the GPS position and the accelerometer data to estimate the position of the balloon.

## GPS Receiver

The GPS receiver provides the position of the balloon in the form of latitude, longitude and altitude.
The sensor data is provided in the form of the ASCII protocol NMEA 0183.
A third party parser is used to extract the position data from the NMEA 0183 data.

## Accelerometer

The accelerometer is probably I2C or SPI ??

## State Space Model

The state space model for the balloon tracking problem is as follows:

bla bla bla

## Particle Filter Algorithm

The particle filter algorithm is as follows:

1. Initialize particles with random positions and weights.
2. ...



