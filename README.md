# dassault-UAVC
Pattern recognition code used by my team for the Dassault UAV (Unmanned aerial vehicle) Challenge in October 2015.

In order to run, this program will need OpenCV, g++, and a working video stream (can also be set to work on a video).

This program has the following features:
- Detection and track 3 different patterns (arrow, square and cross).
- Correction the Fish-Eye effect of the embedded camera.
- Correction of ambient light and colors.
- possible discussion with another program by ZeroMQ (that is how this program could communicate with the drone's software.
