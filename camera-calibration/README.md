# Camera Calibration

This is code for calibrating cameras using openCV. It doesn't actually do anything at the moment but it can get you
the camera's focal length and focal center (needed for pose estimation), as well as the numbers to compensate
for lens distortion.

The python script is 99.9% stolen from [the openCV guide](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

The image grabbing script takes one argument which should be an integer number of frames to grab. It'll save them all as numbered
jpg files.
