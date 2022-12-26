# Camera Calibration

This is code for calibrating cameras using openCV. It doesn't actually do anything at the moment but it can get you
the camera's focal length and focal center (needed for pose estimation), as well as the numbers to be able to compensate
for lens distortion if we want to do this in the future.

It only works for "normal" cameras that can be accessed using raw openCV functions (i.e. not the flir)

The python script is 98% stolen from [the openCV guide](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

The image grabbing script takes one argument which should be an integer number of frames to grab. It'll save them all as numbered
jpg files. Don't use it, just use the calibration script now.

## Instructions
1. Print the [9x6 opencv chessboard pattern](https://github.com/opencv/opencv/blob/4.x/samples/data/chessboard.png) and put it on something flat.
2. Update square size in the python script (optional if all you're getting are camera properties).
3. Run the python script.
  - It will show the stream from the camera. Press space to save current frame as a calibration image. Press q when done collecting images.
  - Get at least 10 calibration images of the chessboard from many different angles. More images is better.
4. It will print out the focal lengths and focal centers. Edit the script if you want more info.
