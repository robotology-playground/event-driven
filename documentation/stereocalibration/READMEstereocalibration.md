# RGB-Event system calibration

This is a guideline to calibrate a stereo system, in this case a RGB camera and an event-driven sensor. 
The calibration follows the classic procedure detecting a predefined pattern moved in front of the cameras.
The algorithm extracts key-points to compute the intrinsic and extrinsic parameters.

## Requirements

* Yarp 
* event-driven
* RGB camera and event-based camera able to connect with Yarp

## How to setup the calibration 

The calibration between frame-based camera and event-driven camera can work also with different resolutions of the cameras. Your only need to move a pattern in front of the cameras whilst the calibration is running. 
The algorithm will find the pattern in different positions of the board and will compute the intrinsic and extrinsic parameters. 

### How to use the video

The pattern used to calibrate both cameras is a video of a [checkerboard](https://github.com/robotology/event-driven/blob/camera-calibration-supervisor/documentation/stereocalibration/video_checkboard.mp4) flashing at around 60 Hz. 
Depending on the device you are using you will need to set the screen luminosity and the refresh rate (if allowed). 
If the refresh rate of the screen is enough to generate the events pattern, the video paused representing the checkerboard, already works for the calibration. 
Otherwise, you need to show the video to the cameras setting the video in loop.

Your vFramerLite parameters should be: \
 `--displays "(/right (BLACK))" --frameRate 30 --eventWindow 0.034`

### How to run the calibration

First run the framework `yarpmanager`. This tool allows you to run applications on Yarp Network. 
* In cluster run the server you need to have up. 
* In Entities search for `stereoCalib`, then run and connect ports and views.

The `stereoCalib` works exploiting the information in the `stereoCalib.ini` file. 
Please, check the parameters carefully. 
Furthermore, if you want to calibrate only one camera, you only need to set MonoCalib to 1. 

```
standalone

[STEREO_CALIBRATION_CONFIGURATION]

MonoCalib 0
boardWidth 8  -->number of slots
boardHeight 6   -->number of slots
boardSize 0.008
numberOfPairs 20
#boardType ASYMMETRIC_CIRCLES_GRID
```

After running the application, you need to open a RPC port.
`yarp rpc /stereoCalib/cmd`
Once the port is active, communication must be established to start the calibration:
`>>start`

Now, you can start with your calibration process moving the board in front of the cameras waiting for the pattern to be found. 




