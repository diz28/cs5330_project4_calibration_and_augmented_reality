#### Di Zhang
#### Feb 10, 2023
#### CS5330 - Computer Vision

# cs5330_project4_calibration_and_augmented_reality


## Project Description

The mean objective of this project is to construct augmented reality object on chessborads. With the correct camera calibration you can do a lot of fun stuff on top of the chessborad. Have fun!

## How to install and run

### Have the following 4 programs ready on your **Linux** machine:

- opencv 4.7.0
- g++
- vscode
- make

### How to run in terminal


1. have a camera ready and connected to your machine.
2. go into the project folder in terminal and type in `make`, a excutable `main` should appear.
3. run `main` in terminal, a video window would pop up.
4. prepare a 7 by 10 chessboard(printed for best result)
5. have your camera pointed at it, you would see all the corners appear on top of the chessboard
6. move your camera around and press `s` to take 7 to 10 images as calibration images
7. then press `c` you should be able to see a 3d axis on one corner of the chessboard, and a 3d Y on top of the chessboard
8. you can also point the camera to a `ArUco` marker to see a preset image overlaied on top of the marker
9. you can press `z` to go back to normal
10. you can also have 2 chessboard in the same frame and both of them are going to show the corners
11.  press `v` to show the `SIFT` features


## Program features (matching mechanisms)

### Basic requirments
1. press `s` to take calibration images
2. press `c` to calibrate camera
3. press `p` to show 3d objects
4. press `z` to go back to only show the corners
5. calibration paramenters can be saved to `yaml` files, so the user does not need to calibrate everytime.


#### Extensions
- you can point the camera to a `ArUco` and have a preset image overlaied on top of it
- when run the program, you can have a chessboard image as the second argument and allow the program to run on the image
- made the 3d object a solid object instead of lines
- you can have 2 chessboards in the image, and both of them are going to show the corners


### Time travel days - 3