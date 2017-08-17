
- version 1.0

################
# DEPENDENCIES #
################

- Opencv 3 minimum

################
# INSTALLATION #
################

1. extract library folder
2. $ cmkdir build
3. $ cd build
4. $ cmake ..
5. $ make
6. $ sudo make install
7. $ make doc (optional for generating documentation)

#########
# USAGE #
#########

lab_mono_calibration [IMAGEPATH] [IMAGEEXP] [YMLPATH] -[OPTIONS]

- ./bin/lab_mono_calibration pathToImageFolder cam1_image%05d.png pathToYMLFile.yml       | using a sequence of images
- ./bin/lab_mono_calibration pathToImageFolder calibration_vid.mp4 pathToYMLFile.yml      | using a video
- ./bin/lab_mono_calibration /dev video0 pathToYMLFile.yml                                | using the camera
 
#####################
# Supported options #
#####################

- -H,-h  display this help.
- -n     specify the minimum number of images to stop the calibration.
- -m     specify the MRE threshold to stop the calibration.
- -e     set the size of the elements of the chessboard.
- -d     display rectified images, otherwise save them.
- -c     calibrate and rectify with the provided images.
- -i     interval rate at which images are processed.
