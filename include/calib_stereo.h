#ifndef CALIB_STEREO_H_INCLUDED
#define CALIB_STEREO_H_INCLUDED

#include "patternDetection.h"

//! Calibrate from the provided images.
/*! save the calibration parameters into the yml file provided.*/
void stereo_calibrate(std::string pathToImages, std::string filename, CalibParams& params);
//! Rectify images using the parameters file provided.
void stereo_rectify(std::string pathToImages, std::string rectFolder, std::string filename, CalibParams& params);
//! Calibrate from the provided images and rectify them.
/*! calls stereo_calibrate and stereo_rectify.*/
void stereo_calibrateAndRectify(std::string pathToImages, std::string rectFolder, std::string paramsFile, CalibParams& params);



#endif // CALIB_STEREO_H_INCLUDED
