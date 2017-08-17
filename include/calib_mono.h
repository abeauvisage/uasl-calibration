#ifndef CALIB_MONO_H_INCLUDED
#define CALIB_MONO_H_INCLUDED

#include <sys/stat.h>
#include "wingetopt.h"

#include "patternDetection.h"

//! Calibrate from the provided images.
/*! save the calibration parameters into the yml file provided.*/
void mono_calibrate(std::string pathToImages, std::string filename, CalibParams params=CalibParams());
//! Rectify images using the parameters file provided.
void mono_rectify(std::string pathToImages, std::string rectFolder, std::string filename, CalibParams params=CalibParams());
//! Calibrate from the provided images and rectify them.
/*! calls mono_calibrate and mono_rectify.*/
void mono_calibrateAndRectify(std::string pathToImages, std::string rectFolder, std::string paramsFile, CalibParams params=CalibParams());

#endif // CALIB_MONO_H_INCLUDED
