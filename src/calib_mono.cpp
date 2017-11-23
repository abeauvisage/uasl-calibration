#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <Windows.h>
#endif

#include "calib_mono.h"

using namespace cv;
using namespace std;

void mono_calibrate(string pathToImages, string filename, CalibParams& params){
    //creating 3D structure of the pattern and detecting the corresponding 2D features
    vector<Point3f> structureBoard; //3D features of the chessboard (by default, z=0)
    vector<int> images_idx; // images where the pattern is found
    calcPatternPosition(structureBoard,params);
    vector<vector<Point2f>> imagePoints = findPattern(pathToImages,images_idx,params);
    cout << "[Calibration] Pattern found in " << imagePoints.size() << " images." << endl;

    FileStorage paramsFile(filename, FileStorage::WRITE);
    if(!paramsFile.isOpened()){
        cerr << "[Calibration] yml file could not be created." << endl;
        cerr << filename << endl;
        exit(-1);
    }

	cout << "[Calibration] " << params.image_size << endl;

    vector<vector<Point3f>> objectPoints(imagePoints.size(),structureBoard);

    Mat distCoeffs, R,P,T;
    vector<Mat >rvecs,tvecs;
    double cm[9] ={500,0,params.image_size.width/2.0,0,500,params.image_size.height/2.0,0,0,1};
    Mat cameraMatrix(3, 3, CV_64F,cm);
    distCoeffs = Mat::zeros(5, 1, CV_64F);
    R = Mat::eye(3, 3, CV_64F);
    P = Mat::eye(3, 4, CV_64F);
    T = Mat::zeros(3,1,CV_64F);

    cout << "[Calibration] estimating parameters ..." << endl;
    calibrateCamera(objectPoints,imagePoints,params.image_size,cameraMatrix,distCoeffs,rvecs,tvecs,CV_CALIB_USE_INTRINSIC_GUESS);
    cout << "[Calibration] done!" << endl;

    vector<double> values;
    double mre = computeMRE(objectPoints,imagePoints,rvecs,tvecs,cameraMatrix,distCoeffs,values),prev_mre=0;
    while(mre > params.MAX_MRE && values.size() > params.MIN_IMAGES && fabs(mre-prev_mre) > 0.0){
        prev_mre = mre;
        cout << "[Calibration] MRE: " << mre << endl;
        double max_mre = *(max_element(values.begin(),values.end()));
        for(int k=values.size()-1;k>=0;k--){
            if(values[k]> 0.9 * max_mre){
                objectPoints.erase(objectPoints.begin()+k);
                imagePoints.erase(imagePoints.begin()+k);
                rvecs.erase(rvecs.begin()+k);
                tvecs.erase(tvecs.begin()+k);
            }
        }
        cout << "[Calibration] " << values.size() << " elements..." << endl;
        calibrateCamera(objectPoints,imagePoints,params.image_size,cameraMatrix,distCoeffs,rvecs,tvecs,CV_CALIB_USE_INTRINSIC_GUESS);
        mre = computeMRE(objectPoints,imagePoints,rvecs,tvecs,cameraMatrix,distCoeffs,values);
    }

    cout << "[Calibration] final MRE: " << mre << endl;
    cout << "[Calibration] " << values.size() << " elements..." << endl;
    cout << "[Calibration] K:" << endl << cameraMatrix << endl;
    cout << "[Calibration] D:" << endl << distCoeffs.t() << endl;
    paramsFile << "K" << cameraMatrix << "D" << distCoeffs;
    paramsFile.release();

    cout << "[Calibration] parameters written in: " << filename << endl;
}

void mono_rectify(string pathToImages, string rectFolder, string filename,CalibParams& params){

    FileStorage intFile(filename, FileStorage::READ);
    unsigned dot = filename.find_last_of(".");
    FileStorage paramsFile(filename.substr(0,dot)+"_rectified.yml", CV_STORAGE_WRITE);
    namedWindow("rectified",WINDOW_NORMAL);
    if(!intFile.isOpened()){
        cerr << "[error] yml file could not be opened." << endl;
        exit(-1);
    }

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F),distCoeffs = Mat::eye(5, 1, CV_64F);

    // filling the calibration parameters from the params file
    intFile["K"] >> cameraMatrix;
    intFile["D"] >> distCoeffs;


    VideoCapture cap;cap.open(pathToImages+"/"+params.cam_name);
    unsigned percent =  params.cam_name.find_last_of("%");
	if(!cap.isOpened()){
        std::cerr << "could not open video or find images. exiting..." << std::endl;
        exit(-1);
    }else{
    	params.image_size.height = cap.get(CAP_PROP_FRAME_HEIGHT);params.image_size.width = cap.get(CAP_PROP_FRAME_WIDTH);
	}

	//computing the mapping function
    Mat rmap[2];
    Mat new_cameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, params.image_size, 0, params.image_size, 0);
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),new_cameraMatrix, params.image_size, CV_16SC2, rmap[0], rmap[1]);

//    cout << cameraMatrix << distCoeffs << endl;
//
    if( paramsFile.isOpened() )
    {
        paramsFile << "K" << new_cameraMatrix << "D" << distCoeffs;
        paramsFile.release();
    }
    else{
		std::cerr << "[error] can not save the calibration parameters\n";
		exit(-1);
    }

    Mat img, rimg;
    cap >> img;
    while(!img.empty()){

		//creatign the name of the rectified image
        stringstream ss;
        ss << rectFolder << params.cam_name.substr(0,percent) << setw(5) << setfill('0') << cap.get(CAP_PROP_POS_FRAMES) << "_rec.png";
        string nameimg = ss.str();
        std::cout << "[Rectification] " << nameimg << "\r";cout.flush();

		remap(img, rimg, rmap[0], rmap[1], CV_INTER_LINEAR);

        //if display, display the rectified image, else save into a file
        if(params.display){
            imshow("rectified",rimg);
            char k = waitKey(100);
            if(k == 'c') // if c is pressed, stop the acquisition
                break;
        }
        else{
            imwrite(nameimg.c_str(),rimg); //create the image using Matrix roi
		}

        cap >> img;
    }

    if(!params.display)
        std::cout<< endl << "[Rectification] Rectified images saved in: " << rectFolder << endl;
}

void mono_calibrateAndRectify(string pathToImages, string rectFolder, string paramsFile, CalibParams& params){

    mono_calibrate(pathToImages,paramsFile,params);
    mono_rectify(pathToImages,rectFolder,paramsFile,params);
}

