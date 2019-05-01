#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <Windows.h>
#endif

#include "calib_mono.h"

#include <random>

using namespace cv;
using namespace std;

void mono_calibrate(string pathToImages, string filename, CalibParams& params){

    // creating config file
    FileStorage paramsFile(filename, FileStorage::WRITE);
    if(!paramsFile.isOpened()){
        cerr << "[Calibration mono] yml file could not be created: " << filename << endl;
        exit(-1);
    }

    //creating 3D structure of the pattern and detecting the corresponding 2D features
    vector<Point3f> structureBoard; //3D features of the chessboard (by default, z=0)
    vector<int> images_idx; // images where the pattern is found
    calcPatternPosition(structureBoard,params);
    // detecting pattern in all images and selecting a subset of MAX_IMAGES
    vector<vector<Point2f>> imagePoints = findPattern(pathToImages,images_idx,params);
    std::mt19937 rng(0xFFFFFFFF);
    while(imagePoints.size() > params.MAX_IMAGES){
        std::uniform_int_distribution<uint32_t> uniformDistro(0,imagePoints.size());
        imagePoints.erase(imagePoints.begin()+uniformDistro(rng));
    }

    cout << "[Calibration mono] Pattern found in " << imagePoints.size() << " images." << endl;

    // structure board for each set of points detected
    vector<vector<Point3f>> objectPoints(imagePoints.size(),structureBoard);
    // intrinsic and extrinsic parameters
    Mat D = Mat::zeros(5, 1, CV_64F);
    vector<Mat> rvecs,tvecs;
    Mat K = (Mat_<double>(3,3) << 500,0,params.image_size.width/2.0,0,500,params.image_size.height/2.0,0,0,1);

    //initial estimation
    cout << "[Calibration mono] estimating parameters ..." << endl;
    calibrateCamera(objectPoints,imagePoints,params.image_size,K,D,rvecs,tvecs,CV_CALIB_USE_INTRINSIC_GUESS);
    // compute initial MRE
    vector<double> mre_values;
    double mre = computeMRE(objectPoints,imagePoints,rvecs,tvecs,K,D,mre_values);
    cout << "[Calibration mono] initial MRE: " << mre << endl;

    while(mre > params.MAX_MRE && mre_values.size() > params.MIN_IMAGES){
        //selecting max error
        double max_mre = *(max_element(mre_values.begin(),mre_values.end()));
        for(int k=mre_values.size()-1;k>=0;k--){
            if(mre_values.size() <= params.MIN_IMAGES){
                cout << "BREAK" << endl;
                break;
            }
            // removing images whose MRE > 90% max MRE
            if(mre_values[k]> 0.9 * max_mre){
                objectPoints.erase(objectPoints.begin()+k);
                imagePoints.erase(imagePoints.begin()+k);
                rvecs.erase(rvecs.begin()+k);
                tvecs.erase(tvecs.begin()+k);
                mre_values.erase(mre_values.begin()+k);
            }
        }
        cout << "[Calibration mono] estimating parameters with " << mre_values.size() << " elements..." << endl;
        calibrateCamera(objectPoints,imagePoints,params.image_size,K,D,rvecs,tvecs,CV_CALIB_USE_INTRINSIC_GUESS);
        mre = computeMRE(objectPoints,imagePoints,rvecs,tvecs,K,D,mre_values);
        cout << "[Calibration mono] MRE: " << mre << endl;
    }

    cout << "[Calibration mono] K:" << endl << K << endl;
    cout << "[Calibration mono] D:" << endl << D.t() << endl;
    paramsFile << "K" << K << "D" << D << "MRE" << mre << "MINRE" << *(min_element(mre_values.begin(),mre_values.end())) << "MAXRE" << *(max_element(mre_values.begin(),mre_values.end()));
    paramsFile.release();

    cout << "[Calibration mono] parameters written in: " << filename << endl;
}

void mono_rectify(string pathToImages, string rectFolder, string filename,CalibParams& params){

    /** reading intrinsic files **/

    FileStorage intFile(filename, FileStorage::READ);
    unsigned dot = filename.find_last_of(".");
    FileStorage paramsFile(filename.substr(0,dot)+"_rectified.yml", CV_STORAGE_WRITE); // calibration parameters of rectified images

    if(!intFile.isOpened()){
        cerr << "[error] yml file could not be opened." << endl;
        exit(-1);
    }

    // filling the calibration parameters from the params file
    Mat K, D;
    intFile["K"] >> K;
    intFile["D"] >> D;

    /** reading images **/

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
    Mat new_K = getOptimalNewCameraMatrix(K, D, params.image_size, 0, params.image_size, 0);
    cv::initUndistortRectifyMap(K, D, Mat(),new_K, params.image_size, CV_16SC2, rmap[0], rmap[1]);

    // saving new calibration parameters
    if( paramsFile.isOpened() )
    {
        paramsFile << "K" << new_K << "D" << D;
        paramsFile.release();
    }
    else{
		std::cerr << "[error] can not save the calibration parameters\n";
		exit(-1);
    }

    /** rectifying imgs **/

    Mat img, rimg;
    cap >> img;
    while(!img.empty()){
		//creating the name of the rectified image
        stringstream ss;
        ss << rectFolder << params.cam_name.substr(0,percent) << setw(5) << setfill('0') << cap.get(CAP_PROP_POS_FRAMES) << "_rec.png";
        string nameimg = ss.str();
        std::cout << "[Rectification mono] " << nameimg << "\r";cout.flush();

		remap(img, rimg, rmap[0], rmap[1], CV_INTER_LINEAR);

        //if display, display the rectified image, else save into a file
        if(params.display)
            imshow("rectified",rimg);
        else
            imwrite(nameimg.c_str(),rimg); //create the image using Matrix roi

        cap >> img;
        char k = waitKey(params.display?0:100);
            if(k == 'q') // if q is pressed, stop the acquisition
                break;
    }

    std::cout<< endl << "[Rectification mono] Rectified images saved in: " << rectFolder << endl;
}

void mono_calibrateAndRectify(string pathToImages, string rectFolder, string paramsFile, CalibParams& params){
    mono_calibrate(pathToImages,paramsFile,params);
    mono_rectify(pathToImages,rectFolder,paramsFile,params);
}

