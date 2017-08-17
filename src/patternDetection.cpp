#include "patternDetection.h"

/**** Global variables for pattern detection ****/
static cv::Point2f meanPT;
static cv::Point2f p1,p2;
static std::vector<cv::Point2f> ccenters,cornersP;
/************************************************/

//! Sorting function. Sorts keypoints by distance to the centre of the pattern.
bool sortKpt(cv::KeyPoint kpt1, cv::KeyPoint kpt2){return (cv::norm(kpt1.pt-meanPT) < cv::norm(kpt2.pt-meanPT));}
//!Sorting function. Sorts features by distance to the centre of the pattern.
bool sortPt(cv::Point2f pt1, cv::Point2f pt2){return (cv::norm(pt1-meanPT) < cv::norm(pt2-meanPT));}

//! Sorting function. Sorts corner features by their angle
bool sortAngle(cv::Point2f kpt1, cv::Point2f kpt2){
    cv::Point2f pt1 = kpt1-meanPT, pt2 = kpt2-meanPT;
    float det1 = pt1.x*p1.y-(pt1.y*p1.x), det2 = pt2.x*p1.y-(pt2.y*p1.x);
    return ( (det1>0?1:-1) * acos((pt1.x*p1.x+pt1.y*p1.y)/(cv::norm(pt1)*cv::norm(p1))) < (det2>0?1:-1) * acos((pt2.x*p1.x+pt2.y*p1.y)/(cv::norm(pt2)*cv::norm(p1))));
}

//! Sorting function. Sorts the features belonging to the pyramid by their angle.
bool sortAnglePyr(cv::Point2f kpt1, cv::Point2f kpt2){
    cv::Point2f pt1 = kpt1-p2, pt2 = kpt2-p2;
    float det1 = pt1.x*p1.y-(pt1.y*p1.x), det2 = pt2.x*p1.y-(pt2.y*p1.x);
    return ( (det1>0?1:-1) * acos((pt1.x*p1.x+pt1.y*p1.y)/(cv::norm(pt1)*cv::norm(p1))) < (det2>0?1:-1) * acos((pt2.x*p1.x+pt2.y*p1.y)/(cv::norm(pt2)*cv::norm(p1))));
}

//! Sorting function. Helps to find the top of the pyramid
bool findOrigin(const cv::Point2f& pt1, const cv::Point2f& pt2){
    float min = 1000;
    bool minIdx=true;
    for(unsigned int i=0;i<ccenters.size();i++){
        if(cv::norm(pt1-ccenters[i]) < min){
         min = cv::norm(pt1-ccenters[i]);
         minIdx =true;
        }
        if(cv::norm(pt2-ccenters[i]) < min){
         min = cv::norm(pt2-ccenters[i]);
         minIdx =false;
        }
    }
    return !minIdx;
}

void calcPatternPosition(std::vector<cv::Point3f>& corners,CalibParams& params)
{
    corners.clear();

    switch(params.calib_pattern)
    {
        case Pattern::CHESSBOARD:
        case Pattern::CIRCLES_GRID:
          for( int i = 0; i < params.board_sz.height; ++i )
            for( int j = 0; j < params.board_sz.width; ++j )
                corners.push_back(cv::Point3f(float( j*params.element_size ), float( i*params.element_size ), 0));
          break;

        case Pattern::ASYMMETRIC_CIRCLES_GRID:
          for( int i = 0; i < params.board_sz.height; i++ )
             for( int j = 0; j < params.board_sz.width; j++ )
                corners.push_back(cv::Point3f(float((2*j + i % 2)*params.element_size), float(i*params.element_size), 0));
          break;

        case Pattern::PYRAMID:
            corners.push_back(cv::Point3f(0.450,0.240,0));
            corners.push_back(cv::Point3f(0.375,0.150,0));
            corners.push_back(cv::Point3f(0.300,0.240,0));
            corners.push_back(cv::Point3f(0.375,0.330,0));
            corners.push_back(cv::Point3f(0.525,0.150,0));
            corners.push_back(cv::Point3f(0.225,0.150,0));

            //bottom right
            corners.push_back(cv::Point3f(0.600,0.000,0));
            corners.push_back(cv::Point3f(0.750,0.000,0));
            corners.push_back(cv::Point3f(0.750,0.150,0));
            //top right
            corners.push_back(cv::Point3f(0.750,0.330,0));
            corners.push_back(cv::Point3f(0.750,0.480,0));
            corners.push_back(cv::Point3f(0.600,0.480,0));
            //top left
            corners.push_back(cv::Point3f(0.150,0.480,0));
            corners.push_back(cv::Point3f(0.000,0.480,0));
            corners.push_back(cv::Point3f(0.000,0.330,0));
            //bottom left
            corners.push_back(cv::Point3f(0.000,0.150,0));
            corners.push_back(cv::Point3f(0.000,0.000,0));
            corners.push_back(cv::Point3f(0.150,0.000,0));
            break;
    }
}

void drawPyramidPattern(cv::Mat& img, std::vector<cv::Point2f>& centers, bool found){


    cv::line(img, meanPT, meanPT+cv::Point2f(0,10), cv::Scalar(0,255,0), 1, CV_AA);
    cv::line(img, meanPT, meanPT+cv::Point2f(10,0), cv::Scalar(0,255,0), 1, CV_AA);
    cv::line(img, meanPT, meanPT+p1, cv::Scalar(255,255,255), 1, CV_AA);
    if(ccenters.size() > 0){
        cv::circle(img,ccenters[0],3,cv::Scalar(255,255,255));
        cv::circle(img,ccenters[1],3,cv::Scalar(255,255,255));
        cv::circle(img,ccenters[2],3,cv::Scalar(255,255,255));
        cv::circle(img,ccenters[3],3,cv::Scalar(255,255,255));
    }
    else
        std::cerr << "not enough clusters!" << std::endl;
    if(found){
        cv::line(img, centers[0], centers[3], cv::Scalar(0,0,255),2);
        cv::line(img, centers[0], centers[2], cv::Scalar(0,0,255),2);
        cv::line(img, centers[3], centers[2], cv::Scalar(0,0,255),2);
        cv::line(img, centers[1], centers[0], cv::Scalar(0,0,255),2);
        cv::line(img, centers[4], centers[1], cv::Scalar(0,0,255),2);
        cv::line(img, centers[4], centers[0], cv::Scalar(0,0,255),2);
        cv::line(img, centers[1], centers[2], cv::Scalar(0,0,255),2);
        cv::line(img, centers[5], centers[1], cv::Scalar(0,0,255),2);
        cv::line(img, centers[5], centers[2], cv::Scalar(0,0,255),2);

        cv::line(img, centers[6], centers[7], cv::Scalar(255,0,0),2);
        cv::line(img, centers[7], centers[8], cv::Scalar(255,0,0),2);
        cv::line(img, centers[6], centers[8], cv::Scalar(255,0,0),2);

        cv::line(img, centers[9], centers[10], cv::Scalar(0,255,0),2);
        cv::line(img, centers[10], centers[11], cv::Scalar(0,255,0),2);
        cv::line(img, centers[9], centers[11], cv::Scalar(0,255,0),2);

        cv::line(img, centers[12], centers[13], cv::Scalar(255,0,0),2);
        cv::line(img, centers[13], centers[14], cv::Scalar(255,0,0),2);
        cv::line(img, centers[12], centers[14], cv::Scalar(255,0,0),2);

        cv::line(img, centers[15], centers[16], cv::Scalar(0,255,0),2);
        cv::line(img, centers[16], centers[17], cv::Scalar(0,255,0),2);
        cv::line(img, centers[15], centers[17], cv::Scalar(0,255,0),2);

    }
        for( size_t i = 0; i < centers.size(); i++ ){
        cv::Scalar colour(i*10,0,255-(i*10));
        if(i<6)
            circle(img, centers[i], 1, colour, -1, 8, 0 );
        else
            circle(img, centers[i], 1, colour, -1, 8, 0 );

        std::stringstream ss; ss << i;
        putText(img,ss.str(),centers[i],0,0.5,cv::Scalar(0,255,0));
    }
}

bool findPyramid(const cv::Mat& img, std::vector<cv::Point2f>& centers){

    centers.clear();
    /**** Blob detection ****/
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 0;
    params.maxThreshold = 255;
    params.filterByColor = true;
    params.blobColor = 255;

    params.filterByArea = true;
    params.minArea = 0;
    params.maxArea = 50;
    params.filterByCircularity = true;
    params.minCircularity = 0.75;
    params.filterByConvexity = true;
    params.minConvexity = 0.87;
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.2;
    cv::Ptr<cv::SimpleBlobDetector> blobdet = cv::SimpleBlobDetector::create(params);

    std::vector<cv::KeyPoint> keypoints;
    blobdet->detect( img, keypoints);

    std::cout << keypoints.size() << " keypoints \r ";std::cout.flush();
    if(keypoints.size() != 18)
        return false;

    /**** finding center of the pattern ****/
     meanPT = cv::Point2f(0,0);

     for( unsigned int i = 0; i < keypoints.size(); i++ )
    {
         cv::Point2f center(keypoints[i].pt.x, keypoints[i].pt.y);
         centers.push_back(keypoints[i].pt);
         meanPT += center;
    }
    meanPT/=(int)(keypoints.size());

    /**** feature identification ****/

    // sorting feature by their distance to the center
    std::sort(centers.begin(),centers.end(),sortPt);

    // center position refinement with only features from the center of the pyramid
    meanPT = cv::Point2f(0,0);
    for(uint i=0;i<3;i++)
        meanPT += centers[i];
    meanPT/=3;
    // selecting corner features
    cornersP.clear();
    cornersP.insert(cornersP.end(),centers.begin()+6,centers.end());
    cv::Mat labels,ccenters_;
    //kmeans to cluster the 4 corners
    cv::kmeans(cornersP,4,labels,cv::TermCriteria(cv::TermCriteria::EPS,50,0.1),20,cv::KMEANS_RANDOM_CENTERS,ccenters_);
    ccenters.clear();
    //computing the center of each corner
    for(unsigned int k=0;k<4;k++)
        ccenters.push_back(cv::Point2f(ccenters_.at<float>(k,0),ccenters_.at<float>(k,1)));

    //finding the origin of the pyramid (top summit)
    std::sort(centers.begin()+3,centers.begin()+6,findOrigin);
    p1 = centers[3] - (centers[4]+centers[5])/2;
    p2 = (centers[4]+centers[5])/2 - p1;
    //sorting features by their angle
    sort(centers.begin(),centers.begin()+3,sortAnglePyr);
    sort(centers.begin()+4,centers.begin()+6,sortAnglePyr);
    sort(centers.begin()+6,centers.end(),sortAngle);

    /**** consistency check ****/

    bool corner1 = (norm(centers[6]-centers[7])+norm(centers[7]-centers[8])+norm(centers[8]-centers[6]))/3 < 130;
    bool corner2 = (norm(centers[9]-centers[10])+norm(centers[10]-centers[11])+norm(centers[11]-centers[9]))/3 < 130;
    bool corner3 = (norm(centers[12]-centers[13])+norm(centers[13]-centers[14])+norm(centers[14]-centers[12]))/3 < 130;
    bool corner4 = (norm(centers[15]-centers[16])+norm(centers[16]-centers[17])+norm(centers[17]-centers[15]))/3 < 130;
    bool ratio1 = norm(centers[3]-centers[0])/norm(centers[3]-centers[4]) > 0.45 && norm(centers[3]-centers[0])/norm(centers[3]-centers[4]) < 0.55;
    bool ratio2 = norm(centers[3]-centers[2])/norm(centers[3]-centers[5]) > 0.45 && norm(centers[3]-centers[2])/norm(centers[3]-centers[5]) < 0.55;
    bool ratio3 = norm(centers[2]-centers[0])/norm(centers[4]-centers[5]) > 0.45 && norm(centers[2]-centers[0])/norm(centers[4]-centers[5]) < 0.55;

    return ratio1 && ratio2 && ratio3 && corner1 && corner2 && corner3 && corner4;
}

std::vector<std::vector<cv::Point2f>> findPattern(const std::string& pathToImages, std::vector<int>& image_idx, CalibParams& params){

    cv::VideoCapture cap(pathToImages+"/"+params.cam_name);
	std::cout << "[Calibration] opening " << pathToImages+"/"+params.cam_name << std::endl;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    image_idx.clear();
    cv::namedWindow("calibration",cv::WINDOW_NORMAL);
    if(!cap.isOpened()){
        std::cerr << "could not open video or find images. exiting..." << std::endl;
        exit(-1);
    }else{
        params.image_size.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);params.image_size.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);}

    cv::Mat img, rimg;
    cap >> img;
    while(!img.empty()){

        /**** reading or acquiring the current frame ***/
        if(img.channels()>1){
            rimg = img.clone();
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        }else
            cv::cvtColor(img, rimg, cv::COLOR_GRAY2BGR);

        /**** detecting the calibration pattern ****/
        std::vector<cv::Point2f> corners;
        bool found;
        switch(params.calib_pattern){
            case Pattern::CHESSBOARD:
                found = cv::findChessboardCorners(rimg, params.board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);break;
            case Pattern::ASYMMETRIC_CIRCLES_GRID:
                found = cv::findCirclesGrid(rimg, params.board_sz, corners,cv::CALIB_CB_ASYMMETRIC_GRID);break;
            case Pattern::CIRCLES_GRID:
                found = cv::findCirclesGrid(rimg, params.board_sz, corners,cv::CALIB_CB_SYMMETRIC_GRID);break;
            case Pattern::PYRAMID:
                found = findPyramid(img, corners);break;
        }

        if(found){ // if the pattern could not been found, display the image and continue to the next frame
            /**** displaying the pattern ****/
            switch(params.calib_pattern){
                case Pattern::CHESSBOARD:
                    cv::drawChessboardCorners(rimg, params.board_sz,corners, found);break;
                case Pattern::ASYMMETRIC_CIRCLES_GRID:
                    cv::drawChessboardCorners(rimg, params.board_sz,corners, found);break;
                case Pattern::CIRCLES_GRID:
                    cv::drawChessboardCorners(rimg, params.board_sz,corners, found);break;
                case Pattern::PYRAMID:
                    drawPyramidPattern(rimg,corners, found);break;
            }

            imshow("calibration", rimg);
            char k;
            k = cv::waitKey(100);
            imagePoints.push_back(corners);
            image_idx.push_back(cap.get(cv::CAP_PROP_POS_FRAMES));
            if(k == 'c') // if c is pressed, stop the acquisition
                break;
        }else{
        //                cerr << "chessboard not found" << endl;
            cv::imshow("calibration", rimg);
            cv::waitKey(100);
        }


        for(int i=0;i<params.interval;i++)
            cap >> img;
    }
	cap.release();
    return imagePoints;
}

void findPatternStereo(const std::string& pathToImages, std::vector<std::vector<cv::Point2f>>& leftPoints, std::vector<std::vector<cv::Point2f>>& rightPoints, CalibParams& params){

    leftPoints.clear();rightPoints.clear();
    cv::namedWindow("cleft",cv::WINDOW_NORMAL);cv::namedWindow("cright",cv::WINDOW_NORMAL);
    cv::VideoCapture cap_left,cap_right;

    if(unsigned found = params.cam_name.find_last_of("X")){
        cap_left.open(pathToImages + "/" + params.cam_name.substr(0,found) + "0" + params.cam_name.substr(found+1,params.cam_name.size()));
        cap_right.open(pathToImages + "/" + params.cam_name.substr(0,found) + "1" + params.cam_name.substr(found+1,params.cam_name.size()));
        params.image_size.height = cap_left.get(cv::CAP_PROP_FRAME_HEIGHT);params.image_size.width = cap_left.get(cv::CAP_PROP_FRAME_WIDTH);
    }else{
        std::cerr << "[Calibration] could not identify variable X in camera name." << std::endl;
        exit(-1);
    }

    cv::Mat limg, rimg,climg,crimg;
    cap_left >> limg;
    cap_right >> rimg;
    while(!limg.empty() && !rimg.empty()){
        /**** reading images ****/
        if(limg.channels()>1){
            climg = limg.clone();
            cv::cvtColor(limg, limg, cv::COLOR_BGR2GRAY);
        }else
            cv::cvtColor(limg, climg, cv::COLOR_GRAY2BGR);
        if(rimg.channels()>1){
            crimg = rimg.clone();
            cv::cvtColor(rimg, rimg, cv::COLOR_BGR2GRAY);
        }else
            cv::cvtColor(rimg, crimg, cv::COLOR_GRAY2BGR);

        std::vector<cv::Point2f> lcorners,rcorners;
        bool lfound,rfound;
        /**** detecting the calibration pattern ****/
        switch(params.calib_pattern){
            case Pattern::CHESSBOARD:
                lfound = cv::findChessboardCorners(limg, params.board_sz, lcorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
                rfound = cv::findChessboardCorners(rimg, params.board_sz, rcorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);break;
            case Pattern::ASYMMETRIC_CIRCLES_GRID:
                lfound = cv::findCirclesGrid(limg, params.board_sz, lcorners,cv::CALIB_CB_ASYMMETRIC_GRID);
                rfound = cv::findCirclesGrid(rimg, params.board_sz, rcorners,cv::CALIB_CB_ASYMMETRIC_GRID);break;
            case Pattern::CIRCLES_GRID:
                lfound = cv::findCirclesGrid(limg, params.board_sz, lcorners,cv::CALIB_CB_SYMMETRIC_GRID);
                rfound = cv::findCirclesGrid(rimg, params.board_sz, rcorners,cv::CALIB_CB_SYMMETRIC_GRID);break;
            case Pattern::PYRAMID:
                lfound = findPyramid(limg, lcorners);
                rfound = findPyramid(rimg, rcorners);break;
        }

        if(lfound && rfound){ // if a calibration pattern is not detected, continue
            /**** displaying the calibration pattern ****/
            switch(params.calib_pattern){
                case Pattern::CHESSBOARD:
                    cv::drawChessboardCorners(climg, params.board_sz,lcorners, lfound);
                    cv::drawChessboardCorners(crimg, params.board_sz,rcorners, rfound);break;
                case Pattern::ASYMMETRIC_CIRCLES_GRID:
                    cv::drawChessboardCorners(climg, params.board_sz,lcorners, lfound);
                    cv::drawChessboardCorners(crimg, params.board_sz,rcorners, rfound);break;
                case Pattern::CIRCLES_GRID:
                    cv::drawChessboardCorners(climg, params.board_sz,lcorners, lfound);
                    cv::drawChessboardCorners(crimg, params.board_sz,rcorners, rfound);break;
                case Pattern::PYRAMID:
                    drawPyramidPattern(climg,lcorners, lfound);
                    drawPyramidPattern(crimg,rcorners, rfound);break;
            }

            imshow("cleft", climg);
            imshow("cright", crimg);
            char k;
            k = cv::waitKey(100);
            leftPoints.push_back(lcorners);
            rightPoints.push_back(rcorners);
            if(k == 'c') // if c is pressed, stop the detection
                break;
        }else{
            cv::imshow("cleft", climg);
            cv::imshow("cright", crimg);
            cv::waitKey(100);
        }

        for(int i=0;i<params.interval;i++){
            cap_left >> limg;
            cap_right >> rimg;
        }
    }
}

double computeMRE(const std::vector<std::vector<cv::Point3f>>& objectPoints, const std::vector<std::vector<cv::Point2f>>& imagePts, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const cv::Mat& K, const cv::Mat& dist, std::vector<double>& repValues){

    std::vector<cv::Point2f> reprojPts;
    repValues.clear();
    double mre=0;
    for(unsigned int p=0;p<objectPoints.size();p++){
        double mre_=0;
        projectPoints(objectPoints[p],rvecs[p],tvecs[p],K,dist,reprojPts);
        for(unsigned int q=0;q<reprojPts.size();q++)
            mre_ += sqrt(pow(imagePts[p][q].x-reprojPts[q].x,2)+pow(imagePts[p][q].y-reprojPts[q].y,2));
        mre_ /= reprojPts.size();
        repValues.push_back(mre_);
        mre += mre_;
    }
    mre /= objectPoints.size();
    return mre;
}
