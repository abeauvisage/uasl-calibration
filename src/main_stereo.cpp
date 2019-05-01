#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <Windows.h>
#endif

#include <sys/stat.h>
#include "wingetopt.h"
#include "dirent.h"

#include "calib_stereo.h"

using namespace cv;
using namespace std;

/**** example ****

lab_stereo_calibration [IMAGEPATH] [YMLPATH] -[OPTIONS]

./bin/lab_stereo_calibration pathToImageFolder cam1_image%05d.png pathToYMLFile.yml       | using a sequence of images
./bin/lab_stereo_calibration pathToImageFolder calibration_vid.mp4 pathToYMLFile.yml      | using a video
./bin/lab_stereo_calibration /dev video0 pathToYMLFile.yml                                | using the camera

******************/

static void usage()
{
    cout << "lab_stereo_calibration:" << endl << "Supported options:" << endl;
    cout << "   -H,-h  display this help." << endl;
    cout << "   -n     specify the minimum number of images to stop the calibration." << endl;
    cout << "   -m     specify the MRE threshold to stop the calibration." << endl;
    cout << "   -e     set the size of the elements of the chessboard." << endl;
	cout << "   -d     display rectified images, otherwise save them." << endl;
	cout << "   -c     calibrate and rectify with the provided images." << endl;
	cout << "   -i     interval rate at which images are processed." << endl;
}

int main(int argc, char** argv){

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#else
	const mode_t DEFAULT_MKDIR_PERMISSIONS = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
#endif

	int c;
	bool calibration=false;
	CalibParams params;
	string pathToImages=argv[1],rectFolder="",ymlPath=argv[3];
	params.cam_name=argv[2];

	while ((c = getopt(argc, argv, "n:i:m:p:e:h:cdHh")) != -1)
	{
		switch (c)
		{
        case 'n':
            params.MIN_IMAGES = stof(optarg);
            break;
        case 'i':
            params.interval = stof(optarg);
            break;
        case 'm':
            params.MAX_MRE = stof(optarg);
            break;
        case 'p':
            if(string(optarg)=="chessboard")
                params.calib_pattern = Pattern::CHESSBOARD;
            if(string(optarg)=="pyramid")
                params.calib_pattern = Pattern::PYRAMID;
            if(string(optarg)=="circles")
                params.calib_pattern = Pattern::CIRCLES_GRID;
            if(string(optarg)=="acircles")
                params.calib_pattern = Pattern::ASYMMETRIC_CIRCLES_GRID;
            break;
        case 'e':
            params.element_size = stof(optarg);
            break;
        case 'h':
		case 'H':
			usage();
			return 1;
        case 'd':
            params.display=true;
            break;
        case 'c':
            calibration=true;
            break;
		default:
			std::cout << "Invalid option -%c" << endl;
			usage();
			return 1;
		}
	}

	if(pathToImages==""){
        cerr << "[error] no image directory provided." << endl;
        usage();
        exit(-1);
	}

	if(params.cam_name==""){
        cerr << "[error] no image expression provided." << endl;
        usage();
        exit(-1);
	}

    if(ymlPath==""){
        cerr << "[error] no params file provided." << endl;
        usage();
        exit(-1);
	}

    DIR *d = NULL;
    d = opendir(pathToImages.c_str());
	if (d == NULL)
	{
		std::cerr << "[error] " << pathToImages << " is not a valid directory" << std::endl;
		exit(-1);
	}else
        std::cout << "[init] directory found: " << pathToImages << std::endl;

    /**** Creating the new folder containing the rectified images ****/

	unsigned found = pathToImages.find_last_of("/");
	if(found == pathToImages.size()-1)
        pathToImages = pathToImages.substr(0,found);
    rectFolder = pathToImages+"_rec/";

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        CreateDirectory(rectFolder, NULL);
    #else
        mkdir(rectFolder.c_str(), DEFAULT_MKDIR_PERMISSIONS);
    #endif

    /***************************************************************/

    if(calibration)
        stereo_calibrateAndRectify(pathToImages,rectFolder,ymlPath,params);
	else
        stereo_rectify(pathToImages,rectFolder,ymlPath,params);

    return 0;
}
