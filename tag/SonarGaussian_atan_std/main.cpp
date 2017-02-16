
#include <iostream>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Sonar.h"
#include "SonarConfig.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    char img_file_name[200];
    FILE *f_img_names = fopen("imgs.txt","r");

//    cout << -90+180.0*atan2(1,-1)/M_PI << endl;
//    cout << -90+180.0*atan2(1,1)/M_PI << endl;
//    cout << -90+180.0*atan2(-1,1)/M_PI << endl;
//    cout << -90+180.0*atan2(-1,-1)/M_PI << endl;
//    cout << endl;
//    cout << 180.0*atan2(1,1)/M_PI << endl;
//    cout << 180.0*atan2(-1,1)/M_PI << endl;
//    cout << 180.0*atan2(-1,-1)/M_PI << endl;
//    cout << 180.0*atan2( 1,-1)/M_PI << endl;

    SonarConfig sc;
    sc.load("../SonarGaussian_atan_std/MachadosConfig");
    Sonar s(sc);

    int frame = 0;
    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        cout << "Frame " << frame++ << endl;
        Mat img = imread(img_file_name,CV_LOAD_IMAGE_ANYDEPTH);
        s.newImage(img);
        waitKey();
    }
    return 0;
}

