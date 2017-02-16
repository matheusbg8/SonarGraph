#include <iostream>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Sonar.h"
#include "SonarConfig.h"
#include "GroundTruth/GroundTruth.h"
#include "GraphMatcher/GraphMatcher.h"
#include "MatchViewer.h"

using namespace std;
using namespace cv;


//// Edge Matching Test
//int main(int argc, char* argv[])
//{
//    GraphMatcher gm;

//    vector<GraphLink *> u,v;

//    u.push_back(new GraphLink(0,0,1,10,1));
//    u.push_back(new GraphLink(0,0,1,50,2));
//    u.push_back(new GraphLink(0,0,1,30,3));
//    u.push_back(new GraphLink(0,0,1,10,4));
//    u.push_back(new GraphLink(0,0,1,20,5));

//    v.push_back(new GraphLink(0,0,1,10,1));
//    v.push_back(new GraphLink(0,0,1,11,2));
//    v.push_back(new GraphLink(0,0,1,30,3));
//    v.push_back(new GraphLink(0,0,1,50,4));
//    v.push_back(new GraphLink(0,0,1,15,5));
//    v.push_back(new GraphLink(0,0,1,20,6));

//    gm.byDistanceCompare(u,v);

//    return 0;
//}


int main(int argc, char* argv[])
{
    MatchViewer mv;

    cout << "OpenCV version " << CV_VERSION << endl;

//    mv.compareWithGroundTruth();

    mv.createGroudTruth();

//    mv.standardExecution();

//    mv.matchTest();

//    mv.gaussianTest();

    cout << "End of main" << endl;

    return 0;
}

//int main(int argc, char* argv[])
//{
//    Mat img = imread(argv[1],CV_LOAD_IMAGE_ANYDEPTH);
//    img.convertTo(img,CV_8UC1);
//    imshow("img", img);
//    waitKey();
//    return 0;
//}
