#include "Tools/RoboMap/RoboMapFileGTLoopGenerator.h"

//opencv
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Transform.h>

#include <tf/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>


// Boost
#include <boost/foreach.hpp>

// STD
#include<string>
#include<ctime>
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace cv;

RoboMapFileGTLoopGenerator::RoboMapFileGTLoopGenerator(const string &similarFileName, const string &matchFileName)
{
    fSimilar = fopen(similarFileName.c_str(),"w");
    fMatch = fopen(matchFileName.c_str(),"w");

    // print header
    fprintf(fSimilar , "Img source ID, Img destin ID, Match Score, Timestamp\n");
    fprintf(fMatch , "Img source ID, Img destin ID, Match Score, Timestamp\n");
}

RoboMapFileGTLoopGenerator::~RoboMapFileGTLoopGenerator()
{
    if(fSimilar != 0x0)
        fclose(fSimilar);

    if(fMatch != 0x0)
        fclose(fMatch);

    cout << "RoboMapFileGTLoopGenerator:: finished..." << endl;
}

void RoboMapFileGTLoopGenerator::generateGTLoop(unsigned srcId, double time,
                                                const vector<pair<unsigned, float> > &similaRegion,
                                                const vector<pair<unsigned, float> > &match)
{
    for(unsigned i = 0 ; i < match.size() ; i++)
    {
        fprintf(fMatch , "%u,%u,%g\n",srcId,
                                match[i].first,
                                match[i].second);
    }

    for(unsigned i = 0; i < similaRegion.size();i++)
    {
        fprintf(fSimilar, "%u,%u,%g\n",srcId,
                similaRegion[i].first,
                similaRegion[i].second);
    }
}
