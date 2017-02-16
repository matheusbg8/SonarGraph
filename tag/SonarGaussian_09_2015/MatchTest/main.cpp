#include <iostream>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SonarDescritor.h"

using namespace std;
using namespace cv;

SonarDescritor sd1, sd2;
Size winSz(800,600);
char winName[] = "wn";

void addEdge(SonarDescritor *sd, int x, int y)
{
    sd->gaussians.push_back(
                Gaussian(x,y,0,1,1,1,0,1)
                           );

    sd->graph.push_back(vector<GraphLink*>());
    sd->clearLinks();
    sd->createGraph(max(winSz.width,winSz.height));
}

void draw()
{
    Mat img(winSz.width, winSz.height, CV_8UC3, Scalar(0,0,0));
    SonarDescritor::drawDescriptorColor(img,&sd1,Scalar(0,255,255),Scalar(0,0,0));
    SonarDescritor::drawDescriptorColor(img,&sd2,Scalar(0,255,255),Scalar(0,0,0));

    imshow(winName,img);
}

void onMouse(int event, int x, int y, int, void *dt)
{
    if(x < winSz.width/2)
    {
        addEdge(&sd1, x,y);
    }else
    {
        addEdge(&sd2, x,y);
    }
}

int main(int argc, char* argv[])
{
    namedWindow(winName, 1);

    setMouseCallback(winName, onMouse);


    sd1.gaussians.push_back(
                Gaussian(winSz.width*0.25,winSz.height*0.5,0,1,1,1,0,1)
                );
    sd1.graph.push_back( vector<GraphLink*>());


    sd2.gaussians.push_back(
                Gaussian(winSz.width*0.75,winSz.height*0.5,0,1,1,1,0,1)
                );
    sd2.graph.push_back( vector<GraphLink*>());

    char c=0;
    while(c != 27)
    {


        c = waitKey();
    }

    return 0;
}
