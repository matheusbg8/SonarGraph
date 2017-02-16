#ifndef GAUSSIANTEST_H
#define GAUSSIANTEST_H


#include <iostream>
#include <vector>
#include <string>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#include "Segmentation/Segment.h"
#include "Gaussian.h"

class GaussianTest
{
public:

    string winName;
    Size winSize;

    Segment sample;
    Gaussian gaussian;
    Mat img;

    GaussianTest();

    void mainLoop();

    void draw();

    void start();

    void clear();

    void mouseEvent(int event, int x, int y);
};

void onGausianTestMouseEvent(int event, int x, int y, int, void* dt);

#endif // GAUSSIANTEST_H
