#ifndef MATCHTEST_H
#define MATCHTEST_H

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SonarDescritor.h"

#include<string>

using namespace std;
using namespace cv;

class MatchTest
{
public:
    SonarDescritor sd1, sd2;
    Size winSz;
    string winName;

    MatchTest();

    void mainLoop();
    void start();

    void addEdge(SonarDescritor *sd, int x, int y);
    void draw();

    void mouseEvent(int event, int x, int y);
};

void onMatchTestMouseEvent(int event, int x, int y, int, void *dt);

#endif // MATCHTEST_H
