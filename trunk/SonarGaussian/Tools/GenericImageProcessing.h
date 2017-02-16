#ifndef GENERICIMAGEPROCESSING_H
#define GENERICIMAGEPROCESSING_H

#include <string>
#include <list>
#include <vector>
#include <cstdio>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace cv;
using namespace std;

class GenericImageProcessing
{
private:
    string dataPath;
    vector<string> imgsName;

public:
    GenericImageProcessing(const string &path);

    bool loadFrames();

    unsigned int computeImageDiff(Mat &img1, Mat &img2);

    void generateAllImgDiff();

};

#endif // GENERICIMAGEPROCESSING_H
