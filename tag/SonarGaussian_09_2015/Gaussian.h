#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Segment.h"

using namespace std;
using namespace cv;

class Gaussian
{
public:
    Gaussian(){}
    Gaussian(float x, float y , float z,
             float dx, float dy, float dz, float ang, unsigned N):
        x(x),y(y),z(z),dx(dx),dy(dy),dz(dz),ang(ang),N(N){}

    void createGaussian(Segment *seg);

    float x, y , z;
    float dx, dy, dz, ang;
    unsigned N;

};

#endif // GAUSSIAN_H
