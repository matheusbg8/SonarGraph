#ifndef GAUSSIANFRAMERENDER_H
#define GAUSSIANFRAMERENDER_H

#include "Sonar/Gaussian.h"
#include "GaussianRender.h"


class GaussianFrameRender
{   
public:
    vector<GaussianRender> rgs;

public:
    GaussianFrameRender();

    void setGassians(vector<Gaussian> &gs);
    GaussianRender &getGaussianRender(unsigned id);
    void render(Mat &img);

};

#endif // GAUSSIANFRAMERENDER_H
