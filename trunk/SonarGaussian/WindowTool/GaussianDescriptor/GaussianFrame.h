#ifndef GAUSSIANFRAME_H
#define GAUSSIANFRAME_H

#include "WindowTool/Frame.h"
#include "Sonar/Gaussian.h"

#include <iostream>
#include <vector>

class GaussianFrame
{
public:

    GaussianFrame();

    vector<Gaussian> gaussians;

    void clear();

};

#endif // GAUSSIANFRAME_H
