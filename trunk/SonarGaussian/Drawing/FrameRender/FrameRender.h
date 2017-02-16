#ifndef FRAMERENDER_H
#define FRAMERENDER_H

#include <vector>
#include "Sonar/Gaussian.h"
#include "Drawing/FrameRender/ObjectRender.h"

using namespace std;

class FrameRender
{
    vector<ObjectRender*> objects;
public:
    FrameRender();
    void clear();

};

#endif // FRAMERENDER_H
