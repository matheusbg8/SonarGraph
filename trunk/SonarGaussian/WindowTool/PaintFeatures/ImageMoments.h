#include "PaintFeature.h"

#ifndef IMAGEMOMENTS_H
#define IMAGEMOMENTS_H

class ImageMoments : public PaintFeature
{
public:
    ImageMoments();

    // PaintFeature interface
public:
    void mouseDrag(int x, int y);
    void mouseDragEnd(int x, int y);
};

#endif // IMAGEMOMENTS_H
