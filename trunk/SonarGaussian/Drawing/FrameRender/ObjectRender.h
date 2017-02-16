#ifndef OBJECTRENDE_H
#define OBJECTRENDE_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace std;
using namespace cv;

/**
 * @brief This is ObjectRender interface and same basic implementation
 * @todo - todo!
 */
class ObjectRender
{
    bool _visible;
public:
    ObjectRender();
    virtual ~ObjectRender();
    virtual void render(Mat &img) = 0;

    void setVisible(bool visible);
    bool isVisible();
};

#endif // OBJECTRENDE_H
