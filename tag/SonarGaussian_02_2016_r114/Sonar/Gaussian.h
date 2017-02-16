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
    Gaussian();

    Gaussian(float x, float y , float intensity,
             float dx, float dy, float di, float ang, unsigned N);

    Gaussian(Segment *seg, float std=1);

    float x,   /**< x stores x (column) mean of segment */
          y,   /**< y stores y (row) mean of segment */
     intensity;  /**< intensity stores intensity mean of segment */

    float dx,  /**< dx stores the length (standard deviation) of the axis with the second greatest axis of dispersion */
          dy,  /**< dy stores the length (standard deviation) of the axis with the greatest dispersion */
          di,  /**< di store the std intensoty of segment*/
         ang;  /**< ang stores the slope of the axis with the greatest dispersion in degree.*/

    unsigned N;  /**< N store the amount of pixel of the segment represented*/

    void createGaussian3x3(Segment *seg, float std=1.f);
    void createGaussian2x2(Segment *seg, float std=1.f);

    void clockWisePoints(Point2f &a,Point2f &b,Point2f &c,Point2f &d) const;

    void merge(const Gaussian &g, float std);

    static bool hasIntersection(const Gaussian &a, const Gaussian &b);

    // On a closest future, when Gaussian atributes will be private
    // this friend delcaration will be make more sence.
    friend ostream & operator <<(ostream &os, const Gaussian& g);
};

ostream & operator <<(ostream &os, const Gaussian& g);

#endif // GAUSSIAN_H
