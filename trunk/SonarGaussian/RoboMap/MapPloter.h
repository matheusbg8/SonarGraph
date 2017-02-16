#ifndef MAPPLOTER_H
#define MAPPLOTER_H

#include <opencv2/core/core.hpp>
#include <string>

using namespace cv;
using namespace std;

class MapPloter
{
private:
    Point2f imgRef[2],
        UTMRef[2],
        UTM2Img_;

    Mat mapImg;
    Mat screen;

    Mat colors;

public:
    MapPloter();

    // Map methods
    void loadMap(const string & mapName);
    Point2f UTM2Img(const Point2f &UTMp);
    void parse2DPoint(const string &str, Point2f &p);

    void drawLine(const Point2f &p1,const Point2f &p2, const Scalar &color);
    void drawLine(const Point2f &p1,const Point2f &p2, float colorPercent);
    void clearImg();

    void showImg();
    void saveImg(const string &imagName);

    void resetColors(int colorMap);
};

#endif // MAPPLOTER_H
