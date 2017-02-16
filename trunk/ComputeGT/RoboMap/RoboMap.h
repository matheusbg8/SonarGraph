#ifndef ROBOMAP_H
#define ROBOMAP_H

#include <string>
#include <map>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include "CGALDef.h"

using namespace cv;
using namespace std;

class SonarState
{
public:
    Point2f p,c;
    float heading, bearing, range;
    bool hasCenter, hasColor;
    Scalar color;

    SonarState(const Point2f &p= Point2f(0.f,0.f),
               float heading=0.f, float bearing=130.f, float range=30.f):
               p(p),c(Point2f(0.f,0.f)),color(color),
               heading(heading),bearing(bearing), range(range)
    {
        hasCenter = false;
        hasColor = false;
    }

    SonarState(const Point2f &p, const Point2f &c,
               float heading=0.f, float bearing=130.f, float range=30.f):
               p(p),c(c),heading(heading),color(color),
               bearing(bearing), range(range)
    {
        hasCenter = true;
        hasColor = false;
    }

    SonarState(const Point2f &p, const Point2f &c,
               float heading, const Scalar & color,
               float bearing=130.f, float range=30.f):
        p(p),c(c),heading(heading),color(color),
        bearing(bearing), range(range)
    {
        hasCenter = true;
        hasColor = true;
    }

};

class RoboMapLabel
{public:
    string str;
    Point2f p;
    Scalar color;
    RoboMapLabel(const string &str, const Point2f &p, const Scalar &color):
        str(str), p(p),color(color){}
};

class RoboPoly
{public:
    Point *poly_;
    unsigned nPts;
    Scalar color;
    RoboPoly(const Point2f *poly, unsigned nPts, const Scalar &color):
        nPts(nPts),color(color)
    {
        poly_ = new Point[nPts];
        for(unsigned i = 0 ; i < nPts; i++)
            poly_[i] = poly[i];
    }
    RoboPoly(const Polygon_2 &poly, const Scalar &color):
        nPts(nPts),color(color)
    {
        poly_ = new Point[poly.size()];
        for(unsigned i = 0 ; i < nPts; i++)
        {
            const PointCGAL &p = poly[i];
            poly_[i] = Point(CGAL::to_double(p.x()),CGAL::to_double(p.y()));
        }
    }
    RoboPoly(const RoboPoly &poly):
        nPts(poly.nPts),color(poly.color)
    {
        poly_ = new Point[nPts];
        for(unsigned i = 0 ; i < nPts; i++)
            poly_[i] = poly.poly_[i];
    }
    ~RoboPoly()
    {
        delete [] poly_;
    }
    const RoboPoly& operator =(const RoboPoly &poly)
    {
        delete [] poly_;
        nPts=poly.nPts;
        color = poly.color;
        for(unsigned i = 0 ; i < nPts; i++)
            poly_[i] = poly.poly_[i];
    }
};

class RoboMap
{
public:

    // ==== RoboMap config ======
    bool firstPosition;

    //  ============================

    // ====== Map conversions =====
    Point2f imgRef[2],
        UTMRef[2],
        UTM2Img_;

    Mat mapImg;
    Mat screen;
    //  ============================
    vector<SonarState> sonarStates;
    vector<RoboMapLabel> labels;
    vector<RoboPoly> polys;

    //  === GTLoop Detection =====
    float sonarRange;  // in meters
    float minDisplacement;  // in meters
    float distBetweenCenters; // in meters
    float distBetweenPositions; // in meters
    //  ============================

    static const Scalar color[6];
    static const unsigned nColor;

    static void getSonarPoly(const Point2f &p, float heading, Point2f *poly,
                     unsigned nPoint=13, float range=30, float bearing=130);

    static void getSonarCGALPoly(const Point2f &p, float heading, Polygon_2 &poly,
                     unsigned nPoint=13, float range=30, float bearing=130);

    void getSonarPolyOnImg(const Point2f &p, float heading, Point *poly,
                     unsigned nPoint=13, float range=30, float bearing=130);

    RoboMap(const string &mapName);
    ~RoboMap();

    // Map methods
    void loadMap(const string & mapName);
    Point2f UTM2Img(const Point2f &UTMp);
    void UTM2Img(Point2f *UTMpts, unsigned nPts);

    void parse2D_2_TF2Vector3(const string &str, Point2f &p);

    // New state methods
    void newSonarState(const SonarState &sonarSate);
    void clearSonarStates();

    void newLabel(const RoboMapLabel &label);
    void clearLabels();

    void newPoly(const RoboPoly &poly);
    void clearPolys();

    void clear();

    void showScreen();
};

#endif // ROBOMAP_H
