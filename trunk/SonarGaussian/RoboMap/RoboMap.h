#ifndef ROBOMAP_H
#define ROBOMAP_H

#include "Sonar/SonarConfig/ConfigLoader.h"

#include<string>
#include<map>
#include<cstdio>
#include <opencv2/core/core.hpp>

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

    //  === GTLoop Detection =====
    float sonarRange;  // in meters
    float minDisplacement;  // in meters
    float distBetweenCenters; // in meters
    float distBetweenPositions; // in meters
    //  ============================

    RoboMap(const string &mapName);
    ~RoboMap();

    // Map methods
    void loadMap(const string & mapName);
    Point2f UTM2Img(const Point2f &UTMp);

    void parse2D_2_TF2Vector3(const string &str, Point2f &p);

    // New state methods
    void newSonarState(const SonarState &sonarSate);
    void clearSonarStates();

    void newLabel(const RoboMapLabel &label);
    void clearLabels();

    void showScreen();
};

#endif // ROBOMAP_H
