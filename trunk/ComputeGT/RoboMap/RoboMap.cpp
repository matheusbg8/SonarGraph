#include "RoboMap.h"

//opencv
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>

// STD
#include<string>
#include<ctime>
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace cv;

#include "ConfigLoader/ConfigLoader.h"

const Scalar RoboMap::color[6] =
{
    Scalar(255 ,   0 ,   0),
    Scalar(  0 , 255 ,   0),
    Scalar(  0 ,   0 , 255),
    Scalar(255 , 255 ,   0),
    Scalar(255 ,   0 , 255),
    Scalar(  0 , 255 , 255)
};

const unsigned RoboMap::nColor =6;

void RoboMap::getSonarPoly(const Point2f &p, float heading, Point2f *poly,
                           unsigned nPoint, float range, float bearing)
{
    if(nPoint < 3) return;

    double rad = heading* M_PI/180.f,
          radBearing = bearing* M_PI/180.f,
          radInc = radBearing/(nPoint-2);

    unsigned pId = 0;

    // Sonar origen
    poly[pId++] = p;

    for(double currentRad = rad + radBearing/2.f;
        pId < nPoint; currentRad-= radInc)
    {
        poly[pId++] = p + Point2f(sin(currentRad), cos(currentRad))*range;
    }
}

void RoboMap::getSonarCGALPoly(const Point2f &p, float heading, Polygon_2 &poly,
                               unsigned nPoint, float range, float bearing)
{
    if(nPoint < 3) return;

    double rad = heading* M_PI/180.f,
          radBearing = bearing* M_PI/180.f,
          radInc = radBearing/(nPoint-2);

    unsigned pId = 0;
    poly.clear();

    // Sonar origen
    poly.push_back(PointCGAL(p.x,p.y));
    pId++;

    for(double currentRad = rad + radBearing/2.f;
        pId < nPoint; currentRad-= radInc)
    {
        const Point2f &pr = p + Point2f(sin(currentRad), cos(currentRad))*range;
        poly.push_back( PointCGAL(pr.x,pr.y));
        pId++;
    }
}

void RoboMap::getSonarPolyOnImg(const Point2f &p, float heading, Point *poly,
                                unsigned nPoint, float range, float bearing)
{
    if(nPoint < 3) return;

    double rad = heading* M_PI/180.f,
          radBearing = bearing* M_PI/180.f,
          radInc = radBearing/(nPoint-2);

    unsigned pId = 0;

    // Sonar origen
    poly[pId++] = UTM2Img(p);

    for(double currentRad = rad + radBearing/2.f;
        pId < nPoint; currentRad-= radInc)
    {
        poly[pId++] = UTM2Img(p + Point2f(sin(currentRad), cos(currentRad))*range);
    }
}

RoboMap::RoboMap(const string &mapName)
{
    cout << "Setuping RoboMap" << endl;

    loadMap(mapName);

    sonarRange = 30.f;
    minDisplacement = 0.11f;
    distBetweenCenters = 3.f;
    distBetweenPositions = 1.f;

    // Create windows
    namedWindow("RoboMap",WINDOW_NORMAL);

}

RoboMap::~RoboMap()
{
    destroyWindow("RoboMap");

    cout << "RoboMap deleted" << endl;
}

void RoboMap::loadMap(const string &mapName)
{
    ConfigLoader config(mapName.c_str());

    string str;

    if(config.getString("General", "MapName", &str))
    {
        cout << "MapName " << str << endl;
        mapImg = imread(str);

        if(mapImg.rows == 0)
        {
            cout << "Error when loading map " << str << endl;
        }
        assert("Problem!!");
    }

    if(config.getString("P_1", "img_x_y", &str))
    {
        cout << "P_1: img_x_y " << str << endl;
        parse2D_2_TF2Vector3(str,imgRef[0]);
    }

    if(config.getString("P_1", "utm_x_y", &str))
    {
        cout << "P_1: utm_x_y " << str << endl;
        parse2D_2_TF2Vector3(str,UTMRef[0]);
    }

    if(config.getString("P_2", "img_x_y", &str))
    {
        cout << "P_2: img_x_y " << str << endl;
        parse2D_2_TF2Vector3(str,imgRef[1]);
    }

    if(config.getString("P_2", "utm_x_y", &str))
    {
        cout << "P_2: utm_x_y " << str << endl;
        parse2D_2_TF2Vector3(str,UTMRef[1]);
    }

    Point2f diffImg = imgRef[1] - imgRef[0],
            diffUTM = UTMRef[1] - UTMRef[0];

    // Compute conversion for small and initial square
    UTM2Img_.x = diffImg.x/diffUTM.x;
    UTM2Img_.y = diffImg.y/diffUTM.y;

    printf("imgRef1 %f %f\n", imgRef[0].x , imgRef[0].y);
    printf("imgRef2 %f %f\n", imgRef[1].x , imgRef[1].y);
    printf("utmRef1 %f %f\n", UTMRef[0].x , UTMRef[0].y);
    printf("utmRef2 %f %f\n", UTMRef[1].x , UTMRef[1].y);
    printf("utmDiff %f %f\n", diffUTM.x , diffUTM.y);
    printf("imgDiff %f %f\n", diffImg.x , diffImg.y);

    printf("utm2img %f %f\n", UTM2Img_.x , UTM2Img_.y);

    cout << "Map loaded!" << endl;
}


void RoboMap::parse2D_2_TF2Vector3(const string &str, Point2f &p)
{
    double x,y;
    sscanf(str.c_str(),"%lf %lf", &x, &y);
    p.x =x;
    p.y =y;
}

Point2f RoboMap::UTM2Img(const Point2f &UTMp)
{
    Point2f p(imgRef[0].x + (UTMp.x - UTMRef[0].x)*UTM2Img_.x,
              imgRef[0].y + (UTMp.y - UTMRef[0].y)*UTM2Img_.y);
    return p;
}

void RoboMap::UTM2Img(Point2f *UTMpts, unsigned nPts)
{
    for(unsigned i = 0 ; i < nPts ; i++)
    {
        Point2f &p = UTMpts[i];
        p.x = imgRef[0].x + (p.x - UTMRef[0].x)*UTM2Img_.x;
        p.y = imgRef[0].y + (p.y - UTMRef[0].y)*UTM2Img_.y;
    }
}

void RoboMap::newSonarState(const SonarState &sonarSate)
{
    sonarStates.push_back(sonarSate);
}

void RoboMap::clearSonarStates()
{
    sonarStates.clear();
}

void RoboMap::newLabel(const RoboMapLabel &label)
{
    labels.push_back(label);
}

void RoboMap::clearLabels()
{
    labels.clear();
}

void RoboMap::newPoly(const RoboPoly &poly)
{
    polys.push_back(poly);
}

void RoboMap::clearPolys()
{
    polys.clear();
}

void RoboMap::clear()
{
    sonarStates.clear();
    labels.clear();
    polys.clear();
}

void RoboMap::showScreen()
{
    // Screen update
    Mat screen = mapImg.clone();

    // Drawing sonar states
    for(unsigned i = 0; i < sonarStates.size(); i++)
    {
        const SonarState& sonar = sonarStates[i];

        Point ps[8];

        getSonarPolyOnImg(sonar.p,sonar.heading, ps,8,30.f,130.f);

        int npts[] = {8};
        const Point* ppt[1] = { ps };
        const Scalar &color = sonar.hasColor? sonar.color : color[i%nColor];

        polylines(screen, ppt, npts, 1,true, color,8);

        circle(screen,UTM2Img(sonar.p),50,color,-1);

        if(sonar.hasCenter)
        {
            circle(screen,
                   UTM2Img(sonar.c),
                   50,color,-1);
        }
    }

    // Drawing labels
    for(unsigned i = 0; i < labels.size(); i++)
    {
        const RoboMapLabel& label = labels[i];

        putText(screen,label.str,
               label.p,
               FONT_HERSHEY_COMPLEX,5,
               label.color,5);
    }

    // Drawing polys
    for(unsigned i = 0; i < polys.size(); i++)
    {
        const RoboPoly& poly = polys[i];

        int npts[] = {poly.nPts};
        const Point* ppt[1] = { poly.poly_ };

        polylines(screen, ppt, npts, 1,true, poly.color,8);
    }

//    resize(screen,screen,Size(screen.cols*0.2,screen.rows*0.2));
//    resize(screen,screen,Size(screen.cols*0.35,screen.rows*0.35));

    imshow("RoboMap",screen);
    waitKey(1);
}

