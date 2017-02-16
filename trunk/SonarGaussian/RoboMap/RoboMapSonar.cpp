#include "RoboMapSonar.h"
#include "Drawing/Drawing.h"
#include "RoboMap/RoboMap.h"

RoboMapSonar::RoboMapSonar(const Point2f &p,
                           float heading, float bearing, float range):
    p(p),heading(heading),bearing(bearing), range(range)
{
}

void RoboMapSonar::render(Mat &img)
{
    float rad = heading* M_PI/180.f,
          radBearing = bearing* M_PI/180.f,
          radInc = radBearing/5;

    unsigned pId = 0;

    Point ps[6];

    // Sonar origen
    ps[pId++] = roboMap->UTM2Img(p);

    for(float currentRad = rad - radBearing/2.f;
        pId < 6; currentRad+= radInc)
    {
        ps[pId++] = roboMap->UTM2Img(p + Point2f(sin(currentRad), cos(currentRad))*range);
    }

    int npts[] = {6};
    const Point* ppt[1] = { ps };

    polylines(img, ppt, npts, 1,true, Drawing::color[0],8);
}
