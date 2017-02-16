#include "Frame.h"
#include <cmath>

#include <iostream>
using namespace std;

Gaussian *Frame::findGaussian(int x, int y, int *id)
{
    Gaussian *gr=0x0;
    *id = -1;
    float minDist=FLT_MAX;

    for(unsigned g = 0 ; g < gaussians.size(); g++)
    {
        float dx = gaussians[g].x - x,
              dy = gaussians[g].y - y,
              dist = sqrt(dx*dx + dy*dy);

//        cout << "Mouse Dist " << dist << " to g" << g
//             << " max " << max(gaussians[g].dx,gaussians[g].dy)
//             << endl;
        if(max(gaussians[g].dx,gaussians[g].dy) >= dist
           && minDist > dist )
        {

            gr = &gaussians[g];
            *id = g;
            minDist = dist;
        }
    }
    return gr;
}

void Frame::removeMatch(unsigned srcGauss)
{
    cout << "Clearing matchs of gaussian " << srcGauss << endl;
    match[srcGauss].clear();
}

void Frame::removeAllMatch()
{
    cout << "Clearing all matchs of frame" << endl;
    for(unsigned i = 0 ; i < match.size() ; i++)
        match[i].clear();
}
