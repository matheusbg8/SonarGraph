#include "Frame.h"
#include <cmath>

#include <iostream>
using namespace std;

Frame::~Frame()
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame:Destructor:: using destructor!"
             << " frame " << frameNumber
             << " gaussians " << gaussians.size()
             << endl;
    #endif
}

const Gaussian *Frame::findGaussian(int x, int y, int *id)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::findGaussian: use!" << endl;
    #endif

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

Gaussian *Frame::findGaussianPrc(float x, float y, int *id, float prec)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::findGaussianPrc: use!" << endl;
    #endif

    Gaussian *gr=0x0;
    *id = -1;

    float minDist=FLT_MAX;

    for(unsigned g = 0 ; g < gaussians.size(); g++)
    {
        float dx = gaussians[g].x - x,
              dy = gaussians[g].y - y,
              dist = sqrt(dx*dx + dy*dy);

        if(minDist > dist )
        {

            gr = &gaussians[g];
            *id = g;
            minDist = dist;
        }
    }
    if(fabs(gaussians[*id].x - x) >= prec ||
       fabs(gaussians[*id].y - y) >= prec)
    {
        *id = -1;
        gr = 0x0;
    }
    return gr;
}


void Frame::newMatch(unsigned srcGaussianID, unsigned destFrame, unsigned dstGaussianID)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::newMatch: use!" << endl;
    #endif

    if(srcGaussianID >= match.size())
    {
        cout << "Frame::newMatch: Problem, invalid srcGaussian ID!" << endl;
    }
    match[srcGaussianID].push_back(PUU(destFrame,dstGaussianID));
}

bool Frame::findMatch(unsigned u, unsigned destFrame, int *v)
{
    cout << "Frame::findMatch: use!" << endl;
    *v = -1;
    for(unsigned i = 0 ; i < match[u].size(); i++)
    {
        if(match[u][i].first == destFrame)
        {
            *v = match[u][i].second;
            return true;
        }
    }
    return false;
}

unsigned Frame::nMatchs(unsigned destFrame)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::nMatchs: use!" << endl;
    #endif

    unsigned matchs = 0;

    for(unsigned u = 0 ; u < match.size();u++)
    {
        for(unsigned i = 0 ; i < match[u].size(); i++)
        {
            if(match[u][i].first == destFrame)
                matchs++;
        }
    }
    return matchs;
}

void Frame::removeGaussianMatch(unsigned srcGauss)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame:removeGaussianMatch::Clearing matchs of gaussian " << srcGauss << endl;
    #endif

    match[srcGauss].clear();
}

void Frame::removeGaussianMatchToFrame(unsigned srcGauss, unsigned destFrameID)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::removeGaussianMatchToFrame: use!" << endl;
    #endif

    unsigned i = 0;
    while(i< match[srcGauss].size())
    {
        if(match[srcGauss][i].first == destFrameID)
        {
            match[srcGauss].erase(match[srcGauss].begin() + i);
        }else
        {
            i++;
        }
    }
}

void Frame::removeMatchToFrame(unsigned dstFrameID)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::removeMatchToFrame: use!" << endl;
    #endif

    for(unsigned u = 0 ; u < match.size(); u++)
    {
        removeGaussianMatchToFrame(u,dstFrameID);
    }
}

void Frame::removeAllMatch()
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Clearing all matchs of frame" << endl;
    #endif
    for(unsigned u = 0 ; u < match.size() ; u++)
        match[u].clear();
}
