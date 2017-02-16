#include "FrameGT.h"
#include <cmath>

#include <iostream>
using namespace std;

FrameGT::FrameGT(const string &fileName, unsigned frameNumber):
    Frame(fileName,frameNumber),
    threshold1(0),
    threshold2(0)
{
}

FrameGT::~FrameGT()
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame:Destructor:: using destructor!"
             << " frame " << frameNumber
             << " gaussians " << gaussians.size()
             << endl;
#endif
}

/**
 * @brief Find a closest gaussian of position x,y in this frame,
 *  return the found gaussian.
 *
 * @param x - position x to search
 * @param y - position y to search
 * @param id - id of gaussian founnd
 * @return const Gaussian -
 */
const Gaussian *FrameGT::findClosestGaussian(int x, int y, int *id)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::findClosestGaussian: use!" << endl;
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

/**
 * @brief Return the first gaussian who intersect
 * a given gaussian.
 *
 * @param g - Given gaussian
 * @param id - Gaussian found id.
 * @return Gaussian - Gaussian found or 0x0.
 */
Gaussian *FrameGT::findIntersectGaussian(Gaussian &g, int *id)
{
    *id = -1;
    for(unsigned i = 0 ; i < gaussians.size(); i++)
    {
        Gaussian *myG = &gaussians[i];
        if(Gaussian::hasIntersection(g,*myG))
        {
             *id = i;
            return myG;
        }
    }
    return 0x0;
}

/**
 * @brief Find a gaussian on position x , y with precision
 *
 * @param x
 * @param y
 * @param id
 * @param prec
 * @return Gaussian
 */
Gaussian *FrameGT::findGaussianPrc(float x, float y, int *id, float prec)
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


void FrameGT::newMatch(unsigned srcGaussianID, unsigned destFrame, unsigned dstGaussianID)
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

/**
 * @brief Find match of gaussian u on frame destFrame
 *
 * @param u
 * @param destFrame
 * @param v
 * @return bool
 */
bool FrameGT::findMatch(unsigned u, unsigned destFrame, int *v)
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

unsigned FrameGT::nMatchs(unsigned destFrame)
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

/**
 * @brief Remove all match of gaussaian srgGauss to all frames
 *
 * @param srcGauss
 */
void FrameGT::removeGaussianMatch(unsigned srcGauss)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame:removeGaussianMatch::Clearing matchs of gaussian " << srcGauss << endl;
    #endif

    match[srcGauss].clear();
}

/**
 * @brief Remove all match of gaussaian srgGauss to frameID
 *
 * @param srcGauss
 * @param destFrameID
 */
void FrameGT::removeGaussianMatchToFrame(unsigned srcGauss, unsigned destFrameID)
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

/**
 * @brief Remove all matchs to dest frame ID
 *
 * @param dstFrameID
 */
void FrameGT::removeMatchToFrame(unsigned dstFrameID)
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Frame::removeMatchToFrame: use!" << endl;
    #endif

    for(unsigned u = 0 ; u < match.size(); u++)
    {
        removeGaussianMatchToFrame(u,dstFrameID);
    }
}

/**
 * @brief Remove all match of this frame
 *
 */
void FrameGT::removeAllMatch()
{
    #ifdef FRAME_TRACKING_DEBUG
        cout << "Clearing all matchs of frame" << endl;
    #endif
    for(unsigned u = 0 ; u < match.size() ; u++)
        match[u].clear();
}

/**
 * @brief Clear all matchs and gaussians
 *
 */
void FrameGT::clear()
{
    match.clear();
    gaussians.clear();
    threshold1 = threshold2 = 0;
}


ostream &operator <<(ostream &os, FrameGT &fr)
{
    typedef pair<unsigned,unsigned> PUU;

    os << (Frame&) fr;

    os << "Matchs:" << endl;
    vector< vector<PUU> > &match = fr.match;
    for(unsigned uG = 0 ; uG < match.size(); uG++)
    {
        for(unsigned iG = 0 ; iG < match[uG].size() ; iG++)
        {
            PUU &edge = match[uG][iG];
            os << "g " << uG << " to "
               << "frame " << edge.first
               << " g " << edge.second
               << endl;
        }
    }

    os << "Gaussians:" << endl;
    vector<Gaussian> &gs = fr.gaussians;
    for(unsigned i = 0 ;i < gs.size();i++)
    {
        os << gs[i] << endl;
    }
    return os;
}
