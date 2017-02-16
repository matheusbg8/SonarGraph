#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <vector>

#include "Gaussian.h"

using namespace std;

// ==== DEBUG Section ====
//#define FRAME_TRACKING_DEBUG

// === end debug section ==

class Frame
{
public:
    Frame(const string& fileName,unsigned frameNumber):
        fileName(fileName),
        threshold1(0),
        threshold2(0),
        frameNumber(frameNumber){}
    ~Frame();

    string fileName;
    vector<Gaussian> gaussians;
    unsigned threshold1, threshold2, frameNumber;

    typedef pair<unsigned,unsigned> PUU;
    //                <FrameID , dest Gaussian ID>
    vector< vector<PUU> > match;
    /// @todo - This struct need to be otimized to allow faster erasers.

    // Find gaussian on position x,y in this frame, return id of gaussian found
    const Gaussian *findGaussian(int x, int y, int *id);

    // Find a gaussian on position x , y with precision
    Gaussian *findGaussianPrc(float x, float y, int *id, float prec);

    void newMatch(unsigned srcGaussianID, unsigned destFrame, unsigned dstGaussianID);

    // Find match of gaussian u on frame destFrame
    bool findMatch(unsigned u, unsigned destFrame, int *v);
    unsigned nMatchs(unsigned destFrame);

    // Remove all match of gaussaian srgGauss to all frames
    void removeGaussianMatch(unsigned srcGauss);

    // Remove all match of gaussaian srgGauss to frameID
    void removeGaussianMatchToFrame(unsigned srcGauss, unsigned destFrameID);

    // Remove all matchs to dest frame ID
    void removeMatchToFrame(unsigned dstFrameID);

    // Remove all match of this frame
    void removeAllMatch();
};

#endif // FRAME_H
