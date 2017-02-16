#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <vector>

#include "Gaussian/Gaussian.h"

using namespace std;

class Frame
{
public:
    Frame(const string& fileName):
        fileName(fileName),
        threshold1(150),
        threshold2(180){}

    string fileName;
    vector<Gaussian> gaussians;
    unsigned threshold1, threshold2;

    //                <FrameID , dest Gaussian ID>
    vector< vector<pair<unsigned,unsigned> > > match;

    Gaussian *findGaussian(int x, int y, int *id);

    void removeMatch(unsigned srcGauss);
    void removeAllMatch();

};

#endif // FRAME_H
