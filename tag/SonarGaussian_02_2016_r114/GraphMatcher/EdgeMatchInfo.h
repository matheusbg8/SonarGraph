#ifndef EDGEMATCHINFO_H
#define EDGEMATCHINFO_H

using namespace std;

class EdgeMatchInfo
{
public:
    unsigned u, v;
    float score;
    EdgeMatchInfo(unsigned u, unsigned v, float score);
};

#endif // EDGEMATCHINFO_H
