#ifndef GRAPHLINK_H
#define GRAPHLINK_H

#include<vector>
#include<list>

using namespace std;

class GraphLink
{
public:
    GraphLink(float ang=0.f,float rAng=0.f, float p=0.f, int dest=-1):
        ang(ang),p(p),rAng(rAng),invAngle(0.f),dest(dest){}

    float ang, p, rAng, invAngle;
    int dest;

    static void computeInvAng( vector<GraphLink *> &vertex); // We change de order of edges
    static void computeInvAngStatic(vector<GraphLink*> vertex);// Here we change de invAng of edge without change de edges order

};

bool compInvAng(const GraphLink *a, const GraphLink *b);
bool compInvAngRef(const GraphLink &a, const GraphLink &b);

bool compAng(const GraphLink *a, const GraphLink *b);
bool compAngRef(const GraphLink &a, const GraphLink &b);

bool compDist(const GraphLink *a, const GraphLink *b);
bool compDistRef(const GraphLink &a, const GraphLink &b);

bool compDistReverse(const GraphLink *a, const GraphLink *b);
bool compDistReverseRef(const GraphLink &a, const GraphLink &b);

#endif // GRAPHLINK_H
