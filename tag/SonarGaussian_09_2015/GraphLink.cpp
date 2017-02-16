#include "GraphLink.h"
#include <algorithm>

// We change de order of edges
void GraphLink::computeInvAng(vector<GraphLink *> &vertex)
{
    if(vertex.size() > 0)
    {
        // Sort edges by ang
        sort(vertex.begin(),vertex.end(),compAng);
        for(unsigned j = 1 ; j < vertex.size(); j++)
        {
            vertex[j]->invAngle = vertex[j]->ang - vertex[j-1]->ang;
        }
        if(vertex.size() > 1)
            vertex[0]->invAngle =
      360.f - vertex[vertex.size()-1]->ang + vertex[0]->ang;
        else vertex[0]->invAngle = 360.f;
    }
}

void GraphLink::computeInvAngStatic(vector<GraphLink *> vertex)
{ // Here we change de invAng of edge without change de edges order of original vector
    if(vertex.size() > 0)
    {
        // Sort edges by ang
        sort(vertex.begin(),vertex.end(),compAng);
        for(unsigned j = 1 ; j < vertex.size(); j++)
        {
            vertex[j]->invAngle = vertex[j]->ang - vertex[j-1]->ang;
        }
        if(vertex.size() > 1)
            vertex[0]->invAngle =
      360.f - vertex[vertex.size()-1]->ang + vertex[0]->ang;
        else vertex[0]->invAngle = 360.f;
    }
}

bool compInvAng(const GraphLink* a, const GraphLink* b)
{
    return a->invAngle < b->invAngle;
}

bool compInvAngRef(const GraphLink &a, const GraphLink &b)
{
    return a.invAngle < b.invAngle;
}

bool compAng(const GraphLink *a, const GraphLink *b)
{
    return a->ang < b->ang;
}

bool compAngRef(const GraphLink &a, const GraphLink &b)
{
    return a.ang < b.ang;
}

bool compDist(const GraphLink *a, const GraphLink *b)
{
    return a->p < b->p;
}

bool compDistRef(const GraphLink &a, const GraphLink &b)
{
    return a.p < b.p;
}

bool compDistReverse(const GraphLink &a, const GraphLink &b)
{
    return a.p > b.p;
}

bool compDistReverseRef(const GraphLink &a, const GraphLink &b)
{
    return a.p > b.p;
}

bool compRelativeAng(const GraphLink *a, const GraphLink *b)
{
    return a->rAng < b->rAng;
}
