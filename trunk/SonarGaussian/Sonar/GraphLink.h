#ifndef GRAPHLINK_H
#define GRAPHLINK_H

#include<vector>
#include<list>

using namespace std;


/**
 * @brief This structure represent an edge, a link between two vertex,
 * and hold informations about edge slope (degree), edge lenght (pixel)
 * and detin vertex ID.
 *
 */
class GraphLink
{
public:
    GraphLink(float ang=0.f,float rAng=0.f,float invAng=0.f, float p=0.f, int dest=-1):
        ang(ang),p(p),rAng(rAng),invAngle(invAng),dest(dest){}

    float ang, // inclination relative to vertical image axis
            p, // lenght od edge
         rAng, // inclination relative to gassian principal axis
     invAngle; // inclination diference to next edge

    int dest; // dest vertex

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

bool compRelativeAng(const GraphLink *a, const GraphLink *b);

/**
 * @brief This static method make a copy of a dynamic allocated vertex
 * to a static allocated vertex.
 *
 * @param source - A dynamic allocated graph.
 * @param dest - A static allocated graph.
 */
void staticCopyVertex(const vector<GraphLink*> &source, vector<GraphLink> &dest);

/**
 * @brief This static method copy a dynamic allocated vertex.
 *  A new vertex will be created and furthermore will be need
 * deleted after use.
 *
 * @param source - A dynamic alloceted vertex will be copied.
 * @param dest - A dynamic allocated vertex reference.
 */
void dinamicCopyVertex(const vector<GraphLink*> &source, vector<GraphLink *> &dest);

/**
 * @brief Thsi static method delete a dynamic allocated vertex.
 *
 * @param vertex -
 */
void freeVertex(vector<GraphLink *> &vertex);

void copyVertexList(const vector<GraphLink*> &source, list<GraphLink> &dest);


#endif // GRAPHLINK_H
