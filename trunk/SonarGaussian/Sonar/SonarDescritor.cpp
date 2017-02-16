#include "SonarDescritor.h"
#include <cfloat>
#include <iostream>
#include <cstdio>

#include "Drawing/Drawing.h"

using namespace std;

SonarDescritor::SonarDescritor():
    x(0.f),y(0.f),ang(0.f)
{
}

SonarDescritor::~SonarDescritor()
{
    clearGraph();
}

/**
 * @brief Clear all link of graph
 *
 */
void SonarDescritor::clearLinks()
{
    for(unsigned gi = 0; gi < graph.size() ; gi++)
    {
        graph[gi].clear();
    }
}

void SonarDescritor::clearGraph()
{
    for(unsigned i = 0 ; i < graph.size(); i++)
    {
        for(unsigned j = 0 ; j < graph[i].size(); j++)
            delete graph[i][j];
    }

    graph.clear();
}

void SonarDescritor::clearGaussian()
{
    gaussians.clear();
}


/**
 * @brief Create a symetric graph linking vertex with
 * distances lower than graphLinkDistance.
 *
 * @param graphLinkDistance
 */
void SonarDescritor::createGraph(float graphLinkDistance, bool direct)
{
    graph.clear();
    graph.resize(gaussians.size());

    for(unsigned i = 0 ; i < gaussians.size() ; i++)
    {
        float cx = gaussians[i].x,
              cy = gaussians[i].y;

        for(unsigned j = i+1 ; j < gaussians.size() ; j++)
        {
            float x = gaussians[j].x,
                  y = gaussians[j].y,
                  dx=x-cx, dy=y-cy,
                  d = sqrt(dx*dx + dy*dy);

            if(d <= graphLinkDistance)
            {
                float dt = 180.f*atan2f(dx,-dy)/M_PI, edt;
                if(dt<0.f) dt+=360.f;

                edt = dt - gaussians[i].ang;
                if(edt < 0.f) edt += 360.f;

                graph[i].push_back(new GraphLink(dt,edt,0.f, d,j));

                if(direct)
                {
                if(dt > 180.f) dt-=180.f;
                else dt+= 180.f;

                edt = dt - gaussians[j].ang;
                if(edt < 0.f) edt += 360.f;

                graph[j].push_back(new GraphLink(dt,edt,0,d,i));
                }
            }
        }

        GraphLink::computeInvAng(graph[i]);
    }
}

void SonarDescritor::createGraphNeighborRelative(float graphNeigborDistanceRelativeLink)
{
    /// @todo - Consider using KD-Tree for optimized searchers (you need to change a heart of this code for do this)

//    graph.clear();
//    graph.resize(gaussians.size());

//    vector<vector<float> > distances(gaussians.size(), vector<float>(gaussians.size() ));
//    vector<vector<uchar> > linkDone(gaussians.size(), vector<uchar>(gaussians.size(),0));

//    for(unsigned i = 0 ; i < gaussians.size() ; i++)
//    {
//        float cx = gaussians[i].x,
//              cy = gaussians[i].y;

//        // Compute the distances to vertex i.
//        for(unsigned j = i+1 ; j < gaussians.size() ; j++)
//        {
//            float x = gaussians[j].x,
//                  y = gaussians[j].y,
//                  dx=x-cx, dy=y-cy,
//                  d = sqrt(dx*dx + dy*dy);

//            distances[i][j] = d;
//            distances[j][i] = d;
//        }
//        distances[i][i] = FLT_MAX;

//        // Search the closest neighbord vertex to i
//        float closestNeighborDistance=FLT_MAX;/* *min_element(distances[i].begin(), distances[i].end());*/

//        for(unsigned j = 0; j < gaussians.size() ; j++)
//        {
//            if(closestNeighborDistance > distances[i][j])
//                closestNeighborDistance = distances[i][j];
//        }

//        // Create the edges relative to vertex i
//        for(unsigned j = 0; j < gaussians.size() ; j++)
//        {
//            if(i==j) continue;
//            float dist_ij = distances[i][j],
//                  dx=gaussians[j].x-cx, dy=gaussians[j].y-cy;

//            if(linkDone[i][j] == 0 && dist_ij-closestNeighborDistance <= graphNeigborDistanceRelativeLink)
//            {
////                cout << "link between " << i << " and " << j << " distance " << dist_ij << " cDistance " << closestNeighborDistance
////                     << " (" << gaussians[i].x << " , " << gaussians[i].y << ") ("
////                     <<  gaussians[j].x << " , " << gaussians[j].y << ")" << endl;
//                // Link Happen's Mark

//                linkDone[i][j] = 1;
//                linkDone[j][i] = 1;

//                // Calculate Edge wight (Edge degree relative to North, Vertex Distance)
//                float dt = 180.f*atan2f(dx,-dy)/M_PI;
//                if(dt<0.f) dt+=360.f;

//                graph[i].push_back(new GraphLink(dt,dist_ij,j));

//                if(dt > 180.f) dt-=180.f;
//                else dt+= 180.f;

//                graph[j].push_back(new GraphLink(dt,dist_ij,i));
//            }
//        }

//        GraphLink::computeInvAng(graph[i]);
    //    }
}

float sqrt_dist(Point2f &p1, Point2f &p2)
{
    float dx = p1.x - p2.x, dy = p1.y - p2.y;
    return dx*dx - dy*dy;
}

void SonarDescritor::addGaussian(const Gaussian &g, bool merge)
{
    bool merged = false;

    if(merge)
    for(int i = gaussians.size()-1 ; i >= 0; i--)
    {
//        cout << "Intersectio test " << gaussians.size() << " - " << i << endl;
        if(Gaussian::hasIntersection(g,gaussians[i]))
        {
            cout << "Merge with " << i << endl;

            gaussians[i].merge(g,1.f);
            merged = true;
            break;
        }
    }

    if(!merged)
        gaussians.push_back(g);
}

unsigned SonarDescritor::numberOfEdges()
{
    unsigned nE = 0u;
    for(unsigned i = 0u; i < graph.size() ; i++)
    {
        nE += graph[i].size();
    }
    return nE;
}
