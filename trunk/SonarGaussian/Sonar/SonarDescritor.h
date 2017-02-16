#ifndef SONARDESCRITOR_H
#define SONARDESCRITOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<vector>
#include "Segmentation/Segmentation.h"
#include"Gaussian.h"
#include"GraphLink.h"
#include "GraphMatcher/MatchInfo/MatchInfoExtended.h"

using namespace std;
using namespace cv;


/**
 * @brief This class describe a environment
 * with a graph and a Gaussian functions extratcted
 * from acoustic images.
 *
 */
class SonarDescritor
{
public:
    vector<Gaussian> gaussians;
    vector<vector<GraphLink*> > graph;/**< This is our graph representatation, a vector of vertex */

    // Currently we are not using this attributes.
    float x, y, ang; /**< This attributes are about frame allingment */

    SonarDescritor();

    ~SonarDescritor();

    void clearLinks();

    void clearGraph();

    void clearGaussian();

    void createGraph(float graphLinkDistance, bool direct=true);

    void createGraphNeighborRelative(float graphNeigborDistanceRelativeLink);

    void addGaussian(const Gaussian &g, bool merge=false);

    unsigned numberOfEdges();
};

#endif // SONARDESCRITOR_H
