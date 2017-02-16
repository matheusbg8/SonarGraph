#ifndef VERTEXMATCHER_H
#define VERTEXMATCHER_H

#include "Sonar/GraphLink.h"
#include "Sonar/SonarConfig/ConfigLoader.h"
#include "Sonar/Gaussian.h"
#include "GraphMatcher/MatchInfo/MatchInfoWeighted.h"


/**
 * @brief This class compute the error between two vertex.
 *
 */
class VertexMatcher
{
public:
    VertexMatcher();
    virtual ~VertexMatcher();

    /**
     * @brief Set algorithm and the algorithm parameters
     *
     * @param config - A generic config
     * @return bool - true if was sucess seted
     */
    virtual bool load(ConfigLoader &config) = 0;


    /**
     * @brief Compute the similarity between vertex
     *
     * @param gu - Vertex one
     * @param gv - Vertex two
     * @param eu - Edges of vertex one
     * @param ev - Edges of vertex two
     * @param matchEdges - Optional pointer to unsigned int
     * that will be filled with the amount of matched edges.
     *
     * @return float - The similarity between the vertices
     */
    virtual float vertexMatch(Gaussian &gu, Gaussian &gv,
                              vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                              unsigned *matchEdges=0x0) = 0;


    /**
     * @brief It is a debug mode of similarity computation,
     * It will compute the similarity between vertex and
     * retur the score of each pair of edges matched.
     *
     * @param gu - First compared vertex
     * @param gv - Second compared vertex
     * @param u - Adjcent edges of first vertex
     * @param v - Adjacente edges of second vertex
     * @param matchEdges - A struct wich represent the
     * similarity between each pair of edges.
     * @return float - Similarity between the vertices
     */
    virtual float vertexMatch(Gaussian &gu, Gaussian &gv,
                              vector<GraphLink *> &u, vector<GraphLink *> &v,
                              vector<MatchInfoWeighted> &matchEdges) = 0;
};

#endif // VERTEXMATCHER_H
