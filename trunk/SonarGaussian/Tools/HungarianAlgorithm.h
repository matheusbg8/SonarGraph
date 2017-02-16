#ifndef HUNGARIANALGORITHM_H
#define HUNGARIANALGORITHM_H

#include <vector>
#include "GraphMatcher/MatchInfo/MatchInfoWeighted.h"


/**
 * @brief Solve the assingment problem in time O(n^3)
 *
 *
 * Special thanks for x-ray from topcoder where this
 * code was copied with small modifications.
 *  https://www.topcoder.com/community/data-science/data-science-tutorials/assignment-problem-and-hungarian-algorithm/
 *
 */
class HungarianAlgorithm
{
private:
    #define HA_INF 100000000 //just infinity
    #define HA_N 55 //max number of vertices in one part

    float lx[HA_N], ly[HA_N]; //labels of X and Y parts
    int xy[HA_N]; //xy[x] - vertex that is matched with x,
    int yx[HA_N]; //yx[y] - vertex that is matched with y
    bool S[HA_N], T[HA_N]; //sets S and T in algorithm
    float slack[HA_N]; //as in the algorithm description
    int slackx[HA_N]; //slackx[y] such a vertex, that
     // l(slackx[y]) + l(y) - w(slackx[y],y) = slack[y]
    int prev[HA_N]; //array for memorizing alternating paths

    void init_labels();
    void augment();
    void update_labels();
    void add_to_tree(int x, int prevx);
public:
    HungarianAlgorithm();

    float cost[HA_N][HA_N]; //cost matrix
    int n, max_match; //n workers and n jobs

    void clear();
    unsigned maxVertex();
    int hungarian(std::vector<MatchInfoWeighted> &matchs);

};


#endif // HUNGARIANALGORITHM_H
