#include "VMBrutalForce.h"

#include <stack>
using namespace std;

VMBrutalForce::VMBrutalForce()
{
}

void VMBrutalForce::brutalFORCExplore(vector<GraphLink *> &u, vector<vector<vector<PFPUU> > > &explore)
{
    //    vector< // Group Size
    //            vector< // Group
    //                vector<PFPUU> // Elements (Diff Ang, initial Edge ID, end Edge ID)
    //                  >
    //          > explore;

        explore.resize(u.size()+1,vector< vector<PFPUU> >());


        explore[u.size()].push_back( vector<PFPUU> ());
        for(unsigned i = 0; i < u.size()-1; i++)
        {
            explore[u.size()][0].push_back(PFPUU(u[i]->invAngle,PUU(i,i+1)));
        }
        explore[u.size()][0].push_back(PFPUU(u[u.size()-1]->invAngle,PUU(u.size()-1,0)));

        stack<
             pair<unsigned,// setSize
                pair<unsigned, // setID
                     unsigned  // postion of last swap
                    >
                 >
             > q; // Recursive Stack!

        q.push(PUPUU(u.size(),PUU(0,0)));
        unsigned setSize,nextSize, setID,lastSwapPostion;

        while(!q.empty())
        {
            setSize = q.top().first;
            setID = q.top().second.first;
            lastSwapPostion = q.top().second.second;
            q.pop();

            nextSize = setSize-1;

            // Get current config
            vector<PFPUU> &exp = explore[setSize][setID];
            unsigned iSwap, jSwap;

            // Perform recursive swaps in corrent config set
            // Changes aways start at beginner (iSwap=0)
            for(lastSwapPostion, iSwap =0; lastSwapPostion < setSize; lastSwapPostion++, iSwap++)
            {
                // Create a new set configuration
                unsigned newSetID = explore[nextSize].size();

                explore[nextSize].push_back( vector<PFPUU> ());
                vector<PFPUU> &newExp = explore[nextSize][newSetID];

                newExp.reserve(nextSize);

                jSwap = (iSwap+1)%setSize;
                // Blend two elements (iSwap with jSwap)
               newExp.push_back(PFPUU(
                                     exp[iSwap].first + exp[jSwap].first,
                                     PUU(exp[iSwap].second.first, exp[jSwap].second.second)
                                      )
                                 );

                unsigned i = (iSwap+2)%setSize,
                         m = setSize-3;

                // Copy remainded elements
                while(m < 999999)
                {
                    newExp.push_back(exp[i]);
                    i=(i+1)%setSize; // Circular increment
                    m--;
                }

                // Work with new set created
                if(nextSize>2 && lastSwapPostion < nextSize)
                {
                    q.push(PUPUU(nextSize,PUU(newSetID,lastSwapPostion)));
                }
            }
        }

    #ifdef DEBUG_GRAPHMACHER
        for(unsigned setSize = u.size(); setSize > 1;setSize--)
        {
            for(unsigned setID = 0 ; setID < explore[setSize].size(); setID++)
            {
                for(unsigned k = 0 ; k < explore[setSize][setID].size() ; k++)
                {
                    cout << "(" << explore[setSize][setID][k].first
                         << " , " << explore[setSize][setID][k].second.first
                         << " , " << explore[setSize][setID][k].second.second
                         << " ) ";
                }
                cout << endl;
            }
        }
    #endif
}

void VMBrutalForce::brutalFORCEEdgeConfigCompare(vector<GraphLink *> &u, vector<PFPUU> &uExp, vector<GraphLink *> &v, vector<PFPUU> &vExp)
{
    //    if(uExp.size() > 0 && uExp.size() != vExp.size())
    //        return;

    //    unsigned nEdges = uExp.size(),
    //             nComb = nEdges-1;

    //    float bestScore=999999.f,
    //          angDiff, distDiff;

    //    int bestComb = -1;

    //    for(unsigned comb = 0 ; comb < nComb; comb++)
    //    {
    //        float angMeanDiff = 0.f,
    //              distMeanDiff = 0.f;

    //        unsigned i = 0;
    //        while(i < nEdges)
    //        {
    //            angDiff = fabs(uExp[i].first - vExp[(i+comb)%nEdges].first);
    //            if(angDiff > angThreshold)
    //                break;

    //            distDiff = fabs(uExp[i].second.first - vExp[(i+comb)%nEdges].second.first);
    //            if(distDiff > distThreshold)
    //                break;
    //            // The second edge will be tested ont the next iteration

    //            angMeanDiff += angDiff;
    //            distMeanDiff += distDiff;

    //            i++;
    //        }

    //        if(i < nEdges)
    //            continue;

    //        // OK, we find a match
    //        // now, we will evaluate this match

    //        angMeanDiff /= nEdges;
    //        distMeanDiff /= nEdges;

    //        if(angMeanDiff)
    //    }
}

bool VMBrutalForce::load(ConfigLoader &config)
{

    return false;
}

float VMBrutalForce::vertexMatch(Gaussian &gu, Gaussian &gv, vector<GraphLink *> &eu, vector<GraphLink *> &ev, unsigned *matchEdges)
{

}

float VMBrutalForce::vertexMatch(Gaussian &gu, Gaussian &gv,
                                 vector<GraphLink *> &u, vector<GraphLink *> &v,
                                 vector<MatchInfoWeighted> &matchEdges)
{

}
