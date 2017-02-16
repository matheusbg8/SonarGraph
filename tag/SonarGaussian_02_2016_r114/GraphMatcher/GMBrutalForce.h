#ifndef GMBRUTALFORCE_H
#define GMBRUTALFORCE_H

#include "GraphMatcher/GraphMatcher.h"

class GMBrutalForce : public GraphMatcher
{
public:
    typedef pair<float,float> PFF;
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<float,PUU> PFPUU;
    typedef pair<unsigned,PUU> PUPUU;
    typedef pair<PUU,PFF> PUUPFF;

    GMBrutalForce();

    void brutalFORCExplore(vector<GraphLink *> &u,
                           vector< vector< vector<PFPUU> > > &explore);

    void brutalFORCEEdgeConfigCompare(vector<GraphLink *> &u, vector<PFPUU> &uExp,
                                      vector<GraphLink *> &v, vector<PFPUU> &vExp
                                     );

    void brutalFORCECompare(vector<GraphLink *> &u,vector< vector< vector<PFPUU> > > &uExp,
                            vector<GraphLink *> &v,vector< vector< vector<PFPUU> > > &vExp
                            );

    void brutalFORCE(vector<GraphLink *> &u, vector<GraphLink *> &v);

};



//// ===== Brutal Force explore test ====================

//#include <iostream>

//#include <cstdio>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "Sonar.h"
//#include "SonarConfig.h"
//#include "GroundTruth/GroundTruth.h"
//#include "GraphMatcher.h"
//#include "MatchViewer.h"

//using namespace std;
//using namespace cv;

//int main(int argc, char* argv[])
//{
//    typedef pair<unsigned,unsigned> PUU;
//    typedef pair<float,PUU> PFPUU;

//    GraphMatcher gm;

//    vector< vector< vector<PFPUU> > > explore;
//    vector<GraphLink *> u;

//    u.push_back(new GraphLink(0,0,1,1));
//    u.push_back(new GraphLink(0,0,1,2));
//    u.push_back(new GraphLink(0,0,1,3));
//    u.push_back(new GraphLink(0,0,1,4));
//    u.push_back(new GraphLink(0,0,1,5));
//    u.push_back(new GraphLink(0,0,1,6));
//    u.push_back(new GraphLink(0,0,1,7));
//    u.push_back(new GraphLink(0,0,1,8));
//    u.push_back(new GraphLink(0,0,1,9));
//    u.push_back(new GraphLink(0,0,1,10));
//    u.push_back(new GraphLink(0,0,1,11));
//    u.push_back(new GraphLink(0,0,1,12));
//    u.push_back(new GraphLink(0,0,1,13));
//    u.push_back(new GraphLink(0,0,1,14));
//    u.push_back(new GraphLink(0,0,1,15));
//    u.push_back(new GraphLink(0,0,1,16));
//    u.push_back(new GraphLink(0,0,1,17));
//    u.push_back(new GraphLink(0,0,1,18));
//    u.push_back(new GraphLink(0,0,1,19));
//    u.push_back(new GraphLink(0,0,1,20));
//    u.push_back(new GraphLink(0,0,1,21));
//    u.push_back(new GraphLink(0,0,1,22));

//    Cronometer c;
//    gm.brutalFORCExplore(u,explore);
//    float t = c.read();

//    unsigned k = 0;
//    for(unsigned i = 0 ; i < explore.size() ; i++)
//    {
//        cout << i << " , " << explore[i].size() << endl;
//        k+= explore[i].size();
//    }
//    cout << "Total , " << k << endl;
//    cout << "time , " << t << endl;

//    return 0;
//}


#endif // GMBRUTALFORCE_H
