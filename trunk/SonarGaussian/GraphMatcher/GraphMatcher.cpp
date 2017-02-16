#include "GraphMatcher.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <stack>
#include <queue>

#include "Drawing/Drawing.h"

#include "GraphMatcher/VertexMatcher/VMBrutalForce.h"
#include "GraphMatcher/VertexMatcher/VMHeuristicByAngVariation.h"
#include "GraphMatcher/VertexMatcher/VMHeuristicByAngVariation1.h"
#include "GraphMatcher/VertexMatcher/VMScalenePC.h"
#include "GraphMatcher/VertexMatcher/VMWeghtedSumPC.h"

#include "GraphMatcher/GraphMatchFinder/GMFVertexByVertex.h"
#include "GraphMatcher/GraphMatchFinder/GMFHungarian.h"
#include "GraphMatcher/GraphMatchFinder/GMFByVertexMatch.h"
#include "GraphMatcher/GraphMatchFinder/GMFByEdgeExploration.h"
#include "GraphMatcher/GraphMatchFinder/GMFBestDirect.h"

class VertexMatch
{
public:
    VertexMatch(float score=FLT_MAX, float angCorrection=0.f, int IDMathc=-1):
        score(score),angCorrection(angCorrection), IDMatch(IDMathc){}
    float score,angCorrection;
    int IDMatch;
};

void GraphMatcher::byDistanceCompare(vector<GraphLink *> &u, vector<GraphLink *> &v)
{
    vector<float> bipartiteEdgeMatch[u.size()];

    for(unsigned i = 0; i < u.size() ;i++)
        bipartiteEdgeMatch[i].resize(v.size(),-1);

    distThreshold = 5;

    for(unsigned i = 0; i < u.size() ;i++)
    {
        for(unsigned j = 0; j < v.size() ;j++)
        {
            float diff = fabs(u[i]->p - v[j]->p);
            if(diff <= distThreshold)
            {
                bipartiteEdgeMatch[i][j] = diff;
            }
        }
    }

    cout << "Bipartite Graph" << endl;
    for(unsigned i = 0; i < u.size() ;i++)
    {
        for(unsigned j = 0; j < v.size() ;j++)
        {
            if(bipartiteEdgeMatch[i][j] >= 0)
                cout << i+1 << " <-(" << bipartiteEdgeMatch[i][j] << ")-> " << j+1 << endl;
        }
    }

//    vector<PUU> edgeMatchGraph[u.size()];

//    for(unsigned m = 0; m < u.size() ;m++)
//    {
//        for(unsigned n = 0; n < u.size() ;n++)
//        {
//            for(unsigned i = m+1; i != m;i= (i+1)%u.size())
//            {
//                for(unsigned j = n+1; j != n ;j = (j+1)%v.size())
//                {
//                    if(bipartiteEdgeMatch[i][j] >= 0)
//                        cout << i+1 << " <-(" << bipartiteEdgeMatch[i][j] << ")-> " << j+1 << endl;
//                }
//            }
//        }
//    }

    byDistanceCompare_(u,v,bipartiteEdgeMatch);
}

void GraphMatcher::byDistanceCompare_(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                      vector<float> *edgeMatch)
{
//    unsigned uIni=0, vIni=0;


//    for(unsigned i = uIni ; i < u.size() ; i++)
//    {
//        for(unsigned j = vIni; j < v.size() ; j++)
//        {
//            if(edgeMatch)


//        }
//    }
}

float GraphMatcher::scaleneError(float degreeAngDiff, float length1, float length2)
{
    return sqrt(length1*length1 + length2*length2 -2.f*length1*length2*cos( (degreeAngDiff*M_PI)/180.f ));
}

GraphMatcher::GraphMatcher(ConfigLoader &config):
    m_vm(0x0), m_gmf(0x0)
{
    loadDefaultConfig();
    load(config);
}

GraphMatcher::GraphMatcher():
    m_vm(0x0), m_gmf(0x0)
{
    loadDefaultConfig();
}

void GraphMatcher::setVertexMatcher(VertexMatcher *vm)
{
    if(vm != 0x0)
    {
        if(m_vm != 0x0)
            delete m_vm;

        m_vm = vm;
        m_gmf->setVertexMatcher(m_vm);
    }else
    {
        cout << "GraphMatcher warning: Invalid Vertex Matcher!" << endl;
    }
}

void GraphMatcher::setGraphFinder(GraphMatchFinder *gmf)
{
    if(gmf != 0x0)
    {
        if(m_gmf != 0x0)
            delete m_gmf;

        m_gmf = gmf;
        m_gmf->setVertexMatcher(m_vm);
    }else
    {
        cout << "GraphMatcher warning: Invalid Graph Matcher Finder!" << endl;
    }
}

void GraphMatcher::loadDefaultConfig()
{
    if(m_vm == 0x0)
        m_vm = new VMHeuristicByAngVariation;

    if(m_gmf == 0x0)
        m_gmf = new GMFVertexByVertex;

    m_gmf->setVertexMatcher(m_vm);
}

void GraphMatcher::load(ConfigLoader &config)
{
    VertexMatcher *vm=0x0;
    GraphMatchFinder *gmf=0x0;
    string str;

    if(config.getString("General","VertexMatcher",&str))
    {
        if(str == "VMBrutalForce")
        {
            vm = new VMBrutalForce;

        }else if(str == "VMHeuristicByAngVariation")
        {
            vm = new VMHeuristicByAngVariation;

        }else if(str == "VMHeuristicByAngVariation1")
        {
            vm = new VMHeuristicByAngVariation1;

        }else if(str == "VMScalenePC")
        {
            vm = new VMScalenePC;

        }else if(str == "VMWeghtedSumPC")
        {
            vm = new VMWeghtedSumPC;
        }
    }

    if(config.getString("General","GraphMatchFinder",&str))
    {
        if(str == "GMFVertexByVertex")
        {
            gmf = new GMFVertexByVertex;
        }else if(str == "GMFHungarian")
        {
            gmf = new GMFHungarian;
        }else if(str == "GMFByVertexMatch")
        {
            gmf = new GMFByVertexMatch;
        }else if(str == "GMFByEdgeExploration")
        {
            gmf = new GMFByEdgeExploration;
        }else if(str == "GMFBestDirect")
        {
            gmf = new GMFBestDirect;
        }
    }

    if(vm != 0x0)
    {
        vm->load(config);
        setVertexMatcher(vm);
    }else
    {
        cout << "Warning: GraphMatcher didn't get VertexMatcher configs!" << endl;
    }

    if(gmf != 0x0)
    {
        gmf->load(config);
        setGraphFinder(gmf);
    }else
    {
        cout << "Warning: GraphMatcher didn't get GraphMatcheFinder configs!" << endl;
    }

}

void GraphMatcher::findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                             vector<MatchInfo> &vertexMatch)
{
    m_gmf->findMatch(sd1,sd2,vertexMatch);
}

void GraphMatcher::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                  vector<MatchInfoExtended> &matchInfo)
{
    m_gmf->findMatchDebug(sd1,sd2,matchInfo);
}

void GraphMatcher::drawMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                             vector<MatchInfo> &vertexMatch,
                             Mat colorImg)
{
    Drawing::drawDescriptorColor(colorImg,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    Drawing::drawDescriptorColor(colorImg,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].uID , v = vertexMatch[i].vID;
        line(colorImg,
             Point2f(sd1->gaussians[u].x, sd1->gaussians[u].y),
             Point2f(sd2->gaussians[v].x, sd2->gaussians[v].y),
             Scalar(0,0,255),2);
    }
}

void GraphMatcher::drawMatchOnImgs(Mat &result,const Size2i &rSZ,
                                   SonarDescritor *sd1, Mat &img1,
                                   SonarDescritor *sd2, Mat &img2,
                                   vector<MatchInfo> &vertexMatch)
{
    result = //Mat::zeros(rSZ.height, rSZ.width,CV_8UC3);
             Mat(rSZ.height, rSZ.width,CV_8UC3);

    Size2i imSZ(rSZ.width/2,rSZ.height);

    Mat cimg1, cimg2;

    cvtColor(img1,cimg1,CV_GRAY2BGR);
    cvtColor(img2,cimg2,CV_GRAY2BGR);

    Drawing::drawDescriptorColor(cimg1,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    Drawing::drawDescriptorColor(cimg2,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    resize(cimg1,cimg1,imSZ);
    resize(cimg2,cimg2,imSZ);

    cimg1.copyTo(result(Rect(0         , 0 , imSZ.width , imSZ.height)));
    cimg2.copyTo(result(Rect(imSZ.width, 0 , imSZ.width , imSZ.height)));

    float e1x = (float)imSZ.width/ (float)img1.cols, e1y = (float)imSZ.height / (float)img1.rows,
          d1x = 0 , d1y = 0,
          e2x = (float)imSZ.width/ (float)img2.cols, e2y = (float)imSZ.height / (float)img2.rows,
          d2x = imSZ.width, d2y = 0;

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].uID , v = vertexMatch[i].vID;
        line(result,
             Point2f(d1x + e1x*sd1->gaussians[u].x, d1y + e1y*sd1->gaussians[u].y),
             Point2f(d2x + e2x*sd2->gaussians[v].x, d2y + e2y*sd2->gaussians[v].y),
             Scalar(0,0,255),2);
    }

}

void GraphMatcher::drawVertexMatch(Mat colorImg, unsigned idU, const Gaussian& vu , const vector<GraphLink *> &eu, unsigned idV, const Gaussian &vv, const vector<GraphLink *> &ev, const Rect &drawingArea)
{
    float ux = drawingArea.x + drawingArea.width/4.f,
          uy = drawingArea.y + drawingArea.height/2.f,
          vx = ux*3.f,
          vy = uy,
          mr = min(drawingArea.width,drawingArea.height)/2.f;


    char tempStr[40];

    // Find longest edge to normalization after
    unsigned maxECount = max(eu.size(),ev.size()),
             maxEdist = 0.f;
    for(unsigned i =0; i < maxECount ; i++)
    {
        if(i < eu.size() && maxEdist < eu[i]->p)
            maxEdist = eu[i]->p;
        if(i < ev.size() && maxEdist < ev[i]->p)
            maxEdist = ev[i]->p;
    }


    // Draw elipse u
    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vu.dx, vu.dy),
            vu.ang,
            0.0 , 360.0,
            Scalar(255,0,0),
            2 , 8 , 0);

    sprintf(tempStr, "ID %d", idU);
    putText(colorImg,tempStr,
           Point2f(ux+20, uy+20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(0,255,255),2);


    // Draw edges u
    for(unsigned i =0; i < eu.size() ; i++)
    {
        float r = mr*(eu[i]->p/maxEdist),
              ang = eu[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(ux, uy),
             Point2f(ux + r*sin(ang),
                     uy - r*cos(ang)),
             Scalar(255,0,0),2);

        // Draw Edge's angle
        sprintf(tempStr,"p%.2f ; a%.2f ; i%2.f", eu[i]->p , eu[i]->ang,eu[i]->invAngle);

        int baseLine = 0;
        Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.4,1,&baseLine);

        putText(colorImg, tempStr,
                Point2f(ux + r*sin(ang)-textSize.width/2,
                        uy - r*cos(ang)),
                FONT_HERSHEY_COMPLEX, 0.4,
                Scalar(0,255,255), 1, 8);
    }

    // Draw elipse v
    ellipse(colorImg,
            Point2f(vx, vy),
            Size2f(vv.dx, vv.dy),
            vv.ang,
            0.0 , 360.0,
            Scalar(0,0,255),
            2 , 8 , 0);

    sprintf(tempStr, "ID %d", idV);
    putText(colorImg,tempStr,
           Point2f(vx+20, vy+20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,0),2);

    // Draw edges v
    for(unsigned i =0; i < ev.size() ; i++)
    {
        float r = mr*(ev[i]->p/maxEdist),
              ang = ev[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(vx, vy),
             Point2f(vx + r*sin(ang),
                     vy - r*cos(ang)),
             Scalar(0,0,255),2);

        // Draw Edge's angle
        sprintf(tempStr,"p%.2f ; a%.2f ; i%2.f",ev[i]->p, ev[i]->ang,ev[i]->invAngle);

        int baseLine = 0;
        Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.4,1,&baseLine);

        putText(colorImg, tempStr,
                Point2f(vx + r*sin(ang)-textSize.width/2,
                        vy - r*cos(ang)),
                FONT_HERSHEY_COMPLEX, 0.4,
                Scalar(255,255,0), 1, 8);

    }
}

