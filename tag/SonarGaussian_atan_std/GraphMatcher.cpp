#include "GraphMatcher.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

class VertexMatch
{
public:
    VertexMatch(float score=FLT_MAX, float angCorrection=0.f, int IDMathc=-1):
        score(score),angCorrection(angCorrection), IDMatch(IDMathc){}
    float score,angCorrection;
    int IDMatch;
};

bool compVertexMatch(const VertexMatch *a, const VertexMatch *b)
{
    return a->score < b->score;
}

void GraphMatcher::staticCopyVertex(const vector<GraphLink *> &source, vector<GraphLink> &dest)
{
    dest.resize(source.size(), 0x0);
    for(unsigned i =0; i < source.size(); i++)
        dest[i] = (*(source[i]));
}

void GraphMatcher::dinamicCopyVertex(const vector<GraphLink *> &source, vector<GraphLink*> &dest)
{
    dest.resize(source.size(), 0x0);
    for(unsigned i =0; i < source.size(); i++)
        dest[i] = new GraphLink(*(source[i]));
}

void GraphMatcher::freeVertex(vector<GraphLink *> &vertex)
{
    for(unsigned i =0; i < vertex.size(); i++)
        delete vertex[i];
    vertex.clear();
}

void GraphMatcher::copyVertexList(const vector<GraphLink *> &source, list<GraphLink> &dest)
{
    for(unsigned i = 0 ; i < source.size(); i++)
        dest.push_back(*(source[i]));
}

bool GraphMatcher::findPairOfVertex(vector<GraphLink*> &u, vector<GraphLink*> &v, float *invAngError, float *edgeDistError, float *angCorrectionMean, int *edgeCut)
{
    // Sort edges by weight increasing order (distances vertex to vertex)
    sort(u.begin(),u.end(), compDist);
    sort(v.begin(),v.end(), compDist);

    int i , j;
    float distError, angError, ACangCorrectionMean=0.f;

    *edgeDistError=0.f;
    *invAngError=0.f;
    edgeCut=0;

    // Compute edges match by wight (vertex distances)
    j = 0;
    i = 0;
    while(i < u.size() && j < v.size())
    {
        distError = fabs(u[i]->p - v[j]->p);

        if(distError > distThreshold)
        {   // cut no match edge

            // cut the edge with lowest weight
            if(v[i]->p < u[j]->p)
            {
                *edgeDistError+= v[i]->p;
                edgeCut++;
                v.erase(v.begin()+j);
            }else
            {
//                edgeDistError+= cu[i]->p;
                edgeCut++;
                u.erase(u.begin()+i);
            }
        }else
        {
            *edgeDistError += distError;
            ACangCorrectionMean+= u[i]->ang - v[j]->ang;
            i++;
            j++;
        }
    }

    while(i < u.size())
    {
        *edgeDistError+= u[i]->p;
        u.erase(u.begin()+i);
    }

    while(j < v.size())
    {
        *edgeDistError+= v[j]->p;
        v.erase(v.begin()+j);
    }

    *angCorrectionMean = ACangCorrectionMean/u.size();

    if(u.size() <= 1 || v.size() <= 1)
    {
        // No match found
        return false; // Infinty
    }

    // Compute vertex match by invAng (Edges distances)
    GraphLink::computeInvAngStatic(v);// No change the edges order, it's important
    GraphLink::computeInvAngStatic(u);// No change the edges order

    for(i = 0; i < v.size(); i++)
    {
        angError = fabs(v[i]->invAngle - u[i]->invAngle);
        if(angError > angThreshold)
        {   // No match found
            return false;
        }
        *invAngError += angError;
    }

    return true;
}

void GraphMatcher::compVertexDist(const vector<GraphLink *> &u, const vector<GraphLink *> &v)
{
/*
    const vector<GraphLink *> *vm , *vM;
    if(u.size() > v.size())
    {
        vM = &u;
        vm = &v;
    }else
    {
        vM = &v;
        vm = &u;
    }

    vector< vector< pair<float , unsigned> > > match(vm->size());
    float distError;
    unsigned i, j;
    for(i = 0 ; i < vm->size() ; i++)
    {
        for(j = 0 ; j < vM->size(); j++)
        {
            distError = fabs((*vm)[i]->p - (*vM)[j]->p);
            if( distError < distThreshold)
                match[i].push_back(make_pair(distError, j));
        }
        sort(match[i].begin(), match[i].end());
    }

    for(i = 0; i < match.size() ; i++)
    {
        for(j = 0 ; j < match[i].size() ; j++)
        {
            cout << "Match " << i << " com " << j << " score " << distError << endl;
        }
    }
*/
}

void GraphMatcher::buscaMatch(GraphLink *link, vector<GraphLink *> vertex, int star, int end, float ang)
{
/*
    float angError = fabs(link->invAngle - vertex[star]->invAngle),
          distError = fabs(link->p - vertex[star]->p);

    if(angError < angThreshold)
    {
        if

    }
*/
}


GraphMatcher::GraphMatcher(const SonarConfig &config)
{
    distThreshold= config.graphMatcher_distThreshold;
    angThreshold= config.graphMatcher_angThreshold;
    stdXWeightError= config.graphMatcher_stdXWeightError;
    stdYWeightError= config.graphMatcher_stdYWeightError;
    stdIWeightError= config.graphMatcher_stdIWeightError;
    angWeightError= config.graphMatcher_angWeightError;
    invAngWeightError= config.graphMatcher_invAngWeightError;
    edgeDistanceWeightError= config.graphMatcher_edgeDistanceWeightError;
    drawVertexMatching= config.graphMatcher_drawVertexMatching;
}

void GraphMatcher::matche(SonarDescritor *sd)
{
}

void GraphMatcher::matche(SonarDescritor *sd1, SonarDescritor *sd2, vector< pair<unsigned, unsigned> > &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph , &g2 = sd2->graph;

    float score, bestScore = FLT_MAX, angCorrection=0.f, tAngCorrection,
          edgeDistError=0.f, invAngError=0.f;
    int vertexIDMatch, edgeCut=0;
    bool foundVertexPair=false;

    vector<GraphLink*> cu, cv; // Tempoarary Copy of Edges

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1.size() <= 1) continue;
        bestScore = FLT_MAX;
        vertexIDMatch=-1;
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2.size() <= 1) continue;

            // Copy the vertex, warnning it need to be FREE! after use
            dinamicCopyVertex(g1[u],cu);
            dinamicCopyVertex(g2[v],cv);

            foundVertexPair = findPairOfVertex(cu,cv, &edgeDistError , &invAngError , &tAngCorrection,&edgeCut);

            if(foundVertexPair)
            {
                //Compute the vertex score
                score = edgeDistError * edgeDistanceWeightError +
                        invAngError * invAngWeightError +
                        fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeightError +
                        fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeightError +
                        fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeightError +
                        fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeightError
                        ;

                if(drawVertexMatching)
                {
                    int w=1800 , h=900;
                    Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                    drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                    drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
                    imshow("VertexMatch",colorImg);
                    waitKey();
                }


                if(bestScore > score)
                {
                    bestScore = score;
                    vertexIDMatch = v;
                    angCorrection = tAngCorrection;
                }
            }

            // Free the dinamic vertex copy
            freeVertex(cu);
            freeVertex(cv);
        }

        if(vertexIDMatch >=0)
        {
            vertexMatch.push_back(pair<unsigned,unsigned>(u,vertexIDMatch));
            cout << "Match " << u << " com " << vertexIDMatch << " score " << bestScore
                 << " AngCorrection " << angCorrection
                 << " translacao = " << sd1->gaussians[u].x - sd2->gaussians[vertexIDMatch].x
                 << " , " << sd1->gaussians[u].y - sd2->gaussians[vertexIDMatch].y << endl;
        }
    }
}

void GraphMatcher::matche2(SonarDescritor *sd1, SonarDescritor *sd2, vector< pair<unsigned, unsigned> > &vertexMatch)
{
/*
    vector< vector<GraphLink*> > &g1 = sd1->graph , &g2 = sd2->graph;
    vector<VertexMatch*> mg1[g1.size()], mg2[g2.size()];


    float score, angCorrection;
    int vertexIDMatch;

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1.size() <= 1) continue;

        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2.size() <= 1) continue;

            score = compVertex(g1[u],g2[v],&angCorrection);

            if(score < FLT_MAX)
            {
                mg1[u].push_back(new VertexMatch(score,angCorrection,v));
                mg2[v].push_back(new VertexMatch(score,angCorrection,u));
            }
        }

        sort(mg1[u].begin(), mg1[u].end(), compVertexMatch);
    }

    for(unsigned v = 0 ; v < g2.size() ; v++)
    {
        sort(mg2[v].begin(),mg2[v].end(), compVertexMatch);
    }

//    if(vertexIDMatch >=0)
//    {
//        vertexMatch.push_back(pair<unsigned,unsigned>(u,vertexIDMatch));
//        cout << "Match " << u << " com " << vertexIDMatch << " score " << bestScore
//             << " AngCorrection " << angCorrection
//             << " translacao = " << sd1->gaussians[u].x - sd2->gaussians[vertexIDMatch].x
//             << " , " << sd1->gaussians[u].y - sd2->gaussians[vertexIDMatch].y << endl;
//    }
*/
}

void GraphMatcher::drawMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch, Mat colorImg)
{
    SonarDescritor::drawDescriptorColor(colorImg,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    SonarDescritor::drawDescriptorColor(colorImg,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].first , v = vertexMatch[i].second;
        line(colorImg,
             Point2f(sd1->gaussians[u].x, sd1->gaussians[u].y),
             Point2f(sd2->gaussians[v].x, sd2->gaussians[v].y),
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
    float drawAng = vu.ang;
    if(vu.dx > vu.dy)
    {
        if(drawAng<0.f)
            drawAng =   90.f + vu.ang;
        else drawAng = -90.f + vu.ang;
    }

    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vu.dx, vu.dy),
            drawAng,
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
    drawAng = vv.ang;
    if(vv.dx > vv.dy)
    {
        if(drawAng<0.f)
            drawAng =   90.f + vv.ang;
        else drawAng = -90.f + vv.ang;
    }

    ellipse(colorImg,
            Point2f(vx, vy),
            Size2f(vv.dx, vv.dy),
            drawAng,
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
