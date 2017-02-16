#include "SonarDescritor.h"
#include <cfloat>
#include <iostream>
#include <cstdio>

#include "Drawing/Drawing.h"

using namespace std;

SonarDescritor::SonarDescritor()
{
}

SonarDescritor::~SonarDescritor()
{
    for(unsigned i = 0 ; i < graph.size(); i++)
    {
        for(unsigned j = 0 ; j < graph[i].size(); j++)
            delete graph[i][j];
    }

    graph.clear();
}

void SonarDescritor::drawDescriptor(Mat colorImg, SonarDescritor *sd, bool drawEdges,
                                    bool drawEdgesAngle, bool drawEdgesInvAngle,
                                    bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
{
    float ri = 0x00, rf = 0xff, dr = rf-ri, r,
          gi = 0x00, gf = 0x00, dg = gf-gi, g,
          bi = 0xff, bf = 0x00, db = bf-bi, b,
          mt=FLT_MAX, Mt=0.f, dt, t;

    // Search bigest pixel instensity
    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        if(Mt < sd->gaussians[i].z)
            Mt = sd->gaussians[i].z;
        if(mt > sd->gaussians[i].z)
            mt = sd->gaussians[i].z;
    }

//    Mt = 120;
//    cout << "Mt " << Mt << endl;
    dt = Mt-mt;

    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        t = (sd->gaussians[i].z - mt)/dt;
        r = dr*t + ri;
        g = dg*t + gi;
        b = db*t + bi;

//        cout << "color " << t << " ( " << r << " , " << g << " , " << b << ")" << endl;
        char tempStr[50];

        for(unsigned j =0; j < sd->graph[i].size() ; j++)
        {
            GraphLink *link = sd->graph[i][j];

            if(drawEdges)
            {
                line(colorImg,Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[link->dest].x, sd->gaussians[link->dest].y),
                     Scalar(b,g,r),2);
            }

            if(drawEdgesAngle)
            {
                sprintf(tempStr,"%.2f", link->ang);
                putText(colorImg, tempStr,
                        Point2f(sd->gaussians[i].x + 0.25*link->p*sin(link->ang*M_PI/180.f),
                                sd->gaussians[i].y - 0.25*link->p*cos(link->ang*M_PI/180.f)),
                        FONT_HERSHEY_COMPLEX, 0.4,
                        Scalar(b, g, r), 1, 8);
            }

            if(drawEdgesInvAngle)
            {
                line(colorImg,
                     Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f),
                             sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f)),
                     Scalar(0,0,255),2);

                sprintf(tempStr,"%.2f", link->invAngle);
                int baseLine = 0;
                Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.5,1,&baseLine);

                putText(colorImg, tempStr,
                        Point2f( sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f) - (textSize.width / 2.f),
                                 sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f) + (textSize.height /2.f) ),
                        FONT_HERSHEY_COMPLEX, 0.5,
                        Scalar(0, 255, 0), 1, 8);
            }
        }

        if(drawElipses)
        {
            float drawAng = sd->gaussians[i].ang;
            if(sd->gaussians[i].dx > sd->gaussians[i].dy)
            {
                if(drawAng<0.f)
                    drawAng =   90.f + sd->gaussians[i].ang;
                else drawAng = -90.f + sd->gaussians[i].ang;
            }

            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    drawAng,//180.0*atan2(sd->gaussians[i].dy,sd->gaussians[i].dx)/M_PI,
                    0.0 , 360.0,
                    Scalar(b,g,r),
                    2 , 8 , 0);
        }

        if(drawVertexID)
        {
            sprintf(tempStr, "ID %d", i);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+20, sd->gaussians[i].y+20),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(0,0,255),2);
        }

        if(drawElipsesAngle)
        {
            sprintf(tempStr, "%.2f", sd->gaussians[i].ang);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+40, sd->gaussians[i].y),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(b,g,r),2);

            line(colorImg,
                 Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                 Point2f(sd->gaussians[i].x + 50.f*sin(sd->gaussians[i].ang*M_PI/180.f),
                         sd->gaussians[i].y - 50.f*cos(sd->gaussians[i].ang*M_PI/180.f)),
                 Scalar(0,255,0),2);
        }

    }
}

void SonarDescritor::drawDescriptorColor(Mat colorImg, SonarDescritor *sd, const Scalar &color, const Scalar &txtColor, bool drawEdges, bool drawEdgesAngle, bool drawEdgesInvAngle, bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
{
    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        char tempStr[50];

        for(unsigned j =0; j < sd->graph[i].size() ; j++)
        {
            GraphLink *link = sd->graph[i][j];

            if(drawEdges)
            {
                line(colorImg,Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[link->dest].x, sd->gaussians[link->dest].y),
                     color,1);
            }

            if(drawEdgesAngle)
            {
                sprintf(tempStr,"%.2f", link->ang);
                putText(colorImg, tempStr,
                        Point2f(sd->gaussians[i].x + 0.25*link->p*sin(link->ang*M_PI/180.f),
                                sd->gaussians[i].y - 0.25*link->p*cos(link->ang*M_PI/180.f)),
                        FONT_HERSHEY_COMPLEX, 0.4,
                        txtColor, 1, 8);
            }

            if(drawEdgesInvAngle)
            {
                line(colorImg,
                     Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f),
                             sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f)),
                     txtColor,2);

                sprintf(tempStr,"%.2f", link->invAngle);
                int baseLine = 0;
                Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.5,1,&baseLine);

                putText(colorImg, tempStr,
                        Point2f( sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f) - (textSize.width / 2.f),
                                 sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f) + (textSize.height /2.f) ),
                        FONT_HERSHEY_COMPLEX, 0.5,
                        txtColor, 1, 8);
            }
        }

        if(drawElipses)
        {
            float drawAng = sd->gaussians[i].ang;
            if(sd->gaussians[i].dx > sd->gaussians[i].dy)
            {
                if(drawAng<0.f)
                    drawAng =   90.f + sd->gaussians[i].ang;
                else drawAng = -90.f + sd->gaussians[i].ang;
            }

            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    drawAng,//180.0*atan2(sd->gaussians[i].dy,sd->gaussians[i].dx)/M_PI,
                    0.0 , 360.0,
                    color,
                    2 , 8 , 0);
        }

        if(drawVertexID)
        {
            sprintf(tempStr, "ID %d", i);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+20, sd->gaussians[i].y+20),
                   FONT_HERSHEY_COMPLEX,0.5,
                   txtColor,2);
        }

        if(drawElipsesAngle)
        {
            sprintf(tempStr, "%.2f", sd->gaussians[i].ang);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+40, sd->gaussians[i].y),
                   FONT_HERSHEY_COMPLEX,0.5,
                   txtColor,2);

            line(colorImg,
                 Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                 Point2f(sd->gaussians[i].x + 50.f*sin(sd->gaussians[i].ang*M_PI/180.f),
                         sd->gaussians[i].y - 50.f*cos(sd->gaussians[i].ang*M_PI/180.f)),
                 txtColor,2);
        }

    }
}

void SonarDescritor::drawVertex(Mat colorImg, SonarDescritor *sd,unsigned vertexID,const Rect &drawingArea, const Scalar &color, const Scalar &txtColor)
{
    float ux = drawingArea.x + drawingArea.width/2.f,
          uy = drawingArea.y + drawingArea.height/2.f,
          mr = min(drawingArea.width,drawingArea.height)/2.f;

    char tempStr[40];
    const Gaussian &vertex = sd->gaussians[vertexID];
    const vector<GraphLink*> &edges = sd->graph[vertexID];

    // Find longest edge to normalization after
    float maxEdist = 0.f;
    for(unsigned i =0; i < edges.size() ; i++)
    {
        if(maxEdist < edges[i]->p)
            maxEdist = edges[i]->p;
    }

    // Draw elipse u
    float drawAng = vertex.ang;
    if(vertex.dx > vertex.dy)
    {
        if(drawAng<0.f)
            drawAng =   90.f + vertex.ang;
        else drawAng = -90.f + vertex.ang;
    }

    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vertex.dx, vertex.dy),
            drawAng,
            0.0 , 360.0,
            color,
            2 , 8 , 0);

    line(colorImg,Point2f(ux, uy),
         Point2f(ux + mr*sin(vertex.ang*M_PI/180.f),
                 uy - mr*cos(vertex.ang*M_PI/180.f)),
         Scalar(0,0,255),2);

    sprintf(tempStr, "ID %d, %.2f", vertexID,vertex.ang);
    putText(colorImg,tempStr,
           Point2f(20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);


    // Draw edges u
    for(unsigned i =0; i < edges.size() ; i++)
    {
        float r = mr*(edges[i]->p/maxEdist),
              ang = edges[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(ux, uy),
             Point2f(ux + r*sin(ang),
                     uy - r*cos(ang)),
             color,2);

        // Draw Edge's angle
        sprintf(tempStr,"p%.2f ; ar%.2f ; a%.2f ; i%.2f", edges[i]->p , edges[i]->rAng,edges[i]->ang,edges[i]->invAngle);

        int baseLine = 0;
        Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.6,2,&baseLine);

        putText(colorImg, tempStr,
                Point2f(ux + r*sin(ang)-textSize.width/2,
                        uy - r*cos(ang)),
                FONT_HERSHEY_COMPLEX, 0.6,
                txtColor, 2, 8);
    }
}

void SonarDescritor::drawVertexMatch(Mat bgrImg1, Mat bgrImg2,
                                     SonarDescritor &sd1, SonarDescritor &sd2,
                                     vector<SpatialMatchInfo> &matchInfo,
                                     const Scalar &color, const Scalar &txtColor,
                                     int thickness)
{
    typedef pair<float,float> PFF;
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<PFF,PUU> PUUPFF;


    // For each iMatch match between gaussian u in frame 1 and gaussian v in frame 2
    for(unsigned iMatch = 0 ; iMatch < matchInfo.size(); iMatch++)
    {
        SpatialMatchInfo &mi = matchInfo[iMatch];
        vector<PUUPFF> &emi = mi.edgeMatchInfo;

        vector<GraphLink*> &edgeFr1 = sd1.graph[mi.uID],
                           &edgeFr2 = sd2.graph[mi.vID];

        Gaussian &guFr1 = sd1.gaussians[mi.uID],
                 &guFr2 = sd2.gaussians[mi.vID];


        // For each edge match i between gaussian u in frame 1 and gaussian v in frame 2
        for(unsigned i=0 ; i < emi.size(); i++)
        {
            // Take dest gaussian of i match edge
            Gaussian &gvFr1 = sd1.gaussians[edgeFr1[emi[i].first.first]->dest],
                     &gvFr2 = sd2.gaussians[edgeFr2[emi[i].first.second]->dest];

            line(bgrImg1,
                 Point2f(guFr1.x , guFr1.y),
                 Point2f(gvFr1.x, gvFr1.y),
                 Drawing::color[iMatch%Drawing::nColor],thickness);

            line(bgrImg2,
                 Point2f(guFr2.x , guFr2.y),
                 Point2f(gvFr2.x, gvFr2.y),
                 Drawing::color[iMatch%Drawing::nColor],thickness);
        }
    }
}

void SonarDescritor::clearLinks()
{
    for(unsigned gi = 0; gi < graph.size() ; gi++)
    {
        graph[gi].clear();
    }
}

void SonarDescritor::createGraph(float graphLinkDistance)
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

                if(dt > 180.f) dt-=180.f;
                else dt+= 180.f;

                edt = dt - gaussians[j].ang;
                if(edt < 0.f) edt += 360.f;

                graph[j].push_back(new GraphLink(dt,edt,0,d,i));
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
