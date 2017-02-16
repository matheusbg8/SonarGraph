#include "SonarDescritor.h"
#include <cfloat>
#include <iostream>
#include <cstdio>

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
                     color,2);
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
