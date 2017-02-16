#include "Drawing.h"

#include "Drawing/Chart.h"

// Old colors
//const Scalar Drawing::color[6] =
//{
//    Scalar(255 ,   0 ,   0),
//    Scalar(  0 , 255 ,   0),
//    Scalar(  0 ,   0 , 255),
//    Scalar(255 , 255 ,   0),
//    Scalar(255 ,   0 , 255),
//    Scalar(  0 , 255 , 255)
//};

//const unsigned Drawing::nColor =6;

const Scalar Drawing::color[12] =
{
    Scalar(  0 ,   0 , 255),
    Scalar(  0 , 128 , 255),
    Scalar(  0 , 255 , 255),
    Scalar(  0 , 255 , 128),
    Scalar(  0 , 255 ,   0),
    Scalar(255 ,   0 ,   0),
    Scalar(255 ,   0 , 127),
    Scalar(255 ,   0 , 255),
    Scalar(127 ,   0 , 255),
    Scalar(128 , 255 ,   0),
    Scalar(255 , 255 ,   0),
    Scalar(255 , 128 ,   0)
};

const unsigned Drawing::nColor =12;


void Drawing::drawImgsTogether(const Size &windowSize, Mat &leftBGR8img, Mat &rightBGR8img, Mat &result,
                               Scalar_<float> *el,Scalar_<float> *er)
{
    if(result.cols != windowSize.width || result.rows != windowSize.height)
    {
        result = Mat(windowSize.height, windowSize.width,CV_8UC3);
        cout << "Allocing img for drawing together" << endl;
    }

    Size imSZ(windowSize.width/2,windowSize.height);

    // Temprary BGR resized img
    Mat lImg, rImg;

    // Resizing
    resize(leftBGR8img,lImg,imSZ);
    resize(rightBGR8img,rImg,imSZ);

    lImg.copyTo(result(Rect(0         , 0 , imSZ.width , imSZ.height)));
    rImg.copyTo(result(Rect(imSZ.width, 0 , imSZ.width , imSZ.height)));

    // Compute scale and offset betwen two images
    el->val[0] = (float)imSZ.width/ (float)leftBGR8img.cols, el->val[1] = (float)imSZ.height / (float)leftBGR8img.rows;
    el->val[2] = 0 , el->val[3] = 0;
    er->val[0] = (float)imSZ.width/ (float)rightBGR8img.cols, er->val[1] = (float)imSZ.height / (float)rightBGR8img.rows;
    er->val[2] = imSZ.width, er->val[3] = 0;
}

void Drawing::convertFrameMatch(vector<vector<pair<unsigned, unsigned> > > &frMatch, unsigned fr, vector<pair<unsigned,unsigned> > *matchs)
{
    matchs->clear();
    for(unsigned u = 0 ; u < frMatch.size(); u++)
    {
        for(unsigned i = 0 ; i < frMatch[u].size() ; i++)
        {
            if(frMatch[u][i].first == fr)
            {
                matchs->push_back(pair<unsigned,unsigned>(u,frMatch[u][i].second));
            }
        }
    }
}

void Drawing::drawMatchings(Mat &BGRImg, const Scalar_<float> &el, const Scalar_<float> &er,
                            vector<Gaussian> &gl, vector<Gaussian> &gr,
                            vector<MatchInfo> &matchs, const Scalar &color)
{
    for(unsigned i = 0 ; i < matchs.size(); i++)
    {
        unsigned u = matchs[i].uID,
                 v = matchs[i].vID;

        line(BGRImg,
             Point(el.val[2] + el.val[0]*gl[u].x, el.val[3] + el.val[1]*gl[u].y),
             Point(er.val[2] + er.val[0]*gr[v].x, er.val[3] + er.val[1]*gr[v].y),
             color,2);
    }
}

void Drawing::drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                            const Point2f &begin, const Point2f &end,
                            const Scalar &color, int thickness)
{
    line(BGRImg,
         Point(el.val[2] + el.val[0]*begin.x, el.val[3] + el.val[1]*begin.y),
         Point(er.val[2] + er.val[0]*end.x, er.val[3] + er.val[1]*end.y),
         color,thickness);
}

void Drawing::drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                            const vector<Point2f> &begin, const vector<Point2f> &end,
                            const Scalar &color, int thickness)
{
    for(unsigned i = 0 ; i < begin.size(); i++)
    {
        line(BGRImg,
             Point(el.val[2] + el.val[0]*begin[i].x, el.val[3] + el.val[1]*begin[i].y),
             Point(er.val[2] + er.val[0]*end[i].x, er.val[3] + er.val[1]*end[i].y),
             color,thickness);
    }
}

void Drawing::drawGaussian(Mat &img, Gaussian &g, const Scalar &lineColor,
                           const Scalar &txtColor, int id, bool drawElipses,
                           bool drawVertexID, bool drawElipsesAngle)
{
    if(drawElipses)
    {
        ellipse(img,
                Point2f(g.x, g.y),
                Size2f(g.dx, g.dy),
                g.ang,
                0.0 , 360.0,
                lineColor,
                2 , 8 , 0);
    }

    char tempStr[200];
    if(drawVertexID)
    {
        sprintf(tempStr, "ID %d", id);
        putText(img,tempStr,
               Point2f(g.x+20, g.y+20),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);
    }

    if(drawElipsesAngle)
    {
        double sinT = sin(g.ang*M_PI/180.f),
              cosT = cos(g.ang*M_PI/180.f),
              sinO = sin((g.ang-90)*M_PI/180.f),
              cosO = cos((g.ang-90)*M_PI/180.f),
              dx2 = 20+g.dx, dy2 = g.dy;

        sprintf(tempStr, "%.2f", g.ang);
        putText(img,tempStr,
               Point2f(g.x+ dx2 *sinO, g.y - dx2 * cosO),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);

        line(img,
             Point2f(g.x + dy2 *sinT, g.y - dy2 *cosT),
             Point2f(g.x + (dy2+50.f)*sinT,
                     g.y - (dy2+50.f)*cosT),
             txtColor,2);
    }
}

void Drawing::drawGaussianText(Mat &img, Gaussian &g,
                               const Scalar &txtColor, const char *txt,
                               const Point2f &txtPosition)
{
    putText(img,txt,
           Point2f(g.x, g.y) + txtPosition,
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,2);
}

void Drawing::drawGaussians(Mat &img, vector<Gaussian> &gs,
                            const Scalar &lineColor, const Scalar &txtColor,
                            bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
{
    for(unsigned i = 0 ; i < gs.size() ; i++)
    {
        Drawing::drawGaussian(img,gs[i],lineColor,txtColor,i,
                              drawElipses,drawVertexID,drawElipsesAngle);
    }
}


/**
 * @brief Draw the four point which form a gaussian
 *
 * @param img
 * @param g
 * @param id
 * @param dotsColor
 * @param txtColor
 * @param drawVertexID
 * @param drawElipsesAngle
 * @param linkDots
 */
void Drawing::drawGaussianDots(Mat &img, Gaussian &g,
                               int id,
                               const Scalar &dotsColor, const Scalar &txtColor,
                               bool drawVertexID, bool drawElipsesAngle,
                               bool linkDots)
{
    Point2f pts[4];
    g.clockWisePoints(pts[0],pts[1],pts[2],pts[3]);

    if(linkDots)
    for(unsigned i = 0 ; i < 4; i++)
    {
        line(img,pts[i],pts[(i+1)%4],txtColor);
    }

    for(unsigned i = 0 ; i < 4; i++)
    {
        circle(img,pts[i],2,dotsColor,-1);
    }

    char tempStr[200];
    if(drawVertexID)
    {
        sprintf(tempStr, "ID %d", id);
        putText(img,tempStr,
               Point2f(g.x+20, g.y+20),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);
    }

    if(drawElipsesAngle)
    {
        sprintf(tempStr, "%.2f", g.ang);
        putText(img,tempStr,
               Point2f(g.x+40, g.y),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);

        line(img,
             Point2f(g.x, g.y),
             Point2f(g.x + 50.f*sin(g.ang*M_PI/180.f),
                     g.y - 50.f*cos(g.ang*M_PI/180.f)),
             txtColor,2);
    }
}

void Drawing::drawGaussiansDots(Mat &img, vector<Gaussian> &gs,
                                const Scalar &dotsColor, const Scalar &txtColor,
                                bool drawVertexID, bool drawElipsesAngle,
                                bool linkDots)
{
    for(unsigned i = 0 ; i < gs.size() ; i++)
    {
        Drawing::drawGaussianDots(img,gs[i],i,dotsColor,txtColor,
                              drawVertexID,drawElipsesAngle,linkDots);
    }
}

void Drawing::polarPlot(Mat &bgrResult, vector<double> &normData, Size imgSize, const vector<string> &axisLabels)
{
    Size plotSize(imgSize.width*0.5f, imgSize.height*0.5f);

    bgrResult = Mat(imgSize.height,imgSize.width,CV_8UC3,Scalar(0,0,0));

    float maxRadius = std::min(plotSize.width,plotSize.height)/2.f,
          labelRadius = std::min(imgSize.width*0.85f, imgSize.height*0.85f),
          bearing = 2*M_PI / (normData.size());

    float cRadAng = 0.f;
    Point2f c(imgSize.width/2.f,imgSize.height/2.f);

    vector<Point2f> axisVectors(normData.size());

    // Compute axis vectors
    for(unsigned i = 0 ; i < normData.size(); i++)
    {
        axisVectors[i] = Point(maxRadius*sin(cRadAng) , maxRadius*cos(cRadAng) );
        cRadAng += bearing;
    }

    // Draw data axis and labels
    for(unsigned i = 0 ; i < normData.size(); i++)
    {
        // Draw axi
        line(bgrResult, c,
                c + axisVectors[i],
             Scalar(211,211,211),1);

        // Draw data
        line(bgrResult, c,
                c + normData[i]*axisVectors[i],
             Drawing::color[i%Drawing::nColor],4);

        cRadAng += bearing;
    }

    for(unsigned i = 0 ; i < normData.size(); i++)
    {
        unsigned next = (i+1)%normData.size();
        line(bgrResult, c + normData[i]*axisVectors[i],
                c + normData[next]*axisVectors[next],
             Scalar(211,211,211),2);
    }


    for(unsigned i = 0 ; i < normData.size() && i < axisLabels.size(); i++)
    {
    // Draw Labels
        int baseLine;
        Size textSize = getTextSize(axisLabels[i],FONT_HERSHEY_COMPLEX, 0.6,2,&baseLine);

        putText(bgrResult, axisLabels[i],
                c + 1.25f*axisVectors[i] - Point2f(textSize.width/2.f,0),
                FONT_HERSHEY_COMPLEX, 0.6,
                Drawing::color[i%Drawing::nColor], 2, 8);
    }
}

void Drawing::plot(Mat &result, const Size &size,
                   vector<float> &x,vector<float> &y,
                   const Scalar &color, Scalar *plotInfo,
                   int thickness)
{
    if(x.size() != y.size())
    {
        cout << "Plot error, x and y need to the same size" << endl;
        return;
    }

    double xMin, xMax, yMin,yMax;

    minMaxLoc(x,&xMin, &xMax);
    minMaxLoc(y,&yMin, &yMax);

    cout << "x-> " << xMin << " , " << xMax << endl;
    cout << "y-> " << yMin << " , " << yMax << endl;

    result = Mat(size.height,size.width,
                 CV_8UC3, Scalar(255,255,255)
                );

    Scalar pI(
                size.width/(xMax - xMin),
                size.height/(yMax - yMin),
                -xMin , -yMin
                );

    for(unsigned i = 1; i < x.size() ; i++)
    {
        line(result,

             Point2f(pI.val[0]*(x[i-1]+pI.val[2]),
                     pI.val[1]*(y[i-1]+pI.val[3])
                    ),

             Point2f(pI.val[0]*(x[i]+pI.val[2]),
                     pI.val[1]*(y[i]+pI.val[3])
                     ),

             color, thickness);
    }
    if(plotInfo!=0x0)
        *plotInfo  = pI;
}

void Drawing::makePlotInfo(Mat &result, const Size &size,
                           float xMin, float xMax, float yMin, float yMax,
                           Scalar &plotInfo)
{
    result = Mat(size.height,size.width,
                 CV_8UC3, Scalar(255,255,255)
                );

    plotInfo = Scalar (
                size.width/(xMax - xMin),
                size.height/(yMax - yMin),
                -xMin , -yMin
                );
}

void Drawing::holdPlot(const Scalar &plotInf, Mat &result,
                       vector<float> &x,vector<float> &y,
                       const Scalar &color, int thickness)
{
    if(x.size() != y.size())
    {
        cout << "Plot error, x and y need to the same size" << endl;
        return;
    }

    for(unsigned i = 1; i < x.size() ; i++)
    {
        line(result,

             Point2f(plotInf.val[0]*(x[i-1]+plotInf.val[2]),
                     result.rows-1-plotInf.val[1]*(y[i-1]+plotInf.val[3])
                    ),

             Point2f(plotInf.val[0]*(x[i]+plotInf.val[2]),
                     result.rows-1-plotInf.val[1]*(y[i]+plotInf.val[3])
                     ),

             color, thickness);

    }
}

void Drawing::holdPlotCircle(const Scalar &plotInf, Mat &result,
                             vector<float> &x,vector<float> &y,
                             const Scalar &color, int thickness)
{
    if(x.size() != y.size())
    {
        cout << "Plot error, x and y need to the same size" << endl;
        return;
    }

    for(unsigned i = 0; i < x.size() ; i++)
    {
        circle(result,
             Point2f(plotInf.val[0]*(x[i]+plotInf.val[2]),
                     result.rows -1 - plotInf.val[1]*(y[i]+plotInf.val[3])
                     ),
             7,color, thickness);
    }
}

void Drawing::plot(const Chart &chart, Mat &result)
{
    unsigned w = chart.m_width, h = chart.m_height;

    float    mx = chart.m_minX, Mx = chart.m_maxX,
             my = chart.m_minY, My = chart.m_maxY,
             ex = w/(Mx-mx),
             ey = (h-30)/(My-my),
             dx = 0.f, dy = 15.f;

    result = Mat(chart.m_height, chart.m_width,CV_8UC3, Scalar(255,255,255));

    const vector< vector<pair<float,float> > > &data = chart.data;

    for(unsigned i = 0 ; i < data.size() ;i++)
    {
        const vector<pair<float,float> > &d = data[i];
        const Chart::ChartInfo &dataInfo = chart.dataInfo[i];

        if(dataInfo.plotType == Chart::PLOT_CIRCLE)
        {
            for(unsigned j = 0; j < d.size() ; j++)
            {
                // pair<x, y>
                const pair<float,float> &p = d[j];

                circle(result,
                     Point2f(ex*(p.first-mx)+dx,
                             (result.rows-1) - ey*(p.second-my)-dy
                             ),
                     7,dataInfo.color, dataInfo.thickness);
            }

        }else if(dataInfo.plotType == Chart::PLOT_LINE ||
                 dataInfo.plotType == Chart::PLOT_CONTINUOS_LINE
                )
        {
            unsigned jump = dataInfo.plotType == Chart::PLOT_CONTINUOS_LINE ? 1u : 2u;
            for(unsigned j = 1; j < d.size() ; j+=jump)
            {
                // pair<x, y>
                const pair<float,float> &prv = d[j-1],
                                  &cur = d[ j ];
                line(result,

                 Point2f(ex*(prv.first-mx)+dx,
                         (result.rows-1)-ey*(prv.second-my)-dy
                        ),

                 Point2f(ex*(cur.first-mx)+dx,
                         (result.rows-1)-ey*(cur.second-my)-dy
                        ),

                     dataInfo.color, dataInfo.thickness);

            }
        }

    }

}

void Drawing::drawDescriptor(Mat colorImg, SonarDescritor *sd, bool drawEdges, bool drawEdgesAngle, bool drawEdgesInvAngle, bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
{
    float ri = 0x00, rf = 0xff, dr = rf-ri, r,
          gi = 0x00, gf = 0x00, dg = gf-gi, g,
          bi = 0xff, bf = 0x00, db = bf-bi, b,
          mt=FLT_MAX, Mt=0.f, dt, t;

    // Search bigest pixel instensity
    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        if(Mt < sd->gaussians[i].intensity)
            Mt = sd->gaussians[i].intensity;
        if(mt > sd->gaussians[i].intensity)
            mt = sd->gaussians[i].intensity;
    }

//    Mt = 120;
//    cout << "Mt " << Mt << endl;
    dt = Mt-mt;

    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        t = (sd->gaussians[i].intensity - mt)/dt;
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
            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    sd->gaussians[i].ang,
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

void Drawing::drawDescriptorColor(Mat colorImg, SonarDescritor *sd,
                                  const Scalar &color, const Scalar &txtColor,
                                  bool drawEdges, bool drawEdgesAngle, bool drawEdgesInvAngle,
                                  bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
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
                        Point2f(sd->gaussians[i].x + 0.25*link->p*cos(link->ang*M_PI/180.f),
                                sd->gaussians[i].y - 0.25*link->p*sin(link->ang*M_PI/180.f)),
                        FONT_HERSHEY_COMPLEX, 0.4,
                        txtColor, 1, 8);
            }

            if(drawEdgesInvAngle)
            {
                line(colorImg,
                     Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[i].x + 0.25*link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f),
                             sd->gaussians[i].y - 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f)),
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
            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    sd->gaussians[i].ang,//180.0*atan2(sd->gaussians[i].dy,sd->gaussians[i].dx)/M_PI,
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
            float ex=sd->gaussians[i].x + 50.f*sin(sd->gaussians[i].ang*M_PI/180.f),
                  ey=sd->gaussians[i].y - 50.f*cos(sd->gaussians[i].ang*M_PI/180.f);

            sprintf(tempStr, "%.2f", sd->gaussians[i].ang);
            putText(colorImg,tempStr,
                   Point2f(ex, ey),
                   FONT_HERSHEY_COMPLEX,0.5,
                   txtColor,2);

            line(colorImg,
                 Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                 Point2f(ex,ey),
                 txtColor,2);
        }
    }
}

void Drawing::drawVertex(Mat colorImg, SonarDescritor *sd, unsigned vertexID,
                         const Rect &drawingArea,
                         const Scalar &color, const Scalar &txtColor,
                         bool drawEdgeInfo)
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
    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vertex.dx, vertex.dy),
            vertex.ang,
            0.0 , 360.0,
            color,
            2 , 8 , 0);

    line(colorImg,Point2f(ux, uy),
         Point2f(ux + mr*sin(vertex.ang*M_PI/180.f),
                 uy - mr*cos(vertex.ang*M_PI/180.f)),
         Scalar(0,0,255),2);

    sprintf(tempStr, "ID %d, %.2f", vertexID,vertex.ang);
    putText(colorImg,tempStr,
           Point2f(drawingArea.x+20, drawingArea.y+20),
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
        if(drawEdgeInfo)
        {
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
}

void Drawing::drawVertex(Mat colorImg, SonarDescritor *sd, unsigned vertexID,
                         const Scalar &color, const Scalar &txtColor,
                         bool drawEdgeInfo)
{
    char tempStr[40];
    const Gaussian &vertex = sd->gaussians[vertexID];
    const vector<GraphLink*> &edges = sd->graph[vertexID];

    float ux = vertex.x, uy = vertex.y;

    // Draw elipse u
    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vertex.dx, vertex.dy),
            vertex.ang,
            0.0 , 360.0,
            color,
            2 , 8 , 0);

    // Draw principal axis
    line(colorImg,Point2f(ux, uy),
         Point2f(ux + 30*sin(vertex.ang*M_PI/180.f),
                 uy - 30*cos(vertex.ang*M_PI/180.f)),
         Scalar(0,0,255),2);

    sprintf(tempStr, "ID %d, %.2f", vertexID,vertex.ang);
    putText(colorImg,tempStr,
           Point2f(20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);


    // Draw edges u
    for(unsigned i =0; i < edges.size() ; i++)
    {
        float r = edges[i]->p,
              ang = edges[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(ux, uy),
             Point2f(ux + r*sin(ang),
                     uy - r*cos(ang)),
             color,2);

        // Draw Edge's angle
        if(drawEdgeInfo)
        {
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
}

void Drawing::drawVertexMatch(Mat &colorImg, const Size2i &drawingArea,
                              const SonarDescritor &sd1, const SonarDescritor &sd2,
                              unsigned vertex1Id, unsigned vertex2Id,
                              const vector<MatchInfoWeighted> &edgeMatchInfo,
                              bool drawEdgeInfo, bool drawMatchInfo,
                              const Scalar &color, const Scalar &txtColor,
                              int thickness)
{
    colorImg = Mat(drawingArea.height,drawingArea.width,CV_8UC3,Scalar(0,0,0));

    Size2i subWindowArea(drawingArea.width/2, drawingArea.height);

    float v1x = subWindowArea.width/2.f,
          v1y = subWindowArea.height/2.f,
          v2x = drawingArea.width - v1x,
          v2y = v1y,
          mr = min(drawingArea.width,drawingArea.height)/2.f;

    char tempStr[40];
    const Gaussian &v1 = sd1.gaussians[vertex1Id],
                   &v2 = sd2.gaussians[vertex2Id];

    const vector<GraphLink*> &e1 = sd1.graph[vertex1Id],
                             &e2 = sd2.graph[vertex2Id];

    // Find longest edge to normalization after
    float maxEdist1 = 0.f, maxEdist2 = 0.f;
    unsigned nEdge = max(e1.size(),e2.size());
    for(unsigned i =0; i < nEdge ; i++)
    {
        if(i < e1.size() && maxEdist1 < e1[i]->p)
            maxEdist1 = e1[i]->p;
        if(i < e2.size() && maxEdist2 < e2[i]->p)
            maxEdist2 = e2[i]->p;
    }

    // Draw elipse u
    ellipse(colorImg,
            Point2f(v1x, v1y),
            Size2f(v1.dx, v1.dy),
            v1.ang,
            0.0 , 360.0,
            color,
            2 , 8 , 0);

    ellipse(colorImg,
            Point2f(v2x, v2y),
            Size2f(v2.dx, v2.dy),
            v2.ang,
            0.0 , 360.0,
            color,
            2 , 8 , 0);

    line(colorImg,Point2f(v1x, v1y),
         Point2f(v1x + mr*sin(v1.ang*M_PI/180.f),
                 v1y - mr*cos(v1.ang*M_PI/180.f)),
         txtColor,2);

    line(colorImg,Point2f(v2x, v2y),
         Point2f(v2x + mr*sin(v2.ang*M_PI/180.f),
                 v2y - mr*cos(v2.ang*M_PI/180.f)),
         txtColor,2);

    sprintf(tempStr, "ID %d, %.2f", vertex1Id,v1.ang);
    putText(colorImg,tempStr,
           Point2f(20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    sprintf(tempStr, "ID %d, %.2f", vertex2Id,v2.ang);
    putText(colorImg,tempStr,
           Point2f(subWindowArea.width+20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    // Draw edges
    for(unsigned i =0; i < nEdge ; i++)
    {
        float r, ang;

        if( i < e1.size())
        {
            r = mr*(e1[i]->p/maxEdist1),
            ang = e1[i]->ang*M_PI/180.f;

            line(colorImg,Point2f(v1x, v1y),
                 Point2f(v1x + r*sin(ang),
                         v1y - r*cos(ang)),
                 color,thickness);

            // Draw Edge's angle
            if(drawEdgeInfo)
            {
                sprintf(tempStr,"p%.2f ; ar%.2f ; a%.2f ; i%.2f",
                        e1[i]->p , e1[i]->rAng,e1[i]->ang,e1[i]->invAngle);

                int baseLine = 0;
                Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX,
                                            0.6,2,&baseLine);

                putText(colorImg, tempStr,
                        Point2f(v1x + r*sin(ang)-textSize.width/2,
                                v1y - r*cos(ang)),
                        FONT_HERSHEY_COMPLEX, 0.6,
                        txtColor, 2, 8);
            }
        }

        if( i < e2.size())
        {
            r = mr*(e2[i]->p/maxEdist2),
            ang = e2[i]->ang*M_PI/180.f;

            line(colorImg,Point2f(v2x, v2y),
                 Point2f(v2x + r*sin(ang),
                         v2y - r*cos(ang)),
                 color,thickness);

            // Draw Edge's angle
            if(drawEdgeInfo)
            {
                sprintf(tempStr,"p%.2f ; ar%.2f ; a%.2f ; i%.2f",
                        e2[i]->p , e2[i]->rAng,e2[i]->ang,e2[i]->invAngle);

                int baseLine = 0;
                Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX,
                                            0.6,2,&baseLine);

                putText(colorImg, tempStr,
                        Point2f(v2x + r*sin(ang)-textSize.width/2,
                                v2y - r*cos(ang)),
                        FONT_HERSHEY_COMPLEX, 0.6,
                        txtColor, 2, 8);
            }
        }
    }


    for(unsigned i = 0 ; i < edgeMatchInfo.size();i++)
    {
        const MatchInfoWeighted &m = edgeMatchInfo[i];

        float r = mr*(e1[m.uID]->p/maxEdist1),
              ang = e1[m.uID]->ang*M_PI/180.f;

        line(colorImg,Point2f(v1x, v1y),
             Point2f(v1x + r*sin(ang),
                     v1y - r*cos(ang)),
             Drawing::color[i%Drawing::nColor],thickness);

        if(drawMatchInfo)
        {
            sprintf(tempStr,"mID %u ; fs%.2f",
                    i , m.score);

            int baseLine = 0;
            Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX,
                                        0.4,1,&baseLine);

            putText(colorImg, tempStr,
                    Point2f(v1x + r*sin(ang)-textSize.width/2,
                            v1y - r*cos(ang)),
                    FONT_HERSHEY_COMPLEX, 0.4,
                    txtColor, 1, 8);
        }

        r = mr*(e2[m.vID]->p/maxEdist2),
        ang = e2[m.vID]->ang*M_PI/180.f;

        line(colorImg,Point2f(v2x, v2y),
             Point2f(v2x + r*sin(ang),
                     v2y - r*cos(ang)),
             Drawing::color[i%Drawing::nColor],thickness);

        if(drawMatchInfo)
        {
            sprintf(tempStr,"mID %u ; fs%.2f",
                    i , m.score);

            int baseLine = 0;
            Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX,
                                        0.4,1,&baseLine);

            putText(colorImg, tempStr,
                    Point2f(v2x + r*sin(ang)-textSize.width/2,
                            v2y - r*cos(ang)),
                    FONT_HERSHEY_COMPLEX, 0.4,
                    txtColor, 1, 8);
        }

    }
}

void Drawing::drawGraphVertexMatchs(Mat bgrImg1, Mat bgrImg2,
                              SonarDescritor &sd1, SonarDescritor &sd2,
                              vector<MatchInfoExtended> &matchInfo,
                              int thickness)
{
    // For each iMatch match between gaussian u in frame 1 and gaussian v in frame 2
    for(unsigned iMatch = 0 ; iMatch < matchInfo.size(); iMatch++)
    {
        MatchInfoExtended &mi = matchInfo[iMatch];
        vector<MatchInfoWeighted> &emi = mi.edgeMatchInfo;

        vector<GraphLink*> &edgeFr1 = sd1.graph[mi.uID],
                           &edgeFr2 = sd2.graph[mi.vID];

        Gaussian &guFr1 = sd1.gaussians[mi.uID],
                 &guFr2 = sd2.gaussians[mi.vID];


        // For each edge match i between gaussian u in frame 1 and gaussian v in frame 2
        for(unsigned i=0 ; i < emi.size(); i++)
        {
            // Take dest gaussian of i match edge
            Gaussian &gvFr1 = sd1.gaussians[edgeFr1[emi[i].uID]->dest],
                     &gvFr2 = sd2.gaussians[edgeFr2[emi[i].vID]->dest];

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

void Drawing::drawGraphVertexMatchs(Mat bgrImg1, Mat bgrImg2,
                                    SonarDescritor &sd1, SonarDescritor &sd2,
                                    MatchInfoExtended &matchInfo, int thickness)
{
    vector<MatchInfoWeighted> &emi = matchInfo.edgeMatchInfo;

    vector<GraphLink*> &edgeFr1 = sd1.graph[matchInfo.uID],
                       &edgeFr2 = sd2.graph[matchInfo.vID];

    Gaussian &guFr1 = sd1.gaussians[matchInfo.uID],
             &guFr2 = sd2.gaussians[matchInfo.vID];


    // For each edge match i between gaussian u in frame 1 and gaussian v in frame 2
    for(unsigned i=0 ; i < emi.size(); i++)
    {
        // Take dest gaussian of i match edge
        Gaussian &gvFr1 = sd1.gaussians[edgeFr1[emi[i].uID]->dest],
                 &gvFr2 = sd2.gaussians[edgeFr2[emi[i].vID]->dest];

        line(bgrImg1,
             Point2f(guFr1.x , guFr1.y),
             Point2f(gvFr1.x, gvFr1.y),
             Drawing::color[i%Drawing::nColor],thickness);

        line(bgrImg2,
             Point2f(guFr2.x , guFr2.y),
             Point2f(gvFr2.x, gvFr2.y),
             Drawing::color[i%Drawing::nColor],thickness);
    }

}
