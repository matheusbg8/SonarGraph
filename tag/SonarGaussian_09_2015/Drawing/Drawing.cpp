#include "Drawing.h"

const Scalar Drawing::color[6] =
{
    Scalar(255 ,   0 ,   0),
    Scalar(  0 , 255 ,   0),
    Scalar(  0 ,   0 , 255),
    Scalar(255 , 255 ,   0),
    Scalar(255 ,   0 , 255),
    Scalar(  0 , 255 , 255)
};

const unsigned Drawing::nColor =6;

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
                            vector<Gaussian> &gr, vector<Gaussian> &gl,
                            vector<pair<unsigned,unsigned> > &matchs, const Scalar &color)
{
    for(unsigned i = 0 ; i < matchs.size(); i++)
    {
        unsigned u = matchs[i].first,
                 v = matchs[i].second;

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

void Drawing::drawGaussian(Mat &img, Gaussian &g, const Scalar &lineColor, const Scalar &txtColor, int id, bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
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
