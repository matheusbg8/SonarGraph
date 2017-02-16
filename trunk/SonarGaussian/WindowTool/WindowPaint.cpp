#include "WindowPaint.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cstdio>

#include "WindowTool/PaintFeatures/PaintFeature.h"


void WindowPaint::propagateMouseDragEvent(int x, int y, Point2f p)
{
    for(unsigned i = 0 ; i < pfts.size() ; i++)
    {
        pfts[i]->mouseDrag(x,y);
    }
}

void WindowPaint::propagateMouseDragEndEvent(int x, int y, Point2f p)
{
    for(unsigned i = 0 ; i < pfts.size() ; i++)
    {
        pfts[i]->mouseDragEnd(x,y);
    }
}

void WindowPaint::propagateDrawEndImg(Mat &img8bIts)
{
    for(unsigned i = 0 ; i < pfts.size() ; i++)
    {
        pfts[i]->drawEndImg(gray_8bits);
    }
}

void WindowPaint::propagateDrawEndContour(vector<Point2f> &contour)
{
    for(unsigned i = 0 ; i < pfts.size() ; i++)
    {
        pfts[i]->drawEndContour(contour);
    }
}

void WindowPaint::computAndDrawImageMoments(Mat screen,const Point2f &p)
{
    /// Compute moments
    Moments mu = moments(gray_8bits,false);

    ///  Get the mass centers:
    Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
    char str[200];
    Scalar txtColor(0,0,255);

    sprintf(str, "Area = %lg" , mu.m00);
    putText(screen,str,
           Point2f(p.x + 10,p.y + 10),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    sprintf(str, "Center of Mass = (%lg,%lg)" , mc.x, mc.y);
    putText(screen,str,
           Point2f(p.x + 10,p.y + 30),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    /// Draw Center
    circle( screen, mc, 4, txtColor, -1, 8, 0 );

     double hu[7];
     HuMoments(mu,hu);

     sprintf(str, "Momentos HUs:");
     putText(screen,str,
            Point2f(p.x + 10,p.y + 50),
            FONT_HERSHEY_COMPLEX,0.5,
            txtColor,1);

     for(unsigned j = 0 ; j < 7; j++)
     {
         sprintf(str, "%lg",hu[j]);
         putText(screen,str,
                 Point2f(p.x + 10,p.y + 65+j*15),
                FONT_HERSHEY_COMPLEX,0.5,
                txtColor,1);
     }
}

void WindowPaint::computAndDrawContournsMoments(Mat screen, const Point2f &p)
{
    if(contours.size() == 0)
        return;

    /// Compute moments
    Moments mu = moments(contours[0],false);

    ///  Get the mass centers:
    Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
    char str[200];
    Scalar txtColor(0,255,0);

    sprintf(str, "Area = %lg" , mu.m00);
    putText(screen,str,
           Point2f(p.x + 10,p.y + 10),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    sprintf(str, "Center of Mass = (%lg,%lg)" , mc.x, mc.y);
    putText(screen,str,
           Point2f(p.x + 10,p.y + 30),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    /// Draw Center
    circle( screen, mc, 4, txtColor, -1, 8, 0 );

    double hu[7];
    HuMoments(mu,hu);

    sprintf(str, "Momentos HUs:");
    putText(screen,str,
           Point2f(p.x + 10,p.y + 50),
           FONT_HERSHEY_COMPLEX,0.5,
           txtColor,1);

    for(unsigned j = 0 ; j < 7; j++)
    {
        sprintf(str, "%lg",hu[j]);
        putText(screen,str,
               Point2f(p.x + 10,p.y + 65+j*15),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,1);
    }

}

WindowPaint::WindowPaint():
    nContourns(0),
    mouseDown(false),
    windowSize(600,600),
    sourceWindowName("WindowPaint")
{

}

void WindowPaint::start()
{
    /// Create Window
    namedWindow( sourceWindowName.c_str(), CV_WINDOW_AUTOSIZE );

    /// Initialize image with black background
    resetScren();
    refreshScreen();

    /// Create mouse callback
    setMouseCallback(sourceWindowName,windowPaint_mouse_callback, this);
    /// Start keyborad
    readKeyboar();
}

void WindowPaint::readKeyboar()
{
    char c = 0;
    while(c != 27)
    {
        if(c == 'r')
        {
            resetScren();
            refreshScreen();
        }

        c = waitKey();
    }
}

void WindowPaint::resetScren()
{
    gray_8bits = Mat(windowSize.width,windowSize.height,CV_8UC1, Scalar(0));
    contours.clear();
    nContourns=0;
}

void WindowPaint::refreshScreen()
{
    // About contourns hierarchy
    // http://docs.opencv.org/3.1.0/d9/d8b/tutorial_py_contours_hierarchy.html#gsc.tab=0

    Mat screen;
    cvtColor(gray_8bits,screen,CV_GRAY2BGR);

    computAndDrawImageMoments(screen, Point2f(0,300));

    computAndDrawContournsMoments(screen, Point2f(0,0));
    /// Show in a window
    imshow(sourceWindowName, screen);
}

void WindowPaint::mouseCalback(int event, int x, int y, int flags)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        lasMousePosition = Point2f(x,y);
        mouseDown = true;
        contours.push_back(vector<Point2f>());
    break;
    case CV_EVENT_LBUTTONUP:
        mouseDown = false;
        nContourns++;
    break;
    case CV_EVENT_MOUSEMOVE:
        if(mouseDown == true)
        {
//            cout << "Mouse posution ("
//                 << x << " , " << y << ")"
//                 << endl;

            currentMousePosition = Point2f(x,y);

            if(gray_8bits.at<ushort>(y,x) == 0)
            {
                line(gray_8bits,lasMousePosition ,
                     currentMousePosition, Scalar(255),1);
            }else
            {
                line(gray_8bits,lasMousePosition ,
                     currentMousePosition, Scalar(255),1);
            }
            contours[nContourns].push_back(currentMousePosition);

            propagateDrawEndContour(contours[nContourns]);
            propagateDrawEndImg(gray_8bits);

            refreshScreen();
            lasMousePosition = currentMousePosition;
        }
    break;
    }
}

void WindowPaint::addPaintFeature(PaintFeature *pf)
{
    pfts.push_back(pf);
}

void windowPaint_mouse_callback(int event, int x, int y, int flags, void *object)
{
    WindowPaint *obj = (WindowPaint*) object;
    obj->mouseCalback(event,x,y,flags);
}
