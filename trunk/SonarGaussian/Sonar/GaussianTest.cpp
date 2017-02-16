#include "GaussianTest.h"

GaussianTest::GaussianTest():
    winName("GaussianTest"),
    winSize(800,600)
{
}

void GaussianTest::mainLoop()
{
    char c = 0;

    clear();
    draw();

    while(c != 27) // KEY_ESC
    {
        c = waitKey(0);

        switch(c) // Preview frame
        {
        case 'c': // Clear sample

        break;
        }
    }
}

void GaussianTest::draw()
{
    img = Scalar(0,0,0);

    if(sample.N > 0)
    {

        sample.drawSegment(img, Scalar(255,255,0));
        gaussian.createGaussian3x3(&sample);


        ellipse(img,
                Point2f(gaussian.x, gaussian.y),
                Size2f(gaussian.dx, gaussian.dy),
                gaussian.ang,
                0.0 , 360.0,
                Scalar(0,0,255),
                2 , 8 , 0);

    }
    imshow(winName,img);
}

void GaussianTest::start()
{
    img = Mat(winSize.height, winSize.width, CV_8UC3, Scalar(0,0,0));

    namedWindow(winName, 1);

    setMouseCallback(winName, onGausianTestMouseEvent, this);

    mainLoop();
}

void GaussianTest::clear()
{
    int maxSampleSize= 120000;
    // Initialize/Clear segment
    sample.N = 0;
    sample.MCol = sample.MRow = 0;
    sample.mRow = sample.mCol = 99999;

    // Realocate memory if necsessary
    if(sample.result.rows < 3 || sample.result.cols < maxSampleSize)
    {
        sample.result = Mat(3, maxSampleSize , CV_16UC1, Scalar(0));
    }
}

void GaussianTest::mouseEvent(int event, int x, int y)
{

    if(event == CV_EVENT_LBUTTONDOWN)
    {
        // Register element
        sample.result.at<ushort>(0,sample.N) = y;
        sample.result.at<ushort>(1,sample.N) = x;
        sample.result.at<ushort>(2,sample.N) = 100;
        sample.N++;

        // Take min max col and y
        if(sample.MRow < y)
            sample.MRow = y;
        if(sample.mRow > y)
            sample.mRow = y;
        if(sample.MCol < x)
            sample.MCol = x;
        if(sample.mCol > x)
            sample.mCol = x;

        draw();
    }
}

void onGausianTestMouseEvent(int event, int x, int y, int, void *dt)
{
    GaussianTest *gt = (GaussianTest*) dt;
    gt->mouseEvent(event,x,y);

}
