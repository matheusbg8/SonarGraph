#include "MatchTest.h"

MatchTest::MatchTest():
    winSz(800,600),
    winName("MatchTest")
{

}

void MatchTest::mainLoop()
{
    char c=0;
    while(c != 27)
    {


        c = waitKey();
    }
}

void MatchTest::start()
{
    namedWindow(winName, 1);

    setMouseCallback(winName, onMatchTestMouseEvent, this);

    sd1.gaussians.push_back(
                Gaussian(winSz.width*0.25,winSz.height*0.5,0,1,1,1,0,1)
                );
    sd1.graph.push_back( vector<GraphLink*>());


    sd2.gaussians.push_back(
                Gaussian(winSz.width*0.75,winSz.height*0.5,0,1,1,1,0,1)
                );
    sd2.graph.push_back( vector<GraphLink*>());

    draw();

    mainLoop();
}

void MatchTest::addEdge(SonarDescritor *sd, int x, int y)
{
    sd->gaussians.push_back(
                Gaussian(x,y,0,1,1,1,0,1)
                           );

    sd->graph.push_back(vector<GraphLink*>());
    sd->clearLinks();
    sd->createGraph(max(winSz.width,winSz.height));
}

void MatchTest::draw()
{
    Mat img(winSz.height, winSz.width, CV_8UC3, Scalar(0,0,0));
    SonarDescritor::drawDescriptorColor(img,&sd1,Scalar(0,255,255),Scalar(255,255,255));
    SonarDescritor::drawDescriptorColor(img,&sd2,Scalar(255,255,0),Scalar(255,255,255));

    imshow(winName,img);
}

void MatchTest::mouseEvent(int event, int x, int y)
{
    switch(event)
    {
        case CV_EVENT_LBUTTONUP:
            if(x < winSz.width/2)
            {
                addEdge(&sd1, x,y);
            }else
            {
                addEdge(&sd2, x,y);
            }
            draw();
        break;
    }
}

void onMatchTestMouseEvent(int event, int x, int y, int, void *dt)
{
    MatchTest *mt = (MatchTest*) dt;
    mt->mouseEvent(event,x,y);
}
