#include "GTLoop.h"

#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include "GTLoopLoader.h"

GTLoop::GTLoop():
    roboMap("../../../../SonarGraphData/Maps/Yatcht_Special.ini"),
//   datasetPath("/media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/ARACATI_FULL/")
//    datasetPath("../../../../SonarGraphData/grayData/")
  datasetPath("/media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/grayShort2/")
{
    maxDistBetweenCenters = 15.f;
    maxDistBetweenPositions = 9.f;
}

GTLoop::~GTLoop()
{
}

void GTLoop::showGTLoop()
{
//    namedWindow("BlueImg",WINDOW_NORMAL);
//    namedWindow("PurpleImg",WINDOW_NORMAL);

//    for(unsigned i = 0 ; i < m_matchEdges.size() ; i++)
//    {
//        GTLoopEdge &edge = m_matchEdges[i];
//        GTLoopFrame &src = m_frames[edge.srcId],
//                    &dst = m_frames[edge.dstId];

//        Mat srcImg = imread(datasetPath + "/imgs/" + src.fileName,CV_LOAD_IMAGE_ANYDEPTH),
//            dstImg = imread(datasetPath + "/imgs/" + dst.fileName,CV_LOAD_IMAGE_ANYDEPTH);

//        srcImg.convertTo(srcImg,CV_8UC1,1.0,0.0);
//        dstImg.convertTo(dstImg,CV_8UC1,1.0,0.0);

//        imshow("BlueImg" , srcImg);
//        imshow("PurpleImg" , dstImg);

//        roboMap.clearSonarStates();
//        roboMap.newSonarState(SonarState(src.p,src.c, src.heading));
//        roboMap.newSonarState(SonarState(dst.p,dst.c,dst.heading));

//        roboMap.clearLabels();

//        char str[200];
//        sprintf(str,"Match Score: %.3f", edge.score);
//        roboMap.newLabel(RoboMapLabel(str,Point2f(1200,200),Scalar(0.f,140.f,255.f)));

//        roboMap.showScreen();

//        cout << "Score: " << edge.score << endl;
//        waitKey(1);
//    }
//    destroyWindow("BlueImg");
//    destroyWindow("PurpleImg");

}

bool GTLoop::loadGTLoopWithTimeStamp(const string &file)
{
    FILE *f = fopen(file.c_str(),"r");
    if(f == 0x0)
    {
        cout << "WFGTLoop:: GTLoop can't load GTLoop dataset: "
             << file
             << endl;
        return false;
    }

    char c=0;
    // Dicard first line (header line)
    while(c != '\n')
    {
        c = fgetc(f);
    }

    unsigned src=0u, dst=0u,maxId=0;
    float score;
    double timeStamp;

    //          img sourceID, Img destID, matchScore, timeStamp
    while(fscanf(f,"%u,%u,%g,%lg",
                  &src, &dst, &score,
                  &timeStamp) != -1)
    {
        if(maxId < src)
            maxId = src;
        if(maxId < dst)
            maxId = dst;

        m_matchEdges.push_back(GTLoopEdge(src,dst,score));
    }
    fclose(f);

    indexMatchs();

    for(unsigned i = 0; i < m_matchEdges.size(); i++)
    {
        const GTLoopEdge &ref = m_matchEdges[i];
        cout << ref.srcId << " - ("
             << ref.score << ") - "
             << ref.dstId << endl;
    }
}

bool GTLoop::loadGTLoop(const string &file)
{
    FILE *f = fopen(file.c_str(),"r");
    if(f == 0x0)
    {
        cout << "WFGTLoop:: GTLoop can't load GTLoop dataset: "
             << file
             << endl;
        return false;
    }

    char c=0;
    // Dicard first line (header line)
    while(c != '\n')
    {
        c = fgetc(f);
    }

    unsigned src=0u, dst=0u,maxId=0;
    float score;

    //          img sourceID, Img destID, matchScore, timeStamp
    while(fscanf(f,"%u,%u,%g",
                  &src, &dst, &score) != -1)
    {
        if(maxId < src)
            maxId = src;
        if(maxId < dst)
            maxId = dst;

        m_matchEdges.push_back(GTLoopEdge(src,dst,score));
    }
    fclose(f);

    indexMatchs();

//    for(unsigned i = 0; i < m_matchEdges.size(); i++)
//    {
//        const GTLoopEdge &ref = m_matchEdges[i];
//        cout << ref.srcId << " - ("
//             << ref.score << ") - "
//             << ref.dstId << endl;
//    }
}

void GTLoop::indexMatchs()
{
    m_match.resize(m_frames.size());

    for(unsigned i = 0; i < m_matchEdges.size(); i++)
    {
        m_match[m_matchEdges[i].srcId].push_back(&m_matchEdges[i]);
        m_match[m_matchEdges[i].dstId].push_back(&m_matchEdges[i]);
    }
}

bool GTLoop::createGTLoop()
{
    for(unsigned i = 0 ; i < m_frames.size() ; i++)
    {
        GTLoopFrame & fr = m_frames[i];

        for(unsigned j = i+1 ; j < m_frames.size() ; j++)
        {
            GTLoopFrame & nxtFr = m_frames[j];
//            roboMap.clearSonarStates();

            float centerDist = cv::norm(nxtFr.c-fr.c);
            if(centerDist <= maxDistBetweenCenters)
            {
                m_matchEdges.push_back(GTLoopEdge(i,j,1.f - centerDist/maxDistBetweenCenters));
//                roboMap.newSonarState(SonarState(fr.p,fr.c, fr.heading, Scalar(0.f,255.f,0.f)));
//                roboMap.newSonarState(SonarState(nxtFr.p,nxtFr.c,nxtFr.heading, Scalar(0.f,255.f,0.f)));
            }else
            {
//                roboMap.newSonarState(SonarState(fr.p,fr.c, fr.heading));
//                roboMap.newSonarState(SonarState(nxtFr.p,nxtFr.c,nxtFr.heading));
            }
//            roboMap.showScreen();
        }
    }
    indexMatchs();
}

bool GTLoop::createGTLoopPedroOtavio()
{
    for(unsigned i = 0 ; i < m_frames.size() ; i++)
    {
        GTLoopFrame & fr = m_frames[i];
        cout << "Computing match of frame " << i << "/" << (m_frames.size()-1) << endl;
        for(unsigned j = i+1 ; j < m_frames.size() ; j++)
        {
            GTLoopFrame & nxtFr = m_frames[j];

            #ifdef GTLoop_DEBUG
            roboMap.clear();
            #endif

//            Point2f pDiff( fr.p - nxtFr.p);
//            double angDIff = (fr.heading - nxtFr.heading);

//            double score = sonar_match_2(pDiff.x,pDiff.y, angDIff*M_PI/180.0, 30.0);

            double score = sonar_match_3(fr.p, fr.heading, nxtFr.p, nxtFr.heading);

            m_matchEdges.push_back(GTLoopEdge(i,j, score ));

            #ifdef GTLoop_DEBUG
            roboMap.newSonarState(SonarState(fr.p,fr.c, fr.heading, Scalar(0.f,255.f,0.f)));
            roboMap.newSonarState(SonarState(nxtFr.p,nxtFr.c,nxtFr.heading, Scalar(0.f,255.f,0.f)));

            char str[300];
            sprintf(str, "Score = %lf" , score);
            roboMap.newLabel(RoboMapLabel(str,Point2f(1200,200),Scalar(0.f,140.f,255.f)));

//            sprintf(str,"%g , %g , %g", fr.p.x, fr.p.y, fr.heading);
//            roboMap.newLabel(RoboMapLabel(str,Point2f(2200,2000),Scalar(0.f,140.f,255.f)));

//            sprintf(str,"%g , %g , %g", nxtFr.p.x, nxtFr.p.y, nxtFr.heading);
//            roboMap.newLabel(RoboMapLabel(str,Point2f(200,2000),Scalar(0.f,140.f,255.f)));

            roboMap.showScreen();
            #endif

        }
    }
    indexMatchs();
}

double GTLoop::sonar_match_2(double dx, double dy, double dphi, double raio)
{
    Polygon_2 p;
    p.push_back(PointCGAL(0,0));


    double step = PHI/POINTS_IN_CURVE;
    for (double i = 0; i <= POINTS_IN_CURVE; ++i)
    {
      double theta = (i*step);
      p.push_back(PointCGAL(raio*cos(theta),raio*sin(theta)));
    }

    Polygon_2 poli2;
    poli2.push_back (PointCGAL(dx, dy));

    for (double i = 0; i <= POINTS_IN_CURVE; ++i)
    {
      double theta = (i*step)+dphi;
      poli2.push_back(PointCGAL(raio*cos(theta)+dx,raio*sin(theta)+dy));
    }

    Pwh_list_2                  intR;
    Pwh_list_2::const_iterator  it;
    double match = 0.0;


    if(CGAL::do_intersect(p, poli2)){
      CGAL::intersection (p, poli2, std::back_inserter(intR));
      for (it = intR.begin(); it != intR.end(); ++it) {


        // std::cout << "--> ";
        Polygon_2 ob = (*it).outer_boundary();
        match += CGAL::to_double(ob.area()/ p.area());
      }
    }

    return match;
}

double GTLoop::sonar_match_3(Point2f &p1, double h1, Point2f &p2, double h2)
{
    Polygon_2 poly1, poly2;

    RoboMap::getSonarCGALPoly(p1,h1,poly1);
    RoboMap::getSonarCGALPoly(p2,h2,poly2);

    Pwh_list_2                  intR;
    Pwh_list_2::const_iterator  it;
    double match = 0.0;

    if(CGAL::do_intersect(poly1, poly2))
    {
        CGAL::intersection(poly1, poly2, std::back_inserter(intR));
        for (it = intR.begin(); it != intR.end(); ++it)
        {
            // std::cout << "--> ";
            Polygon_2 ob = (*it).outer_boundary();
            match += CGAL::to_double(ob.area()/ poly1.area());

            #ifdef GTLoop_DEBUG
            Point2f cvPoly[ob.size()];
            CGAL2CV(ob, cvPoly);

            roboMap.UTM2Img(cvPoly,ob.size());

            RoboPoly rPoly(cvPoly,ob.size(),Scalar(0.f,0.f,255.f));
            roboMap.newPoly(rPoly);
            #endif
        }
    }

    return match;
}

bool GTLoop::saveGTLoop(const string &fileName)
{
    cout << "Saving " << fileName << endl;
    FILE *f = fopen(fileName.c_str(),"w");
    if(f == 0x0)
    {
        cout << "WFGTLoop:: GTLoop can't load GTLoop dataset: "
             << fileName
             << endl;
        return false;
    }

    cout << m_matchEdges.size() << " matchings" << endl;

    for(unsigned i = 0 ; i < m_matchEdges.size() ; i ++)
    {
        GTLoopEdge & fr = m_matchEdges[i];
        fprintf(f,"%u,%u,%lf\n",
                fr.srcId, fr.dstId, fr.score);
    }

    fclose(f);
}

bool GTLoop::saveFrameInfo(const string &fileName)
{
    FILE *f = fopen(fileName.c_str(),"w");
    if(f == 0x0)
    {
        cout << "WFGTLoop::saveFrameInfo GTLoop can't save file "
             << fileName
             << endl;
        return false;
    }

    for(unsigned i = 0 ; i < m_frames.size() ; i ++)
    {
        GTLoopFrame & fr = m_frames[i];
        fprintf(f,"%u,%g,%g,%g,%g,%g\n",
                i,fr.p.x, fr.p.y, fr.c.x, fr.c.y,fr.heading);
    }

    fclose(f);
}

void GTLoop::generateHeadingDiffImage()
{
    unsigned nFr = m_frames.size();
    Mat headingDiffImage(nFr,nFr, CV_8UC1);

    for(unsigned uFr = 0; uFr < nFr ; uFr++)
    {
        for(unsigned vFr = 0; vFr < nFr; vFr++)
        {
            float h1 = m_frames[uFr].heading,
                  h2 = m_frames[vFr].heading,
                  diff;

            if(h2 > h1)
                diff = min(h2-h1, 360.f-h2+h1);
            else
                diff = min(h1-h2, 360.f-h1+h2);
            // Normalization between 0 and 255 (begger ang diff is 180 degree)
//            cout << diff << " | " << (diff/180.f)*255 << endl;
            headingDiffImage.at<uchar>(uFr,vFr) = (diff/180.f)*255;
        }
    }

    Mat colorImg;
    applyColorMap(headingDiffImage,colorImg,COLORMAP_JET);

    imwrite("gtHeadingDiffImage.png",headingDiffImage);
    imwrite("gtHeadingDiffImageColor.png",colorImg);

    namedWindow("GT Heading Diff",WINDOW_NORMAL);
    imshow("GT Heading Diff", colorImg);
    waitKey();
}

void GTLoop::start()
{
    cout << "Working with dataset path " << datasetPath << endl;
    GTLoopLoader csvLoader;

    csvLoader.loadPositionFromCSV( datasetPath + "tf_position.csv");
//    csvLoader.loadHeadingFromCSV ( datasetPath + "compass.csv" );
    csvLoader.loadHeadingFromCSV ( datasetPath + "compass_from_gps.csv" );
    csvLoader.loadFramesFromCSV  ( datasetPath + "imgs/sonar_xy_gray_info.csv");


    // Syc information and create GTLoop frames
    csvLoader.syncFramesData("sonar_xy_gray_",m_frames);

//    loadGTLoop("../../../../SonarGraphData/grayData/GTSimilarMatchs.csv");
    /// Compute loop closure ground truth
//    createGTLoop();
    createGTLoopPedroOtavio();

      generateHeadingDiffImage();

      saveGTLoop(datasetPath + "GTMatchs_novo.csv");
//    loadGTLoop(datasetPath + "GTMatchs.csv");

//    wt->saveEmptyFrames((datasetPath + "Frames.txt").c_str());
}

void CGAL2CV(Polygon_2 &cgalPoly, Point2f *cvPoly)
{
    for(unsigned i = 0 ; i < cgalPoly.size(); i++)
    {
        const PointCGAL &p = cgalPoly[i];
        cvPoly[i] = Point2f(CGAL::to_double(p.x()),CGAL::to_double(p.y()));
    }
}

void CV2CGAL(unsigned nPts, Point2f *cvPoly, Polygon_2 &cgalPoly)
{
    cgalPoly.clear();
    for(unsigned i = 0 ; i < nPts; i++)
    {
        const Point2f &p = cvPoly[i];
        cgalPoly.push_back(PointCGAL(p.x,p.y));
    }
}
