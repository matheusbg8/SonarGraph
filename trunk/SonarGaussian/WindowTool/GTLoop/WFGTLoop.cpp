#include "WFGTLoop.h"
#include <iostream>
using namespace std;
#include <opencv2/contrib/contrib.hpp>

#include "WindowTool/WindowTool.h"
#include "RoboMap/MapPloter.h"

WFGTLoop::WFGTLoop():
    roboMap("../../../../SonarGraphData/Maps/Yatcht_Special.ini"),
//   datasetPath("/media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/ARACATI_FULL/")
//    datasetPath("../../../../SonarGraphData/grayData/")
  datasetPath("media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/grayData2/")
{
    firstPause = true;

    maxDistBetweenCenters = 15.f;
    maxDistBetweenPositions = 9.f;
}

WFGTLoop::~WFGTLoop()
{
}

void WFGTLoop::showGTLoop()
{
    namedWindow("BlueImg",WINDOW_NORMAL);
    namedWindow("PurpleImg",WINDOW_NORMAL);

    for(unsigned i = 0 ; i < m_matchEdges.size() ; i++)
    {
        GTLoopEdge &edge = m_matchEdges[i];
        GTLoopFrame &src = m_frames[edge.srcId],
                    &dst = m_frames[edge.dstId];

        Mat srcImg = imread(datasetPath + "/imgs/" + src.fileName,CV_LOAD_IMAGE_ANYDEPTH),
            dstImg = imread(datasetPath + "/imgs/" + dst.fileName,CV_LOAD_IMAGE_ANYDEPTH);

        srcImg.convertTo(srcImg,CV_8UC1,1.0,0.0);
        dstImg.convertTo(dstImg,CV_8UC1,1.0,0.0);

        imshow("BlueImg" , srcImg);
        imshow("PurpleImg" , dstImg);

        roboMap.clearSonarStates();
        roboMap.newSonarState(SonarState(src.p,src.c, src.heading));
        roboMap.newSonarState(SonarState(dst.p,dst.c,dst.heading));

        roboMap.clearLabels();

        char str[200];
        sprintf(str,"Match Score: %.3f", edge.score);
        roboMap.newLabel(RoboMapLabel(str,Point2f(1200,200),Scalar(0.f,140.f,255.f)));

        roboMap.showScreen();

        cout << "Score: " << edge.score << endl;
        if(firstPause)
        {
            waitKey();
            firstPause = false;
        }
        else
            waitKey(1);
    }
    destroyWindow("BlueImg");
    destroyWindow("PurpleImg");

}

bool WFGTLoop::loadGTLoopWithTimeStamp(const string &file)
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

bool WFGTLoop::loadGTLoop(const string &file)
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

void WFGTLoop::indexMatchs()
{
    m_match.resize(m_frames.size());

    for(unsigned i = 0; i < m_matchEdges.size(); i++)
    {
        m_match[m_matchEdges[i].srcId].push_back(&m_matchEdges[i]);
        m_match[m_matchEdges[i].dstId].push_back(&m_matchEdges[i]);
    }
}

bool WFGTLoop::createGTLoop()
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

bool WFGTLoop::saveGTLoop(const string &fileName)
{
    FILE *f = fopen(fileName.c_str(),"w");
    if(f == 0x0)
    {
        cout << "WFGTLoop:: GTLoop can't load GTLoop dataset: "
             << fileName
             << endl;
        return false;
    }

    for(unsigned i = 0 ; i < m_matchEdges.size() ; i ++)
    {
        GTLoopEdge & fr = m_matchEdges[i];
        fprintf(f,"%u,%u,%g\n",
                fr.srcId, fr.dstId, fr.score);
    }

    fclose(f);
}

bool WFGTLoop::saveFrameInfo(const string &fileName)
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

void WFGTLoop::loadWindowTool(const string& imgPrefix)
{
    wt->clearFrames();

    vector<Frame*> &wtFr = wt->frames;
    wtFr.resize(m_frames.size());

    char str[500];
    for(int i = 0 ; i < m_frames.size(); i++)
    {
        sprintf(str,"%s/imgs/%s%05d.png",
                datasetPath.c_str(),
                imgPrefix.c_str(),
                i);
        wtFr[i] = new FrameGT(str,i);
    }

    wt->currentLeftFrame = 0;
    wt->currentRightFrame = 1;

    wt->loadImgMaskAndIsonificationPatern();

}

void WFGTLoop::plotFullMapPath()
{
    // Plot path
    MapPloter mp;
    mp.loadMap("../../../../SonarGraphData/Maps/Yatcht_Special.ini");
    float pathLenght=0;
    vector<CSVTFPosition> &pts = csvData.csvPosition;
    for(unsigned i = 1 ; i < pts.size();i++)
    {
        float dx = pts[i].x - pts[i-1].x,
              dy = pts[i].y - pts[i-1].y;

        pathLenght+= sqrt(dx*dx+dy*dy);

        mp.drawLine(Point2f(pts[i-1].x, pts[i-1].y),
                    Point2f(pts[i].x,pts[i].y),
                    pathLenght / csvData.totalPathLenght);
    }

    mp.showImg();
    mp.saveImg("MapPloter.png");
    waitKey();
}

void WFGTLoop::plotRestrictedMapPath(unsigned minNumFeaturesThreshold)
{
    // Plot path
    MapPloter mp;
    Mat pathBar,
        binPathBar,
        filteredPathBar,
        featuresBar,
        filteredFeaturesBar;

    mp.loadMap("../../../../SonarGraphData/Maps/Yatcht_Special.ini");
    float pathLenght=0;

    vector<CSVFrameDescriptionInfo> &csvFrameDescription = csvData.csvFrameDescription;
    // Important! csvFrameDescription and m_frames should be same size!

    pathBar = Mat(1,m_frames.size(),CV_8UC1);
    binPathBar = Mat(1,m_frames.size(),CV_8UC1);
    featuresBar = Mat(1,m_frames.size(),CV_8UC1);

    unsigned maxNumFeatures = 0, minNumFeatures = 99999,
             filteredFrameCount=0;

    // Compute min/max num features
    for(unsigned i = 0; i < csvFrameDescription.size();i++)
    {
        unsigned currentNumFeatures = csvFrameDescription[i].vertexCount;
        if(currentNumFeatures > maxNumFeatures) maxNumFeatures = currentNumFeatures;
        if(currentNumFeatures < minNumFeatures) minNumFeatures = currentNumFeatures;
        if(currentNumFeatures >= minNumFeaturesThreshold) filteredFrameCount++;
    }

    unsigned filteredFrameId=0;
    filteredPathBar = Mat(1,filteredFrameCount,CV_8UC1);
    filteredFeaturesBar = Mat(1,filteredFrameCount,CV_8UC1);

    // Plot map lines
    for(unsigned i = 0; i < csvFrameDescription.size();i++)
    {
        unsigned currentNumFeatures = csvFrameDescription[i].vertexCount;
        Point2f p, // Current frame position
                np;// Next frame position

        if(i == 0)
        {
            p = np = m_frames[i].p;
        }else
        {
            p = m_frames[i-1].p,
            np = m_frames[i].p;
        }

        float dx = np.x - p.x,
              dy = np.y - p.y;

        pathLenght+= sqrt(dx*dx+dy*dy);
        float normalizedDistance = pathLenght / csvData.totalPathLenght,
              normalizedNumFeatures = ((float)currentNumFeatures-minNumFeatures) /
                                      ((float)maxNumFeatures - minNumFeatures );

        if(currentNumFeatures >= minNumFeaturesThreshold)
        {
            mp.drawLine(Point2f(p.x, p.y),
                        Point2f(np.x,np.y),
                        normalizedDistance);

            binPathBar.at<uchar>(0,i) = 255;
            filteredPathBar.at<uchar>(0,filteredFrameId) = round(normalizedDistance*100.f)*2.55f;
            filteredFeaturesBar.at<uchar>(0,filteredFrameId) = round(normalizedNumFeatures*100.f)*2.55f;
            filteredFrameId++;
        }else
        {
            mp.drawLine(Point2f(p.x, p.y),
                        Point2f(np.x,np.y),
                        Scalar(255,0,255));
            binPathBar.at<uchar>(0,i) = 0;
        }
        pathBar.at<uchar>(0,i) = round(normalizedDistance*100.f)*2.55f;
        featuresBar.at<uchar>(0,i) = round(normalizedNumFeatures*100.f)*2.55f;
    }

    mp.showImg();
    mp.saveImg("RestrictMapPloter.png");

    applyColorMap(pathBar, pathBar, COLORMAP_JET);
    applyColorMap(binPathBar, binPathBar, COLORMAP_JET);
    applyColorMap(filteredPathBar, filteredPathBar, COLORMAP_JET);
    applyColorMap(featuresBar, featuresBar, COLORMAP_JET);
    applyColorMap(filteredFeaturesBar, filteredFeaturesBar, COLORMAP_JET);

    resize(pathBar,pathBar,Size(),1,10,INTER_NEAREST);
    resize(binPathBar,binPathBar,Size(),1,10,INTER_NEAREST);
    resize(filteredPathBar,filteredPathBar,Size(),1,10,INTER_NEAREST);
    resize(featuresBar,featuresBar,Size(),1,10,INTER_NEAREST);
    resize(filteredFeaturesBar,filteredFeaturesBar,Size(),1,10,INTER_NEAREST);

    imwrite("pathBar.png",pathBar);
    imwrite("binPathBar.png",binPathBar);
    imwrite("filteredPathBar.png",filteredPathBar);
    imwrite("featuresBar.png",featuresBar);
    imwrite("filteredFeaturesBar.png",filteredFeaturesBar);

    imshow("pathBar" , pathBar);
    imshow("binPathBar" , binPathBar);
    imshow("filteredPathBar" , filteredPathBar);
    imshow("featuresBar" , featuresBar);
    imshow("filteredFeaturesBar" , filteredFeaturesBar);

//    pathBar,
//    binPathBar,
//    filteredPathBar,
//    featuresBar,
//    filteredFeaturesBar;
    waitKey();
}

void WFGTLoop::start()
{
    cout << "Working with dataset path " << datasetPath << endl;

    csvData.loadPositionFromCSV( datasetPath + "tf_position.csv");
    csvData.loadHeadingFromCSV ( datasetPath + "compass_from_gps.csv" );
//    csvLoader.loadHeadingFromCSV ( datasetPath + "compass.csv" );
    csvData.loadFramesFromCSV  ( datasetPath + "imgs/sonar_tr_gray_info.csv");

    // Syc information and create GTLoop frames
    csvData.syncFramesData("sonar_xy_gray_",m_frames);


//    loadGTLoop("../../../../SonarGraphData/grayData/GTSimilarMatchs.csv");
    loadGTLoop("media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/grayData2/GTMatchs_compass.csv");
    /// Compute loop closure ground truth
//    createGTLoop();

//    saveGTLoop(datasetPath + "GTMatchs.csv");
//    loadGTLoop(datasetPath + "GTMatchs.csv");

    loadWindowTool("sonar_xy_gray_");

//    wt->saveEmptyFrames((datasetPath + "Frames.txt").c_str());
    //    showGTLoop();

    // For restrict map ploting
    csvData.loadFrameDescriptionFromCSV(datasetPath + "Results/ResultFramesInformations.csv");
    plotFullMapPath();
    plotRestrictedMapPath(15);
}

void WFGTLoop::selectedFrame(int frameId)
{

}

void WFGTLoop::keyPress(char c)
{
    cout << "Key pressed (" << c << ")\n";
}

void WFGTLoop::mouseEvent(int event, int x, int y)
{

}

void WFGTLoop::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
}

void WFGTLoop::renderFrameTogether(Mat &screen, const Scalar_<float> &el, unsigned leftFrameId, const Scalar_<float> &er, unsigned rightFrameId)
{

}

void WFGTLoop::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    // Update robot state in roboMap
    GTLoopFrame &leftFr = m_frames[leftId],
                &rightFr = m_frames[rightId];

    roboMap.clearSonarStates();
    roboMap.newSonarState(SonarState(leftFr.p,leftFr.c, leftFr.heading));
    roboMap.newSonarState(SonarState(rightFr.p,rightFr.c,rightFr.heading));

    roboMap.showScreen();
}
