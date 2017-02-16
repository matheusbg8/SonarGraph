#include "WFTopologicalMatch.h"
#include "WindowTool.h"

#include "Drawing/Drawing.h"

FrameSD *WFTopologicalMatch::frame(unsigned i)
{
    // Yes, we are breaking some SOLID principles of OO Programming
    // if you know a better way to solve this, please send me a e-mail
    // matheusbg8@gmail.com, thank you!
    return (FrameSD *) wt->frames[i];
}

int WFTopologicalMatch::selectGaussianOnFrame(int x, int y, int frameID, float precision)
{
    int gaussianID = -1;
    float minDist=FLT_MAX;
    vector<Gaussian> &gaussians = sds[frameID]->gaussians;

    for(unsigned g = 0 ; g < gaussians.size(); g++)
    {
        float dx = gaussians[g].x - x,
              dy = gaussians[g].y - y,
              dist = sqrt(dx*dx + dy*dy);

        if(minDist > dist )
        {
            gaussianID = g;
            minDist = dist;
        }
    }

    if(minDist > precision)
    {
        gaussianID = -1;
    }

    return gaussianID;
}

void WFTopologicalMatch::removeGaussianSelections()
{
    leftSelecGaussian = rightSelecGaussian = -1;
}

WFTopologicalMatch::WFTopologicalMatch():
    leftSelecGaussian(-1),
    rightSelecGaussian(-1),
    showVertexMatch(true),
    sonarConfig("../SonarGaussian_02_2016_r114/MachadosConfig"),
    sonar(sonarConfig)
{
    sonar.setStoreImgs(false);

    if(showVertexMatch)
    {
        namedWindow("VertexMatch", 1);
    }
}

void WFTopologicalMatch::start()
{
//    wt->currentLeftFrame = 129;
//    wt->currentRighFrame = 130;
}

Frame *WFTopologicalMatch::newFrame(const string &fileName, unsigned frameNumber)
{
    // New empty Sonar Descriptor
    sds.push_back(0x0);
    return new FrameSD(fileName,frameNumber);
}

void WFTopologicalMatch::keyPress(char c)
{
}

void WFTopologicalMatch::mouseEvent(int event, int x, int y)
{
    if(event == CV_EVENT_LBUTTONUP)
    {
        if(x < wt->windowSize.width*0.5f) // Left Frame
        {
            x = (x-wt->el.val[2])/wt->el.val[0];
            y = (y-wt->el.val[3])/wt->el.val[1];

            leftSelecGaussian = selectGaussianOnFrame(x,y,wt->currentLeftFrame);

            cout << "Left transform:"
                 << " dx " << wt->el.val[2]
                 << " dy " << wt->el.val[3]
                 << " ex " << wt->el.val[0]
                 << " ey " << wt->el.val[1]
                 << " mx " << x
                 << " my " << y
                 << " left gaussian " << leftSelecGaussian
                 << endl;

            cout << "selected gaussian"
                 << sds[wt->currentLeftFrame]->gaussians[leftSelecGaussian]
                 << endl;

        }else // Right Frame
        {
            x = (x-wt->er.val[2])/wt->er.val[0];
            y = (y-wt->er.val[3])/wt->er.val[1];

            rightSelecGaussian = selectGaussianOnFrame(x,y,wt->currentRighFrame);

            cout << "Right transform:"
                 << " dx " << wt->er.val[2]
                 << " dy " << wt->er.val[3]
                 << " ex " << wt->er.val[0]
                 << " ey " << wt->er.val[1]
                 << " mx " << x
                 << " my " << y
                 << " right gaussian " << rightSelecGaussian
                 << endl;

            cout << "selected gaussian"
                 << sds[wt->currentRighFrame]->gaussians[rightSelecGaussian]
                 << endl;

        }
    }
}

void WFTopologicalMatch::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    SonarDescritor *rightSD = sds[rightId],
                   *leftSD = sds[leftId];

    char tmpTxt[200];
    sprintf(tmpTxt, "frame_%0.3d.png",leftId);

//    leftImg = imread(tmpTxt);

    Drawing::drawDescriptorColor(rightImg,rightSD,Scalar(0,0,255),Scalar(0,255,0),false,false,false,true,true,false);
    Drawing::drawDescriptorColor(leftImg,leftSD,Scalar(0,255,0),Scalar(0,255,0),false,false,false,true,false,false);

    sprintf(tmpTxt, "frame_%0.3d_TR.png",leftId);

    imwrite(tmpTxt,leftImg);

    if(leftSelecGaussian >= 0)
    {
        vector<Gaussian> &leftGaussians = sds[leftId]->gaussians;
        Drawing::drawGaussian(leftImg,leftGaussians[leftSelecGaussian],
                              Scalar(0,255,0),Scalar(0,0,255),leftSelecGaussian);

//        Drawing::drawVertex(leftImg,sds[leftId],leftSelecGaussian,
//                            Scalar(0,255,0),Scalar(0,0,255));
    }

    if(rightSelecGaussian >= 0)
    {

        vector<Gaussian> &rightGaussians = sds[rightId]->gaussians;
        Drawing::drawGaussian(rightImg,rightGaussians[rightSelecGaussian],
                              Scalar(0,255,0),Scalar(0,0,255),rightSelecGaussian);

//        Drawing::drawVertex(rightImg,sds[rightId],rightSelecGaussian,
//                            Scalar(0,255,0),Scalar(0,0,255));

    }

    if(showVertexMatch && leftSelecGaussian >=0 && rightSelecGaussian>=0)
    {
        MatchInfoExtended initialVertexMatch;

//        gm.computeEdgeMatch(sds[leftId]->graph[leftSelecGaussian],
//                            sds[rightId]->graph[rightSelecGaussian],
//                            matchs[0].edgeMatchInfo);

        initialVertexMatch.uID = leftSelecGaussian;
        initialVertexMatch.vID = rightSelecGaussian;
        initialVertexMatch.sBestScore = 99999.9f;
        initialVertexMatch.bestScore =
             gm.computeEdgeMatchAxe(sds[leftId]->graph[leftSelecGaussian],
                               sds[rightId]->graph[rightSelecGaussian],
                               initialVertexMatch.edgeMatchInfo);

//        cout << "Edges match found:" << endl;
//        for(unsigned i = 0; i < edgeMatchs.size() ; i++)
//        {
//            cout << edgeMatchs[i].first.first << " -> " << edgeMatchs[i].first.second << endl;
//        }

        Mat imgVertexMatch;

        Drawing::drawVertexMatch(imgVertexMatch,Size2i(1000,600),
                                 *sds[leftId],*sds[rightId],
                                 leftSelecGaussian,rightSelecGaussian,
                                 initialVertexMatch.edgeMatchInfo,
                                 false,true,
                                 Scalar(0,0,255),Scalar(0,255,0),2);

        imshow("VertexMatch",imgVertexMatch);

        vector<MatchInfoExtended> graphMatch(1,initialVertexMatch);

//        gm.findGraphMatchWithInitialGuess(sds[leftId]->graph[leftSelecGaussian],
//                                          sds[rightId]->graph[rightSelecGaussian],
//                                          initialVertexMatch,graphMatch);


        Drawing::drawVertecesMatchs(leftImg,rightImg,
                                    *sds[leftId],*sds[rightId],
                                    graphMatch,2);

//        Mat imgVertexMatch(800,1000,CV_8UC3,Scalar_<uchar>(0,0,0));

//        Drawing::drawVertex(imgVertexMatch,sds[leftId],leftSelecGaussian,
//                            Rect(0,0,500,800),
//                            Scalar(0,0,255),Scalar(0,255,0),true);

//        Drawing::drawVertex(imgVertexMatch,sds[rightId],rightSelecGaussian,
//                            Rect(500,0,500,800),
//                            Scalar(0,0,255),Scalar(0,255,0),true);


    }else // Try to do automaticaly match
    {
//        vector<vector<GraphLink*> > &gl = sds[leftId]->graph,
//                                    &gr = sds[rightId]->graph;

//        int rightBestVertex=-1;
//        unsigned rightHighVertexDegree=0u;

//        for(unsigned i = 0; i < gr.size();i++)
//        {
//            if(rightHighVertexDegree < gr[i].size())
//            {
//                rightHighVertexDegree = gr[i].size();
//                rightBestVertex = i;
//            }
//        }

//        vector<PUUPFF> edgeMatchs;

////        gm.computeEdgeMatch(sds[leftId]->graph[leftSelecGaussian],
////                            sds[rightId]->graph[rightSelecGaussian],
////                            edgeMatchs);

//        gm.computeEdgeMatchAxe(sds[leftId]->graph[leftSelecGaussian],
//                               sds[rightId]->graph[rightSelecGaussian],
//                               edgeMatchs);

    }
    cout << "Fim render process!!" << endl;

}

void WFTopologicalMatch::renderFrameTogether(Mat &screen, const Scalar_<float> &el, const Scalar_<float> &er)
{
    // Draw texts
    char tempStr[200];
    sprintf(tempStr,
            "Left Gaussian %d",
            leftSelecGaussian);
    putText(screen,tempStr,
           Point2f(30.f, wt->windowSize.height-10),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);

    sprintf(tempStr,
            "Right Gaussian %d",
            rightSelecGaussian);
    putText(screen,tempStr,
           Point2f(wt->windowSize.width*0.5f + 30.f, wt->windowSize.height-10),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);
}

void WFTopologicalMatch::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    switch(wt->selecFrame)
    {
        case -1: removeGaussianSelections(); break;
        case 0: leftSelecGaussian = -1; break;
        case 1: rightSelecGaussian = -1; break;
    }

    if(sds[leftId] == 0x0)
    {
        sds[leftId] = sonar.newImageDirect(leftImg);
    }

    if(sds[rightId] == 0x0)
    {
        sds[rightId] = sonar.newImageDirect(rightImg);
    }

    cout << "Processed frames rightId "
         << rightId
         << " and leftId "
         << leftId
         << endl;

}
