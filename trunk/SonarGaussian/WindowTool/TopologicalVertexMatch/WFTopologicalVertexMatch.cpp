#include "WFTopologicalVertexMatch.h"
#include "WindowTool/WindowTool.h"

#include "Drawing/Drawing.h"

FrameSD *WFTopologicalVertexMatch::frame(unsigned i)
{
    // Yes, we are breaking some SOLID principles of OO Programming
    // if you know a better way to solve this, please send me a e-mail
    // matheusbg8@gmail.com, thank you!
    return (FrameSD *) wt->frames[i];
}

int WFTopologicalVertexMatch::selectGaussianOnFrame(int x, int y, int frameID, float precision)
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

void WFTopologicalVertexMatch::removeGaussianSelections()
{
    leftSelecGaussian = rightSelecGaussian = -1;
}

void WFTopologicalVertexMatch::explore(unsigned edgeId)
{
    if(leftSelecGaussian >=0 && rightSelecGaussian>=0)
    {
        vector<MatchInfoWeighted> &edgeMatch = VertexMatchInfo.edgeMatchInfo;
        vector<GraphLink*> &leftEdges = sds[wt->currentLeftFrame]->graph[leftSelecGaussian],
                           &rightEdges = sds[wt->currentRightFrame]->graph[rightSelecGaussian];

        if(edgeId < edgeMatch.size() )
        {
            GraphLink *leftEdge = leftEdges[edgeMatch[edgeId].uID],
                      *rightEdge = rightEdges[edgeMatch[edgeId].vID];

            leftSelecGaussian = leftEdge->dest;
            rightSelecGaussian = rightEdge->dest;
        }
    }
}

void WFTopologicalVertexMatch::selectBestPairOfLeftGaussian()
{
    if(leftSelecGaussian >= 0)
    {
        unsigned leftId = wt->getLeftFrameId(),
                 rightId = wt->getRighFrameId();

        vector<Gaussian> &leftGaussians = sds[leftId]->gaussians,
                         &rightGaussians = sds[rightId]->gaussians;

        vector<vector<GraphLink*> > &leftGraph = sds[leftId]->graph,
                                    &rightGraph = sds[rightId]->graph;
        vector<MatchInfoWeighted> edgeMatchInfo;

        int bestMatchId=-1;
        unsigned nMatchs=0;
        float error, lowerError= 999999.9f;
        for(unsigned grightId=0; grightId < rightGraph.size(); grightId++)
        {
            error = sonar.computeVertexMatch(
                leftGaussians[leftSelecGaussian],
                rightGaussians[grightId],
                leftGraph[leftSelecGaussian],
                rightGraph[grightId],
                edgeMatchInfo);

            if(edgeMatchInfo.size() > nMatchs ||
               (edgeMatchInfo.size() == nMatchs && error < lowerError) )
            {
                nMatchs = edgeMatchInfo.size();
                bestMatchId = grightId;
                lowerError = error;
            }
        }
        if(bestMatchId >= 0)
        {
            rightSelecGaussian = bestMatchId;
        }
    }
}

WFTopologicalVertexMatch::WFTopologicalVertexMatch():
    leftSelecGaussian(-1),
    rightSelecGaussian(-1),
    showVertexMatch(true),
    sonar(sonarConfig),
    sonarConfig("../SonarGaussian/Configs.ini")
{
    sonar.setStoreImgs(false);

    if(showVertexMatch)
    {
        namedWindow("VertexMatch", 1);
    }
}

void WFTopologicalVertexMatch::start()
{
//        wt->currentLeftFrame = 129;
//        wt->currentRightFrame = 180;
}

Frame *WFTopologicalVertexMatch::newFrame(const string &fileName, unsigned frameNumber)
{
    // New empty Sonar Descriptor
    sds.push_back(0x0);
    return new FrameSD(fileName,frameNumber);
}

void WFTopologicalVertexMatch::keyPress(char c)
{
    switch(c)
    {
    case 'u':
        sonar.segmentationCalibUI(wt->selectedFrameImg());
    break;
    case '1':
        explore(0);
    break;
    case '2':
        explore(1);
    break;
    case '3':
        explore(2);
    break;
    case '4':
        explore(3);
    break;
    case 'b':
        selectBestPairOfLeftGaussian();
    break;
    }
}

void WFTopologicalVertexMatch::mouseEvent(int event, int x, int y)
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

            rightSelecGaussian = selectGaussianOnFrame(x,y,wt->currentRightFrame);

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
                 << sds[wt->currentRightFrame]->gaussians[rightSelecGaussian]
                 << endl;

        }
    }
}

void WFTopologicalVertexMatch::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    SonarDescritor *rightSD = sds[rightId],
                   *leftSD = sds[leftId];

    Drawing::drawDescriptorColor(rightImg,rightSD,Scalar(0,0,255),Scalar(0,255,0),false,false,false,true,true,false);
    Drawing::drawDescriptorColor(leftImg,leftSD,Scalar(0,0,255),Scalar(0,255,0),false,false,false,true,true,false);

//    for(unsigned i = 0; i < matchInfo.size(); i++)
//    {
//        unsigned next = (i+1)%matchInfo.size();
//        const Gaussian &lg = leftSD->gaussians[matchInfo[i].uID],
//                       &nlg = leftSD->gaussians[matchInfo[next].uID],
//                       &rg = rightSD->gaussians[matchInfo[i].vID],
//                       &nrg = rightSD->gaussians[matchInfo[next].vID];

//        line(leftImg,
//             Point2f(lg.x,lg.y),
//             Point2f(nlg.x, nlg.y),
//             Drawing::color[i%Drawing::nColor],4);

//        line(rightImg,
//             Point2f(rg.x,rg.y),
//             Point2f(nrg.x, nrg.y),
//             Drawing::color[i%Drawing::nColor],4);

//    }

//    if(matchInfo.size() > 0)
//    {
//        const Gaussian &leftRef = leftSD->gaussians[matchInfo[0].uID],
//                       &rightRef = rightSD->gaussians[matchInfo[0].vID];

//        for(unsigned i = 1; i < matchInfo.size(); i++)
//        {
//            const Gaussian &lg = leftSD->gaussians[matchInfo[i].uID],
//                           &rg = rightSD->gaussians[matchInfo[i].vID];

//            line(leftImg,
//                 Point2f(leftRef.x,leftRef.y),
//                 Point2f(lg.x, lg.y),
//                 Scalar(0.f,255.f,255.f),4);

//            line(rightImg,
//                 Point2f(rightRef.x,rightRef.y),
//                 Point2f(rg.x, rg.y),
//                 Scalar(0.f,255.f,255.f),4);

//        }
//    }

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
//        gm.computeEdgeMatch(sds[leftId]->graph[leftSelecGaussian],
//                            sds[rightId]->graph[rightSelecGaussian],
//                            matchs[0].edgeMatchInfo);
        VertexMatchInfo.uID = leftSelecGaussian;
        VertexMatchInfo.vID = rightSelecGaussian;
        VertexMatchInfo.sBestScore = 99999.9f;

        /// @todo - Maybe use a specific Vertex Match here
        VertexMatchInfo.edgeMatchInfo.clear();

        VertexMatchInfo.bestScore = sonar.computeVertexMatch(
            sds[leftId]->gaussians[leftSelecGaussian],
            sds[rightId]->gaussians[rightSelecGaussian],
            sds[leftId]->graph[leftSelecGaussian],
            sds[rightId]->graph[rightSelecGaussian],
            VertexMatchInfo.edgeMatchInfo);

        vector<MatchInfoWeighted> &edgeMatchs = VertexMatchInfo.edgeMatchInfo;
        cout << "Edges match found:" << endl;
        for(unsigned i = 0; i < edgeMatchs.size() ; i++)
        {
            cout << edgeMatchs[i].uID << " -> " << edgeMatchs[i].vID<< endl;
        }

        Mat imgVertexMatch;

        Drawing::drawVertexMatch(imgVertexMatch,Size2i(1000,600),
                                 *sds[leftId],*sds[rightId],
                                 leftSelecGaussian,rightSelecGaussian,
                                 VertexMatchInfo.edgeMatchInfo,
                                 false,true,
                                 Scalar(0,0,255),Scalar(0,255,0),2);

        imshow("VertexMatch",imgVertexMatch);

//        vector<MatchInfoExtended> graphMatch(1,initialVertexMatch);

//        gm.findGraphMatchWithInitialGuess(sds[leftId]->graph[leftSelecGaussian],
//                                          sds[rightId]->graph[rightSelecGaussian],
//                                          initialVertexMatch,graphMatch);


        Drawing::drawGraphVertexMatchs(leftImg,rightImg,
                                    *sds[leftId],*sds[rightId],
                                    VertexMatchInfo,4);

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

//        vector<MatchInfoWeighted> edgeMatchs;

////        gm.computeEdgeMatch(sds[leftId]->graph[leftSelecGaussian],
////                            sds[rightId]->graph[rightSelecGaussian],
////                            edgeMatchs);

//        gm.computeEdgeMatchAxe(sds[leftId]->graph[leftSelecGaussian],
//                               sds[rightId]->graph[rightSelecGaussian],
//                               edgeMatchs);

    }

}

void WFTopologicalVertexMatch::renderFrameTogether(Mat &screen, const Scalar_<float> &el, unsigned leftFrameId, const Scalar_<float> &er, unsigned rightFrameId)
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

    Drawing::drawMatchings(screen,el,er,
                           sds[leftFrameId]->gaussians,
                           sds[rightFrameId]->gaussians,
                           matchInfo,Scalar(255.f,255.f,0));

}

void WFTopologicalVertexMatch::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
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

    matchInfo.clear();
    sonar.computeMatchs(sds[leftId], sds[rightId], matchInfo);

    cout << "Processed frames rightId "
         << rightId
         << " and leftId "
         << leftId
         << endl;
}
