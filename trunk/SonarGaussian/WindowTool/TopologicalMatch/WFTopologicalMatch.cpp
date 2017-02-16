#include "WFTopologicalMatch.h"
#include "WindowTool/WindowTool.h"

#include "Drawing/Drawing.h"

FrameSD *WFTopologicalMatch::frame(unsigned i)
{
    // Yes, we are breaking some SOLID principles of OO Programming
    // if you know a better way to solve this, please send me a e-mail
    // matheusbg8@gmail.com, thank you!
    return (FrameSD *) wt->frames[i];
}

WFTopologicalMatch::WFTopologicalMatch():
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
    switch(c)
    {
    case 'u':
        sonar.segmentationCalibUI(wt->selectedFrameImg());
    break;
    case '1':
    break;
    case '2':
    break;
    case '3':
    break;
    case '4':
    break;
    case 'b':
    break;
    }
}

void WFTopologicalMatch::mouseEvent(int event, int x, int y)
{

}

void WFTopologicalMatch::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    SonarDescritor *rightSD = sds[rightId],
                   *leftSD = sds[leftId];

    Drawing::drawDescriptorColor(rightImg,rightSD,Scalar(0,0,255),Scalar(0,255,0),false,false,false,true,true,false);
    Drawing::drawDescriptorColor(leftImg,leftSD,Scalar(0,0,255),Scalar(0,255,0),false,false,false,true,true,false);
}

void WFTopologicalMatch::renderFrameTogether(Mat &screen,
                                             const Scalar_<float> &el, unsigned leftFrameId,
                                             const Scalar_<float> &er, unsigned rightFrameId)
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

void WFTopologicalMatch::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{

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
