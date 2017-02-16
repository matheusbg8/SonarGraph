#include "WindowTool.h"
#include "Drawing/Drawing.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

WindowTool::WindowTool(string windowName, WindowFeature *feature):
    windowName(windowName),
    windowFetaure(feature),
    threshold1(171),
    threshold2(215),
    windowControlName("Control"),
//    windowSize(1850,910),
    windowSize(1300,600),
//    windowSize(1430*2,781),
    currentLeftFrame(0),
    currentRighFrame(1),
    selecFrame(-1),
    useColors(12),// 12 = GrayScale
    intensity(0),
    maxKindOfPixelAnalised(4000), // Used for K-Means auto threshold
    showIndividualFrameLeft(false),
    showIndividualFrameRight(false)
{
    // Register me on feature for be directional communications
    feature->wt = this;
}

WindowTool::~WindowTool()
{
    cvDestroyAllWindows();
}

/**
 * @brief This method return the color mapping name are using
 * to show acoustic image on screen.
 * @param str - The color mapping name are been used.
 */
void WindowTool::colorName(char *str)
{
    switch(useColors)
    {
        case COLORMAP_AUTUMN:
            sprintf(str,"Color Map Autum");
        break;
        case COLORMAP_BONE:
            sprintf(str,"Color Map Bone");
        break;
        case COLORMAP_JET:
            sprintf(str,"Color Map Jet");
        break;
        case COLORMAP_WINTER:
            sprintf(str,"Color Map Winter");
        break;
        case COLORMAP_RAINBOW:
            sprintf(str,"Color Map Rainbow");
        break;
        case COLORMAP_OCEAN:
            sprintf(str,"Color Map Ocean");
        break;
        case COLORMAP_SUMMER:
            sprintf(str,"Color Map Summer");
        break;
        case COLORMAP_SPRING:
            sprintf(str,"Color Map Spring");
        break;
        case COLORMAP_COOL:
            sprintf(str,"Color Map Cool");
        break;
        case COLORMAP_HSV:
            sprintf(str,"Color Map HSV");
        break;
        case COLORMAP_PINK:
            sprintf(str,"Color Map Pink");
        break;
        case COLORMAP_HOT:
            sprintf(str,"Color Map Hoy");
        break;
        case 12:
            sprintf(str,"Gray Scale");
        break;
    }
}


/**
 * @brief Load acoustic imagens
 *
 * @param fileName - File with a list of image names
 * @return bool - Return true if frames was load sucessful.
 */
bool WindowTool::loadEmptyFrames(const char *fileName)
{
    FILE *f_img_names = fopen(fileName,"r");
    char img_file_name[300];

    if(f_img_names == 0x0)
    {
        cout << "Image list file " << fileName << " not found!!" << endl;
        return false;
    }

    unsigned frame=0;
    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        frames.push_back(windowFetaure->newFrame(string(img_file_name),frame++));
    }
    fclose(f_img_names);

    isonificationPattern = imread("IsonificationPattern.png",CV_LOAD_IMAGE_ANYDEPTH);
    grayMask = imread("Mask.png",CV_LOAD_IMAGE_ANYDEPTH);

    if(isonificationPattern.rows == 0 || grayMask.rows == 0)
    {
        // Creating isonification pattern and mask
        createIsonificationPatternAndMask(240);

        imwrite("IsonificationPattern.png", isonificationPattern);
        imwrite("Mask.png", grayMask);
    }

    cvtColor(grayMask,bgrMask,CV_GRAY2BGR);

    return true;
}

void WindowTool::createIsonificationPatternAndMask(unsigned threadPixel)
{
    cout << "Computing Mean and Isonification Pattern..." << endl;

    Mat imgSum,validPixelCount;

    for(unsigned i = 0 ; i < frames.size() ; i++)
    {
        cout << "Looking for image (" << i << "): " << frames[i]->fileName <<endl;
        Mat aImg = imread(string(frames[i]->fileName),CV_LOAD_IMAGE_ANYDEPTH);

        if(imgSum.rows == 0)
        {
            imgSum = Mat(aImg.rows, aImg.cols, CV_32S, Scalar(0));
            validPixelCount = Mat(aImg.rows, aImg.cols, CV_32S, Scalar(0));
        }

        for(unsigned i = 0 ; i < aImg.rows; i++)
        {
            for(unsigned j = 0 ; j < aImg.cols; j++)
            {
                if(!segmentation.imgMask.at<uchar>(i,j)) continue;
                if(aImg.at<ushort>(i,j) < threadPixel)
                {
                    imgSum.at<int>(i,j) += aImg.at<ushort>(i,j);
                    validPixelCount.at<int>(i,j) ++;
                }
            }
        }
    }
    //  openCV Method
    //    normalize(mMeam,mMeam,0,255,NORM_MINMAX);


    // Our method:
    Mat imgMean(imgSum.rows, imgSum.cols, CV_32S, Scalar(0));
    grayMask = Mat(imgSum.rows, imgSum.cols,CV_8U, Scalar(0));

    for(unsigned i = 0 ; i < imgSum.rows; i++)
    {
        for(unsigned j = 0 ; j < imgSum.cols; j++)
        {
            if(validPixelCount.at<int>(i,j) > 0)
            {
                imgMean.at<int>(i,j) = imgSum.at<int>(i,j) / validPixelCount.at<int>(i,j);
                grayMask.at<uchar>(i,j) = 255;
            }
            else
            {
                imgMean.at<int>(i,j) = imgSum.at<int>(i,j);
            }
        }
    }

    imgMean.convertTo(isonificationPattern,CV_8UC1);

    imshow("Computed Isonification Pattern", isonificationPattern);
    imshow("Computed Mask", grayMask);
}

/**
 * @brief This is main Loop of window.
 *
 * Do not call this method directly, to start
 * this window tool call method start()
 *
 */
void WindowTool::mainLoop()
{
    char c=0;

    loadCurrentFrame();

    while(c != 27) // KEY_ESC
    {
        renderImgsOnResult();
        c = waitKey();


        switch(c) // Preview frame
        {
        case ',':
            loadPreviewFrame();
        break;
        case '.': // Next Frame
        case  -1:
            loadNextFrame();
        break;
        case 'c': // Change view color
            useColors = (useColors+1)%13;
        break;
        case ' ': // Remove Frame Selection
            removeSelection();
        break;
        }

        windowFetaure->keyPress(c);
    }
}


/**
 * @brief Start this windows tool.
 *
 */
void WindowTool::start()
{
    currentLeftFrame = 0;
    currentRighFrame = 1;

    windowFetaure->start();

    namedWindow(windowName, 1);
    setMouseCallback(windowName, on_WT_mouse, this);

    mainLoop();
}

void WindowTool::renderImgsOnResult()
{
    if(result.cols != windowSize.width || result.rows != windowSize.height)
    {
        result = Mat(windowSize.height, windowSize.width,CV_8UC3);
    }

    Mat cLeftImg,
        cRightImg;

    leftImg.convertTo(cLeftImg,CV_8UC3,1,-intensity);
    righImg.convertTo(cRightImg,CV_8UC3,1,-intensity);

    if(useColors != 12)
    {
        applyColorMap(cLeftImg,cLeftImg,useColors);
        applyColorMap(cRightImg,cRightImg,useColors);
    }else
    {
        cvtColor(cLeftImg,cLeftImg,CV_GRAY2BGR);
        cvtColor(cRightImg,cRightImg,CV_GRAY2BGR);
    }

    // Apply mask
    bitwise_and(cLeftImg,bgrMask,cLeftImg);
    bitwise_and(cRightImg,bgrMask,cRightImg);

    // Process color image
    windowFetaure->renderProcess(cLeftImg,currentLeftFrame,
                                 cRightImg,currentRighFrame);

    // Blender images
    Drawing::drawImgsTogether(windowSize,cLeftImg,cRightImg,
                              result,&el,&er);

    // Process final screen
    windowFetaure->renderFrameTogether(result,el,er);

    // Draw texts

    // Draw texts
    char tempStr[200];
    sprintf(tempStr,
            "Frame %d-%d , FS %d",
            currentLeftFrame,currentRighFrame,selecFrame);
    putText(result,tempStr,
           Point2f(20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);

    colorName(tempStr);
    putText(result,tempStr,
           Point2f(result.cols/2.f-100.f,20.f),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);

    // Show result
    imshow(windowName,result);
}

bool WindowTool::loadCurrentFrame()
{
    leftImg = imread(string(frames[currentLeftFrame]->fileName),CV_LOAD_IMAGE_ANYDEPTH);
    righImg = imread(string(frames[currentRighFrame]->fileName),CV_LOAD_IMAGE_ANYDEPTH);

    if(leftImg.cols == 0)
    {
        cout << "Image " << frames[currentLeftFrame]->fileName << " not found!!" << endl;
        return false;
    }
    if(righImg.cols == 0)
    {
        cout << "Image " << frames[currentRighFrame]->fileName << " not found!!" << endl;
        return false;
    }

    // Deal with diferent resolution of image mask and acoustic images
    if(grayMask.rows != leftImg.rows ||
       grayMask.cols != leftImg.cols)
    {
        resize(grayMask,
               grayMask,
               Size(leftImg.cols,leftImg.rows)
               );
    }
    if(bgrMask.rows != leftImg.rows ||
       bgrMask.cols != leftImg.cols)
    {
        resize(bgrMask,
               bgrMask,
               Size(leftImg.cols,leftImg.rows)
               );
    }

    // Apply sonar correction
    //    applyMeanCorrection(imgFr1);
    //    applyMeanCorrection(imgFr2);

    //    medianBlur(imgFr1,imgFr1,3);
    //    medianBlur(imgFr2,imgFr2,3);

    windowFetaure->processImages(leftImg,currentLeftFrame,
                                 righImg,currentRighFrame);

    return true;
}

void WindowTool::loadNextFrame()
{
    // Save Current State
//    saveGroundTruth("../../GroundTruth/Yatcht_05_12_2014_especifico.txt");

    // Load Imgs
    if(selecFrame == 0 && currentLeftFrame < currentRighFrame-1)
    {
        currentLeftFrame++;
    }else if(selecFrame == 1 && currentRighFrame < frames.size()-1)
    {
        currentRighFrame++;
    }
    else if(selecFrame == -1 && currentRighFrame < frames.size()-1)
    {
        currentRighFrame++;
        currentLeftFrame++;
    }

    loadCurrentFrame();
}

void WindowTool::loadPreviewFrame()
{
    // Load Imgs
    if(selecFrame == 0 && currentLeftFrame > 0)
    {
        currentLeftFrame--;
    }else if(selecFrame == 1 && currentRighFrame > currentLeftFrame +1)
    {
        currentRighFrame--;
    }
    else if(selecFrame == -1 && currentLeftFrame > 0)
    {
        currentRighFrame--;
        currentLeftFrame--;
    }

    loadCurrentFrame();
}


void WindowTool::selectFrame(int x, int y)
{
    removeSelection();
    if(x < windowSize.width/2)
    {
        selecFrame = 0;
    }else
    {
        selecFrame = 1;
    }
}

void WindowTool::removeSelection()
{
    selecFrame = -1;
}

void WindowTool::mouseUP(int x, int y)
{
//    int mFrame, mGauss;

//    if(selecFrame == 0 && selecGauss >= 0 && mGauss == -1)
//    {
//        frames[currentLeftFrame].removeGaussianMatchToFrame(selecGauss,currentRighFrame);
//        loadCurrentFrame();
//    }
//    else if(mFrame > selecFrame &&
//            mGauss >=0 && selecGauss >= 0)
//    {
//        cout << "Match between frame " << currentLeftFrame << " gaussian " << selecGauss
//             << " with frame " << currentRighFrame << " gaussian " << mGauss  << endl;

//        frames[currentLeftFrame].newMatch(selecGauss,currentRighFrame,mGauss);

//        loadCurrentFrame();
//    }
}

void WindowTool::mouseEvent(int event, int x, int y)
{
    windowFetaure->mouseEvent(event,x,y);

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:

            #ifdef GROUND_TRUTH_MOUSE_DEBUG
                cout << "LEFT DOWN" << "(" << x << " , " << y << ")" << endl;
            #endif

            selectFrame(x,y);
            renderImgsOnResult();
        break;
        case CV_EVENT_LBUTTONUP:

            #ifdef GROUND_TRUTH_MOUSE_DEBUG
                cout << "LEFT UP" << "(" << x << " , " << y << ")" << endl;
            #endif

            mouseUP(x,y);
            renderImgsOnResult();
        break;
        case CV_EVENT_RBUTTONDOWN:

            #ifdef GROUND_TRUTH_MOUSE_DEBUG
                cout << "RIGHT DOWN" << "(" << x << " , " << y << ")" << endl;
            #endif

            removeSelection();
        break;
        case CV_EVENT_RBUTTONUP:
            #ifdef GROUND_TRUTH_MOUSE_DEBUG
                cout << "RIGHT UP" << "(" << x << " , " << y << ")" << endl;
            #endif

            renderImgsOnResult();
        break;
        case CV_EVENT_MOUSEMOVE:
    //            cout << "MOVE " << "(" << x << " , " << y << ")" << endl;
        break;
    }

    mousePosition.x = x; mousePosition.y = y;

    //    if(showHomographyMatchs
    //       && showHomographyMouse
    //       && x < windowSize.width/2.f
    //       && matchHandler.hasHomography())
    //    {
    //        renderImgsOnResult();
    //    }

}


void on_WT_tbTh1(int v, void *userData)
{

}


void on_WT_tbTh2(int v, void *userData)
{

}


void on_WT_tbTh3(int v, void *userData)
{

}


void on_WT_mouse(int event, int x, int y, int, void *userData)
{
    WindowTool *wt = (WindowTool*) userData;
    wt->mouseEvent(event,x,y);
}

