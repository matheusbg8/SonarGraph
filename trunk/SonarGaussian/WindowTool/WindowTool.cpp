#include "WindowTool.h"
#include "Drawing/Drawing.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

WindowTool::WindowTool(string windowName, WindowFeature *feature):
    windowName(windowName),
    threshold1(171),
    threshold2(215),
    windowControlName("Control"),
//    windowSize(1850,910),
//    windowSize(1300,600),
//    windowSize(1430*2,781),
    windowSize(0.68*2*1430,0.68*781),
//    windowSize(0.45*2*1430,0.45*781),
    currentLeftFrame(-1),
    currentRightFrame(-1),
    selecFrame(-1),
    useColors(12),// 12 = GrayScale
    intensity(0),
    maxKindOfPixelAnalised(4000), // Used for K-Means auto threshold
    showIndividualLeftFrame(false),
    showIndividualRightFrame(false)
{
    // Register me on feature for be directional communications
    feature->wt = this;
    addFeature(feature);
}

WindowTool::~WindowTool()
{
    cvDestroyAllWindows();
    clearFrames();
    //clearFetures();
}

void WindowTool::propagateSelectedFrame()
{
    int selecFrameId = selecFrame==-1?selecFrame:
                       selecFrame== 0?currentLeftFrame:
                                     currentRightFrame;

    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->selectedFrame(selecFrameId);
}

void WindowTool::addFeature(WindowFeature *wf)
{
    windowFeatures.push_back(wf);
    wf->wt = this;
}

void WindowTool::clearFetures()
{
    for(unsigned i = 0; i < windowFeatures.size();i++)
    {
        delete windowFeatures[i];
    }
    windowFeatures.clear();
}

void WindowTool::clearFrames()
{
    for(unsigned i =0 ; i < frames.size(); i++)
    {
        delete frames[i];
    }
    frames.clear();
    currentLeftFrame = -1;
    currentRightFrame = -1;

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

void WindowTool::loadImgMaskAndIsonificationPatern()
{
    cout << "Loading insonification correction" << endl;
    isonificationPattern = imread("IsonificationPattern.png",CV_LOAD_IMAGE_ANYDEPTH);
    grayMask = imread("Mask.png",CV_LOAD_IMAGE_ANYDEPTH);

    if(isonificationPattern.rows == 0 || grayMask.rows == 0)
    {
        // Creating isonification pattern and mask
//        createIsonificationPatternAndMask(240);
        createIsonificationPatternAndMask(65535);


        Mat tmpImg;
        isonificationPattern.convertTo(tmpImg,CV_8UC1);
        imshow("Isonification", tmpImg);
        imshow("Mask", grayMask);

        imwrite("IsonificationPattern.png", isonificationPattern);
        imwrite("Mask.png", grayMask);
    }

    insonificationMean = mean(isonificationPattern,grayMask).val[0];
    cout << "Insonification Mean = " << insonificationMean << endl;

    cvtColor(grayMask,bgrMask,CV_GRAY2BGR);
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
    char img_file_name[301];

    if(f_img_names == 0x0)
    {
        cout << "Image list file " << fileName << " not found!!" << endl;
        return false;
    }

    clearFrames();
    Frame *f=0x0;
    unsigned frame=0,size;
    while(fgets(img_file_name, 300, f_img_names) !=NULL)
    {
        size = strlen(img_file_name);
        if(size > 0)
        {
            img_file_name[size-1] = '\0';
            for(unsigned i = 0 ; i < windowFeatures.size(); i++)
                f = windowFeatures[i]->newFrame(string(img_file_name),frame);

            frame++;
            frames.push_back(f);
        }
    }
    fclose(f_img_names);

    loadImgMaskAndIsonificationPatern();

    return true;
}

bool WindowTool::saveEmptyFrames(const char *fileName)
{
    FILE *f_img_names = fopen(fileName,"w");

    if(f_img_names == 0x0)
    {
        cout << "WindowTool:saveEmptyFrames - Write problem when saving " << fileName << " !!" << endl;
        return false;
    }

    for(unsigned i = 0 ; i < frames.size(); i++)
    {
        fprintf(f_img_names,"%s\n", frames[i]->fileName.c_str());
    }

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
            imgSum = Mat(aImg.rows, aImg.cols, CV_32SC1, Scalar(0));
            validPixelCount = Mat(aImg.rows, aImg.cols, CV_32SC1, Scalar(0));
        }

        for(unsigned i = 0 ; i < aImg.rows; i++)
        {
            for(unsigned j = 0 ; j < aImg.cols; j++)
            {
                ushort pixel = aImg.at<ushort>(i,j);
                if(pixel > 0 && pixel < threadPixel)
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
    isonificationPattern = Mat(imgSum.rows, imgSum.cols, CV_16UC1, Scalar(0));
    grayMask = Mat(imgSum.rows, imgSum.cols,CV_8UC1, Scalar(0));

    for(unsigned i = 0 ; i < imgSum.rows; i++)
    {
        for(unsigned j = 0 ; j < imgSum.cols; j++)
        {
            if(validPixelCount.at<int>(i,j) > 0)
            {
                isonificationPattern.at<ushort>(i,j) = imgSum.at<int>(i,j) / validPixelCount.at<int>(i,j);
                grayMask.at<uchar>(i,j) = 255;
            }
            else
            {
                isonificationPattern.at<ushort>(i,j) = imgSum.at<int>(i,j);
            }
        }
    }

    imwrite("Test.png", grayMask);

//    isonificationPattern.convertTo(imgMean,CV_8UC1);
//    imshow("Computed Isonification Pattern", imgMean);
//    imshow("Computed Mask", grayMask);
//    waitKey();

}

void WindowTool::applyMeanCorrection(Mat &imgGray_16bits)
{
    if(isonificationPattern.rows == 0)
    {
        cout << "No insonification patern loaded!!" << endl;
        return;
    }

    cv::add(imgGray_16bits, Scalar(insonificationMean),imgGray_16bits,grayMask);
    cv::subtract(imgGray_16bits, isonificationPattern ,imgGray_16bits,grayMask);

}


/**
 * @brief
 *  This method only save information about its frames, i.e. class Frame
 * @param fileName
 * @return bool
 */
bool WindowTool::save(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
    {
        cout << "WindowTool: The file " << fileName << " can not be saved. May be not permission?" << endl;
        return false;
    }

    unsigned frameId=0;

    // NUmber of frames
    fprintf(f,"%u\n", frames.size());
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        Frame *fr = frames[frameId];

        fprintf(f,"%u,%s\n",
            fr->frameNumber,fr->fileName.c_str());
    }
    fclose(f);
    return true;
}


/**
 * @brief This method only load its frames
 *
 * @param fileName
 * @return bool
 */
bool WindowTool::load(const char *fileName)
{
    setlocale(LC_NUMERIC, "C");
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
    {
        cout << "WindowTool: File " << fileName << " not found to load" << endl;
        return false;
    }

    unsigned frameId,frameCount;
    char str[500];

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameCount);
    frames.resize(frameCount);
    for(frameId = 0 ; frameId < frames.size() ; frameId++)
    {
        // Frame description
        // Number of vms on this frame
        fscanf(f,"%*u,%s\n",str);
        frames[frameId] = new Frame(str,frameId);
    }
    fclose(f);

    loadImgMaskAndIsonificationPatern();

    return true;
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
        case 'q':
            save("test_wtool.txt");
        break;
        case 'n':
            showIndividualLeftFrame= !showIndividualLeftFrame;
        break;
        case 'm':
            showIndividualRightFrame= !showIndividualRightFrame;
        break;
        }

        for(unsigned i = 0; i < windowFeatures.size();i++)
            windowFeatures[i]->keyPress(c);
    }
}


/**
 * @brief Start this windows tool.
 *
 */
void WindowTool::start()
{
    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->start();

    if(frames.size() == 0)
    {
        cout << "WindowTool::Problem, no frames loaded" << endl;
        return ;
    }

    currentLeftFrame = 1;
    currentRightFrame = 135;

//    currentLeftFrame = 32;
//    currentRightFrame = 33;

//    currentLeftFrame = 0;
//    currentRightFrame = 1252;

//    currentLeftFrame = 2;
//    currentRightFrame = 644;

//    currentLeftFrame = 10181;
//    currentRightFrame = 10193;
    //    10194

    namedWindow(windowName, WINDOW_AUTOSIZE);
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

    leftImg.convertTo(cLeftImg,CV_8UC1,1,-intensity);
    righImg.convertTo(cRightImg,CV_8UC1,1,-intensity);

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
    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->renderProcess(cLeftImg,currentLeftFrame,
                                 cRightImg,currentRightFrame);

    // Blender images
    Drawing::drawImgsTogether(windowSize,cLeftImg,cRightImg,
                              result,&el,&er);

    // Process final screen
    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->renderFrameTogether(result,
                                               el,currentLeftFrame,
                                               er,currentRightFrame);
    imwrite("righFrame.png",cRightImg);

    if(showIndividualLeftFrame)
    {
        imshow("Individual Left Frame", cLeftImg);
    }

    if(showIndividualRightFrame)
    {
        imshow("Individual Right Frame", cRightImg);
    }

    // Draw texts
    char tempStr[200];
    sprintf(tempStr,
            "Frame %d-%d , FS %d",
            currentLeftFrame,currentRightFrame,selecFrame);
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
    righImg = imread(string(frames[currentRightFrame]->fileName),CV_LOAD_IMAGE_ANYDEPTH);

    if(leftImg.cols == 0)
    {
        cout << "Image " << frames[currentLeftFrame]->fileName << " not found!!" << endl;
        return false;
    }
    if(righImg.cols == 0)
    {
        cout << "Image " << frames[currentRightFrame]->fileName << " not found!!" << endl;
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
//    applyMeanCorrection(leftImg);
//    applyMeanCorrection(righImg);

//    medianBlur(leftImg,leftImg,3);
//    medianBlur(righImg,righImg,3);

//    GaussianBlur(leftImg,leftImg,Size(3,3),0,0);
//    GaussianBlur(righImg,righImg,Size(3,3),0,0);

//    Laplacian(leftImg,leftImg,CV_32FC1,3);
//    Laplacian(righImg,righImg,CV_32FC1,3);

//    leftImg.convertTo(leftImg,CV_16UC1);
//    righImg.convertTo(righImg,CV_16UC1);

    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->processImages(leftImg,currentLeftFrame,
                                 righImg,currentRightFrame);

    return true;
}

void WindowTool::loadNextFrame()
{
    // Save Current State
//    saveGroundTruth("../../GroundTruth/Yatcht_05_12_2014_especifico.txt");

    // Load Imgs
    if(selecFrame == 0 && currentLeftFrame < currentRightFrame-1)
    {
        currentLeftFrame++;
    }else if(selecFrame == 1 && currentRightFrame < frames.size()-1)
    {
        currentRightFrame++;
    }
    else if(selecFrame == -1 && currentRightFrame < frames.size()-1)
    {
        currentRightFrame++;
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
    }else if(selecFrame == 1 && currentRightFrame > currentLeftFrame +1)
    {
        currentRightFrame--;
    }
    else if(selecFrame == -1 && currentLeftFrame > 0)
    {
        currentRightFrame--;
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

    // Propagate the id of the new selected frame id
    propagateSelectedFrame();
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
    for(unsigned i = 0; i < windowFeatures.size();i++)
        windowFeatures[i]->mouseEvent(event,x,y);

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

bool WindowTool::isFrameSelected()
{
    if(selecFrame!=-1)
        return true;
    return false;
}

bool WindowTool::isCurrentRightFrame(unsigned frameId)
{
    return currentRightFrame == frameId;
}

bool WindowTool::isCurrentLeftFrame(unsigned frameId)
{
    return currentLeftFrame == frameId;
}

int WindowTool::selectedFrameId()
{
    return selecFrame==-1?selecFrame:
           selecFrame==0?currentLeftFrame:
                         currentRightFrame;
}

Mat &WindowTool::selectedFrameImg()
{
    if(selecFrame==0)
        return leftImg;
    else if(selecFrame ==1)
        return righImg;

    cout << "WindowTool: Problem when acessing image frame!" << endl;
    return leftImg;
}

int WindowTool::getLeftFrameId()
{
    return currentLeftFrame;
}

int WindowTool::getRighFrameId()
{
    return currentRightFrame;
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


void on_WT_mouse(int event, int x, int y, int flags, void *userData)
{
    WindowTool *wt = (WindowTool*) userData;
    wt->mouseEvent(event,x,y);
}

