#include "GroundTruth.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

#include "Drawing/Drawing.h"

GroundTruth::GroundTruth():
    threshold1(171),
    threshold2(215),
    windowName("GoundTruth"),
    windowControlName("Control"),
//    windowSize(1850,910),
    windowSize(1300,600),
//    windowSize(1430*2,781),
    currentFrame0(0),
    currentFrame1(1),
    selecFrame(-1),
    selecGauss(-1),
    showThresholdSegmentation(false),
    showMatch(false),
    showHomographyMatchs(true),
    showOpticalFlowMatchs(false),
    showContinuumFeatureMatchs(false),
    showSpatialMatchs(true),
    showHomographyMouse(true),
    showGaussians(false),
    showRegionsOfInterest(false),
    showKMeansResult(true),
    useColors(12),// 12 = GrayScale
    intensity(0),
    maxKindOfPixelAnalised(4000), // Used for K-Means auto threshold
    showIndividualFrameLeft(false),
    showIndividualFrameRight(false)
{

    segmentation.pixelThreshold = threshold2;
    segmentation.searchThreshold = threshold1;
    segmentation.minSampleSize = 15;
    segmentation.maxSampleSize = 150000;
//    segmentation.maxSampleSize = 12000;
//    segmentation.maxSampleSize = 1200;
    segmentation.searchDistance = 30;
    removeSelection();

    cout << "Loading insonification correction" << endl;
    insonificationCorrection = imread("../../GroundTruth/insonificationPatern_ARACATI2014.png",CV_LOAD_IMAGE_GRAYSCALE);

    insonificationMean = cv::mean(insonificationCorrection,segmentation.imgMask).val[0];
    cout << "Insonification Mean = " << insonificationMean << endl;

    insonificationCorrection.convertTo(insonificationCorrection,CV_16U);
}

GroundTruth::~GroundTruth()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:Destructor:: using destructor!" << endl;
    #endif

    cvDestroyAllWindows();
}

void GroundTruth::colorName(char *str)
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

void GroundTruth::saveCurrentImag(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:saveCurrentImag:: use!" << endl;
    #endif

    Mat gray8bits;
    char newFilename[300];

//    cvtColor(imgFr1,bgr8bits,CV_GRAY2BGR);
    imgFr1.convertTo(gray8bits,CV_8UC1);
    sprintf(newFilename, "%s_left.png", fileName);
    imwrite(newFilename, gray8bits);

    imgFr2.convertTo(gray8bits,CV_8UC1);
//    cvtColor(imgFr2,bgr8bits,CV_GRAY2BGR);

    sprintf(newFilename, "%s_right.png", fileName);
    imwrite(newFilename, gray8bits);

}

void GroundTruth::saveGroundTruth(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:saveGroundTruth:: use!" << endl;
    #endif

    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frame=0, g, u, i, nMatch;

    fprintf(f,"%u\n", frames.size());
    for(frame = 0 ; frame < frames.size() ; frame++)
    {
        Frame &fr = frames[frame];

        // Frame description
        // Image filename frameID NumberOfGaussians threshold1 and threhold2
        fprintf(f,"%s %u %u %u %u\n", fr.fileName.c_str() , frame, fr.gaussians.size(), fr.threshold1, fr.threshold2);

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        if(gs.size() == 0) continue;

        for(g = 0 ; g < gs.size(); g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fprintf(f,"%d %f %f %f %f %f %f %f %u\n", g, ga.x, ga.y , ga.z, ga.dx , ga.dy, ga.dz, ga.ang , ga.N );
        }

        // Match Description
        vector< vector<pair<unsigned,unsigned> > > &match = fr.match;
        nMatch=0;
        for(u = 0; u < match.size(); u++)
            nMatch += match[u].size();
        fprintf(f,"%u\n", nMatch);

        for(u = 0; u < match.size(); u++)
        {
            for(i = 0 ; i < match[u].size() ; i++)
            {
                fprintf(f,"%u %u %u\n", u, match[u][i].first, match[u][i].second);
            }
        }
    }
    fclose(f);
}

void GroundTruth::loadGroundTruth(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:loadGroundTruth:: use!" << endl;
    #endif

    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frame, g,frameSize,gaussiansSize, ug, vFr, vG, nMatch,i;
    char imgFileName[300];

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameSize);
    frames.resize(frameSize,Frame(string(""),0));
    for(frame = 0 ; frame < frames.size() ; frame++)
    {
        Frame &fr = frames[frame];

        // Frame description
        // Image filename frameID NumberOfGaussians
        fscanf(f,"%s %u %u %u %u",imgFileName, &fr.frameNumber, &gaussiansSize, &fr.threshold1, &fr.threshold2);
        fr.fileName = imgFileName;

        if(gaussiansSize == 0) continue;

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        gs.resize(gaussiansSize);
        for(g = 0 ; g < gaussiansSize; g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fscanf(f,"%*d %f %f %f %f %f %f %f %u",
                    &ga.x, &ga.y , &ga.z,
                    &ga.dx , &ga.dy, &ga.dz,
                    &ga.ang , &ga.N );
        }

        // Match Description
        vector< vector<pair<unsigned,unsigned> > > &match = fr.match;
        match.resize(gaussiansSize);
        fscanf(f,"%u",&nMatch);

        for( i = 0; i < nMatch; i++)
        {
            fscanf(f,"%u %u %u",&ug, &vFr, &vG);
            match[ug].push_back(pair<unsigned, unsigned>(vFr,vG));
        }
    }
    fclose(f);
}

bool GroundTruth::loadEmptyFrames(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:loadEmptyFrames:: use!" << endl;
    #endif

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
        frames.push_back(Frame(string(img_file_name),frame++));
    }
    fclose(f_img_names);
    return true;
}

void GroundTruth::mainLoop()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:mainLoop:: use!" << endl;
    #endif

    char c=0;
    bool autoGo=false;
    currentFrame0 = 0;
    currentFrame1 = 1;

    loadCurrentFrame();

    while(c != 27) // KEY_ESC
    {
        renderImgsOnResult();
//        computeThreshold();

        #ifdef GROUND_TRUTH_KEYBOARD_DEBUG
                cout << "pressed key " << (int) c << endl;
        #endif

        if(autoGo)
        {
            c = waitKey(300);
        }
        else
        {
           c = waitKey(0);
        }

        switch(c) // Preview frame
        {
        case ',':
            loadPreviewFrame();
        break;
        case '.': // Next Frame
        case  -1:
            loadNextFrame();
        break;
        case 'v': // Change visualization
            showThresholdSegmentation= !showThresholdSegmentation;
        break;
        case 'e': // Show Matchings
            switchShowMatch();
        break;
        case 'c': // Change view color
            useColors = (useColors+1)%13;
        break;
        case 'g': // Show Gaussians
            switchShowGaussians();
        break;
        case 's': // Save GroundTruth
            saveGroundTruth("../GroundTruthBkp.txt");
        break;
        case 'h': // Show Homograpy matchs
            showHomographyMatchs = !showHomographyMatchs;
        break;
        case 'a': // Compute Auto Threshold
            if(selecFrame>=0)
            {
                if(selecFrame==0)
                    computeAutoThreshold(imgFr1, &threshold1,&threshold2);
                else if (selecFrame==1)
                    computeAutoThreshold(imgFr2, &threshold1,&threshold2);

                setTrackbarPos("Th1",windowControlName,threshold1);
                setTrackbarPos("Th2",windowControlName,threshold2);
            }
        break;
        case 'd': // Compute Auto Threshold
            saveCurrentImag("display");
        break;
        case 'q': // Compute Auto Threshold
            autoGo = !autoGo;
        break;
        case ' ': // Remove selections
            removeSelection();
        break;
        case 13: // Key enter, do all homography match
        case 10: // Key enter, do all homography match
            removeSelection();
            acceptHomographyMatchs();
        break;
        case 'z':
            applyMeanCorrection(imgFr1);
            applyMeanCorrection(imgFr2);
        break;
        case 'm':
            addToMean();
        break;
        case 'n':
            showMeam();
        break;
        case 'b':
            cleamMean();
        break;
        case '\\':
            // Show mouse homographic projection (just update display)
        break;
        case 'u':
            // Save display image
            imwrite("Display.png", result);
        break;
        case 'o': // Compute optical flow matchs
            matchHandler.matchWithOpticalFlow(frames[currentFrame0], frames[currentFrame1],
                                              imgFr1,imgFr2);
        break;
        case 'i': // Show/Hide optical flow matchs
            showOpticalFlowMatchs = !showOpticalFlowMatchs;
        break;
        case 'p': // Accept optical flow matchs
            acceptOpticalFlowMatchs();
        break;
        case 't': // Compute continuum matchs
            matchHandler.consecutiveMatch(frames,currentFrame0,currentFrame1);
        break;
        case 'r': // Show/Hide continuum matchs
            showContinuumFeatureMatchs = !showContinuumFeatureMatchs;
        break;
        case 'y': // Accept optical continuums matchs
            acceptContinuumMatchs();
        break;
        case 'x': // Make a mosaic of two images
            makeCurrentMosaic();
        break;
        case '1':
            showIndividualFrameLeft= !showIndividualFrameLeft;
//            segmentation.createAdaptativeThreshold(imgFr1);
        break;
        case '2':
            showIndividualFrameRight= !showIndividualFrameRight;
//            segmentation.createAdaptativeThresholdOtsu(imgFr1);
        break;
        case '3':
            switchShowRegionsOfInterese();
//            segmentation.createPeakImg(imgFr1);
        break;
        }
    }
}

void GroundTruth::start()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:start:: use!" << endl;
    #endif

    namedWindow(windowName, 1);
    namedWindow(windowControlName, 1);

    setMouseCallback(windowName, onMouse, this);

    createTrackbar("Th1", windowControlName, &threshold1, 2000, on_tbTh1, this);
    createTrackbar("Th2", windowControlName, &threshold2, 2000, on_tbTh2, this);
    createTrackbar("Intensity", windowControlName, &intensity, 1000, on_tbTh3, this);

//    createTrackbar("Th1", NULL, &threshold1, 2000, on_tbTh1, this);
//    createTrackbar("Th2", NULL, &threshold2, 2000, on_tbTh2, this);
//    createTrackbar("Intensity", NULL, &intensity, 1000, on_tbTh3, this);

//    cvCreateButton("ShowGaussians",on_GT_ShowGaussian,this,CV_CHECKBOX,1);
//    cvCreateButton("ShowMatch",on_GT_ShowMatch,this,CV_CHECKBOX,1);
//    cvCreateButton("ShowRegionsOfInterest",on_GT_ShowRegionsOfInterest,this,CV_CHECKBOX,1);


    mainLoop();
}

void GroundTruth::computeAutoThreshold(Mat &img16Bits, int *tr1, int *tr2)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:computeAutoThreshold:: use!" << endl;
    #endif

    if(pixelIntensity.rows != maxKindOfPixelAnalised || pixelIntensity.cols != 1)
    {
        pixelIntensity = Mat(maxKindOfPixelAnalised,1,CV_32FC1,Scalar(0.f));
        cout << "Realoc pixelIntensitys!!" << endl;
    }
    nPixelIntensity = 0;

    int sz[1] = {1000};
    SparseMat pixels(1,sz,CV_8U);
    uchar *elem;
    unsigned row,col;
    int intensity, maxIntensity=0;

    for(row = 0 ; row < img16Bits.rows; row++)
    {
        for(col = 0 ; col < img16Bits.cols; col++)
        {
            intensity = img16Bits.at<short unsigned int>(row,col);
            elem = (uchar*) pixels.ptr(intensity,true);
            if(*elem == 0)
            {
                if(maxIntensity< intensity)maxIntensity = intensity;
                pixelIntensity.at<float>(nPixelIntensity,0) = intensity;
                nPixelIntensity++;
                (*elem)++;

                if(nPixelIntensity >= maxKindOfPixelAnalised)
                {
                    row = img16Bits.rows, col = img16Bits.cols;
                    cout << "AutoThreshold: Kind of Intensitys exceded!!!" << endl;
                }
            }
        }
    }

    Mat intensitys = pixelIntensity(Rect(0,0,1,nPixelIntensity)),
        labels, centers;

    int nCluster = 3;
    double compactness;

    compactness = kmeans(intensitys,nCluster,labels,
           TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), // 10 iteration or less than 1 correction
           1 , KMEANS_PP_CENTERS, centers); // Use Arthur and S. Vassilvitskii. k-means++ to compute seeds for 3 time
    float c[nCluster];

    for(int i = 0 ; i < nCluster; i++)
    {
        c[i] = centers.at<float>(i,0);
    }

    sort(c,c+nCluster);

    for(int i = 0 ; i < nCluster; i++)
    {
        cout << "c" << i << " = " << c[i] <<endl;
    }

    if(showKMeansResult)
    {
        Mat km(20,1200,CV_8UC3,Scalar(0,0,0));
        float ex = 1200.f/maxIntensity;
        for(unsigned i = 0 ; i < nPixelIntensity;i++)
        {
            int row = pixelIntensity.at<float>(i,0);
            circle(km,Point2i(ex*row,10),2,Scalar(255,255,255));
        }

        for(unsigned i = 0 ; i < nCluster;i++)
        {
            circle(km,Point2i(ex*c[i],10),5,Scalar(0,0,255));
        }
        imshow("Kmenas Result", km);
    }

    *tr1 = c[0]/2;
//    *tr2 = c[1];
    *tr2 = c[0];

//    *tr1 = (c[1]+c[2])/2;
//    *tr2 = (c[0]+c[1])/2;
    cout << "KMeans compacness = " << compactness << endl;
    cout << "tr1 " << *tr1 << " tr2 " << *tr2 << endl;
    cout << "maxIntensity " << maxIntensity << endl;
}

void GroundTruth::computeThreshold(Mat &srcImg, Mat &destImg)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:computeThreshold:: use!" << endl;
    #endif

    if(destImg.rows != srcImg.rows || destImg.cols != srcImg.cols)
    {
        destImg = Mat(srcImg.rows,srcImg.cols,CV_8UC3);
        cout << "Realoc destImg!!!" << endl;
    }

    if(threshold2 < threshold1)
    {
        threshold1 = threshold2 - 1;

        if(threshold2<= 1)
        {
            threshold1=0;
            threshold2=1;
        }
    }
    setTrackbarPos("Th1",windowControlName,threshold1);
    setTrackbarPos("Th2",windowControlName,threshold2);

    for(unsigned i = 0 ; i < srcImg.rows ; i++)
    {
        for(unsigned j = 0 ; j < srcImg.cols; j++)
        {
            ushort pixel = srcImg.at<ushort>(i,j);
            if(pixel < threshold1)
            {
                destImg.at<uchar>(i,j*3+0) = 255;
                destImg.at<uchar>(i,j*3+1) = 0;
                destImg.at<uchar>(i,j*3+2) = 0;
            }else if(pixel < threshold2)
            {
                destImg.at<uchar>(i,j*3+0) = 0;
                destImg.at<uchar>(i,j*3+1) = 255;
                destImg.at<uchar>(i,j*3+2) = 0;
            }else
            {
                destImg.at<uchar>(i,j*3+0) = 0;
                destImg.at<uchar>(i,j*3+1) = 0;
                destImg.at<uchar>(i,j*3+2) = 255;
            }
        }
    }
}

void GroundTruth::drawGaussian(Mat &img, Gaussian &g,const Scalar &lineColor,const Scalar &txtColor,
                               int id, bool drawElipses, bool drawVertexID, bool drawElipsesAngle)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:drawGaussian:: use!" << endl;
    #endif

    if(drawElipses)
    {
        ellipse(img,
                Point2f(g.x, g.y),
                Size2f(g.dx, g.dy),
                g.ang,
                0.0 , 360.0,
                lineColor,
                2 , 8 , 0);
    }

    char tempStr[200];
    if(drawVertexID)
    {
        sprintf(tempStr, "ID %d", id);
        putText(img,tempStr,
               Point2f(g.x+20, g.y+20),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);
    }

    if(drawElipsesAngle)
    {
        sprintf(tempStr, "%.2f", g.ang);
        putText(img,tempStr,
               Point2f(g.x+40, g.y),
               FONT_HERSHEY_COMPLEX,0.5,
               txtColor,2);

        line(img,
             Point2f(g.x, g.y),
             Point2f(g.x + 50.f*sin(g.ang*M_PI/180.f),
                     g.y - 50.f*cos(g.ang*M_PI/180.f)),
             txtColor,2);
    }
}

bool GroundTruth::findMatch(unsigned ufr, float *ux, float *uy, unsigned vfr, float *vx, float *vy, float prec)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:findMatch:: use!" << endl;
    #endif

    int guID, gvID;
    Frame &uf = frames[ufr], &vf = frames[vfr];

    Gaussian *ug = uf.findGaussianPrc(*ux, *uy,&guID,prec),
             *vg = 0x0;

    if(guID == -1)
        return false;


    uf.findMatch(guID,vfr,&gvID);

    if(gvID == -1)
        return false;

    vg = &vf.gaussians[gvID];


    *ux = ug->x;
    *uy = ug->y;

    *vx = vg->x;
    *vy = vg->y;

    return true;
}

unsigned GroundTruth::nMatchs(unsigned ufr, unsigned vfr)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:nMatchs:: use!" << endl;
    #endif

    Frame &uf = frames[ufr];
    return uf.nMatchs(vfr);
}

void GroundTruth::buildMatchPoints(unsigned srcFr, unsigned destFr, vector<Point2f> &src, vector<Point2f> &dst)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:buildMatchPoints:: use!" << endl;
    #endif

    vector<Gaussian>
          &gu = frames[srcFr].gaussians,
          &gv = frames[destFr].gaussians;

    vector< vector<pair<unsigned,unsigned> > >
          &match = frames[srcFr].match;

    for(unsigned u = 0 ; u < match.size(); u++)
    {
        for(unsigned i = 0 ; i < match[u].size() ; i++)
        {
            if(match[u][i].first == destFr)
            {
                unsigned v = match[u][i].second;
                src.push_back(Point2f(gu[u].x, gu[u].y));
                dst.push_back(Point2f(gv[v].x, gv[v].y));
            }
        }
    }
}

void GroundTruth::renderImgsOnResult()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:renderImgsOnResult:: use!" << endl;
    #endif

    if(result.cols != windowSize.width || result.rows != windowSize.height)
    {
        #ifdef GROUND_TRUTH_MEMORY_DEBUG
            cout << "GroundTruth:renderImgsOnResult:Allocing Img for Result display" << endl;
        #endif
        result = Mat(windowSize.height, windowSize.width,CV_8UC3);
    }

    Size imSZ(windowSize.width/2,windowSize.height);

    Mat cimg1,
        cimg2;

//    segmentation.createPeakImg(imgFr1);
//    segmentation.createGradientImg(imgFr1);
//    segmentation.createLogImg(imgFr1);
//    segmentation.createImg8bitsTruncated(imgFr1);
//    segmentation.createRTImage(imgFr1);
//    segmentation.createRTPlot(imgFr1);

    if(showThresholdSegmentation)
    {
        computeThreshold(imgFr1,cimg1);
        computeThreshold(imgFr2,cimg2);
    }else
    {
        imgFr1.convertTo(cimg1,CV_8UC3,1,-intensity);
        imgFr2.convertTo(cimg2,CV_8UC3,1,-intensity);
        Mat mask;
        cvtColor(segmentation.imgMask,mask,CV_GRAY2BGR);

        if(useColors != 12)
        {
            applyColorMap(cimg1,cimg1,useColors);

            applyColorMap(cimg2,cimg2,useColors);
        }else
        {
            cvtColor(cimg1,cimg1,CV_GRAY2BGR);
            cvtColor(cimg2,cimg2,CV_GRAY2BGR);
        }
        bitwise_and(cimg1,mask,cimg1);
        bitwise_and(cimg2,mask,cimg2);
    }

    vector<Gaussian> &gFr1 = frames[currentFrame0].gaussians,
                     &gFr2 = frames[currentFrame1].gaussians;

    // Drawing Gaussians
    if(showGaussians)
    {
        for(unsigned i = 0 ; i < gFr1.size(); i++)
        {
            drawGaussian(cimg1,gFr1[i],Scalar(0,0,255),Scalar(0,255,0),i);
        }
        for(unsigned i = 0 ; i < gFr2.size(); i++)
        {
            drawGaussian(cimg2,gFr2[i],Scalar(0,0,255),Scalar(0,255,0),i);
        }
    }
    if(selecFrame == 0 && selecGauss >= 0)
    {
        drawGaussian(cimg1,gFr1[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
    }else if(selecFrame == 1 && selecGauss >= 0)
    {
        drawGaussian(cimg2,gFr2[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
    }

    if(showMatch && showSpatialMatchs)
    {
        matchHandler.spatialMatch(frames[currentFrame0],frames[currentFrame1],
                                  250.f,
                                  cimg1,cimg2);
    }

    if(showIndividualFrameLeft)
    {
        imshow("Individual Frame Left", cimg1);
    }

    if(showIndividualFrameRight)
    {
        imshow("Individual Frame Right", cimg2);
    }

    Drawing::drawImgsTogether(windowSize,cimg1,cimg2,
                              result,&el,&er);

    // Draw texts
    char tempStr[200];
    sprintf(tempStr,
            "Frame %d-%d , FS %d G %d",
            currentFrame0,currentFrame1,selecFrame,selecGauss);
    putText(result,tempStr,
           Point2f(20, 20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);

    sprintf(tempStr,
            "Frame %d , Th1 %u Th2 %u",
            currentFrame0,frames[currentFrame0].threshold1,
            frames[currentFrame0].threshold2);
    putText(result,tempStr,
           Point2f(10, imSZ.height-20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);


    sprintf(tempStr,
            "Frame %d , Th1 %u Th2 %u",
            currentFrame1,frames[currentFrame1].threshold1,
            frames[currentFrame1].threshold2);
    putText(result,tempStr,
           Point2f(imSZ.width+10,imSZ.height-20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);

    colorName(tempStr);
    putText(result,tempStr,
           Point2f(imSZ.width-100,20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,255),2);


    // Drawing optical flow matchs
    if(showMatch && showOpticalFlowMatchs)
    {
        Drawing::drawLineMatch(result,el,er,
                               matchHandler.OFMatchSrc,matchHandler.OFMatchDest,
                               Scalar(0,255,255),2);
    }

    // Draw Spatial Matchs
    if(showMatch && showSpatialMatchs)
    {
        Drawing::drawLineMatch(result,el,er,
                               matchHandler.SPMatchSrc,matchHandler.SPMatchDest,
                               Scalar(255,255,0),1);
    }

    // Drawing consecutive matchs
    if(showMatch && showContinuumFeatureMatchs)
    {
        Drawing::drawLineMatch(result,el,er,
                               matchHandler.CMatchSrc,matchHandler.CMatchDest,
                               Scalar(0,255,0),3);
    }

    // Draw Homographic matchs
    if(showMatch && showHomographyMatchs && matchHandler.hasHomography())
    {
        Drawing::drawLineMatch(result,el,er,
                               matchHandler.HMatchSrc,matchHandler.HMatchDest,
                               Scalar(255,0,0),3);
    }

    if(showHomographyMouse && mousePosition.x < result.cols/2 && matchHandler.hasHomography())
    {
        Point2f tMouse;
        matchHandler.transform(
                    Point2f((mousePosition.x-el.val[2])/el.val[0], (mousePosition.y-el.val[3])/el.val[1]),
                    &tMouse);

        circle(result,
               Point2f(er.val[2] + er.val[0]*tMouse.x, er.val[3] + er.val[1]*tMouse.y),
               5, Scalar(0,255,0));
    }

    // Draw Matching
    if(showMatch)
    {
        Drawing::drawLineMatch(result,el,er,
                               matchHandler.GTMatchSrc,matchHandler.GTMatchDest,
                               Scalar(0,0,255),2);
    }

    // Drawing selected Matching
    if(selecFrame == 0 && selecGauss >= 0)
    {
        Point2f src, dest;
        // Homography matching
        if(matchHandler.HMatchSrc.size() >= 4
           && matchHandler.findHMatch(selecGauss,&src, &dest))
        {
            Drawing::drawLineMatch(result,el,er,
                                   src,dest,
                                   Scalar(255,0,0),3);
        }
        // Ground Truth Matching
        if(matchHandler.findGTMatch(selecGauss,&src, &dest))
        {
            Drawing::drawLineMatch(result,el,er,
                                   src,dest,
                                   Scalar(0,0,255),2);
        }
    }

    imshow(windowName,result);
}


bool GroundTruth::loadCurrentFrame()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:loadCurrentFrame:: use!" << endl;
    #endif

    imgFr1 = imread(string(frames[currentFrame0].fileName),CV_LOAD_IMAGE_ANYDEPTH);
    imgFr2 = imread(string(frames[currentFrame1].fileName),CV_LOAD_IMAGE_ANYDEPTH);

    if(imgFr1.cols == 0)
    {
        cout << "Image " << frames[currentFrame0].fileName << " not found!!" << endl;
        return false;
    }
    if(imgFr2.cols == 0)
    {
        cout << "Image " << frames[currentFrame1].fileName << " not found!!" << endl;
        return false;
    }

    // Deal with resolution diferences between image mask and acoustic images
    if(segmentation.imgMask.rows != imgFr1.rows ||
       segmentation.imgMask.cols != imgFr2.cols)
    {
        resize(segmentation.imgMask,
               segmentation.imgMask,
               Size(imgFr1.cols,imgFr1.rows)
               );
    }

    // Apply sonar correction
//    applyMeanCorrection(imgFr1);
//    applyMeanCorrection(imgFr2);

//    medianBlur(imgFr1,imgFr1,3);
//    medianBlur(imgFr2,imgFr2,3);

    if(frames[currentFrame0].gaussians.size() == 0)
        createGaussian(&frames[currentFrame0],imgFr1);

    if(frames[currentFrame1].gaussians.size() == 0)
        createGaussian(&frames[currentFrame1],imgFr2);

    matchHandler.loadMatchs(frames,
                            frames[currentFrame0],frames[currentFrame1],
                            imgFr1, imgFr2,10.f);

    return true;
}

void GroundTruth::loadNextFrame()
{
    // Save Current State
//    saveGroundTruth("../../GroundTruth/Yatcht_05_12_2014_especifico.txt");

    // Load Imgs
    if(selecFrame == 0 && currentFrame0 < currentFrame1-1)
    {
        currentFrame0++;
        selecGauss = -1;
    }else if(selecFrame == 1 && currentFrame1 < frames.size()-1)
    {
        currentFrame1++;
        selecGauss = -1;
    }
    else if(selecFrame == -1 && currentFrame1 < frames.size()-1)
    {
        currentFrame1++;
        currentFrame0++;
        selecGauss = -1;
    }

    loadCurrentFrame();

}

void GroundTruth::loadPreviewFrame()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:loadPreviewFrame:: use!" << endl;
    #endif


    // Load Imgs
    if(selecFrame == 0 && currentFrame0 > 0)
    {
        currentFrame0--;
        selecGauss = -1;
    }else if(selecFrame == 1 && currentFrame1 > currentFrame0 +1)
    {
        currentFrame1--;
        selecGauss = -1;
    }
    else if(selecFrame == -1 && currentFrame0 > 0)
    {
        currentFrame1--;
        currentFrame0--;
        selecGauss = -1;
    }

    loadCurrentFrame();
}

void GroundTruth::selectGaussOnFrame(int x, int y, int *frame, int *gauss)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:selectGaussOnFrame:: use!" << endl;
    #endif

    // Try select Gaussian of Frame
    if(x < windowSize.width/2)
    {
        *frame = 0;
        frames[currentFrame0].findGaussian((x-el.val[2])/el.val[0],(y-el.val[3])/el.val[1], gauss);
    }else
    {
        *frame = 1;
        frames[currentFrame1].findGaussian((x-er.val[2])/er.val[0],(y-er.val[3])/er.val[1], gauss);
    }
}

void GroundTruth::select(int x, int y)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:select:: use!" << endl;
    #endif

    removeSelection();
    selectGaussOnFrame(x,y,&selecFrame,&selecGauss);
}

void GroundTruth::removeSelection()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:removeSelection:: use!" << endl;
    #endif

    selecGauss = selecFrame = -1;
}

void GroundTruth::acceptHomographyMatchs()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:acceptHomographyMatchs:: use!" << endl;
    #endif

    vector<unsigned> gSrcList, gDestList;
    matchHandler.HMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    Frame &currentFrame = frames[currentFrame0];

    // Remove all matchs between frame0 (right displayed) to frame1(left dysplayed)
    currentFrame.removeMatchToFrame(currentFrame1);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], currentFrame1,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    loadCurrentFrame();
}

void GroundTruth::acceptOpticalFlowMatchs()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:acceptOpticalFlowMatchs:: use!" << endl;
    #endif

    vector<unsigned> gSrcList, gDestList;
    matchHandler.OFMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    Frame &currentFrame = frames[currentFrame0];

    // Remove all matchs between frame0 (right displayed) to frame1(left dysplayed)
    currentFrame.removeMatchToFrame(currentFrame1);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], currentFrame1,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    loadCurrentFrame();
}

void GroundTruth::acceptContinuumMatchs()
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:acceptContinuumMatchs:: use!" << endl;
    #endif

    vector<unsigned> gSrcList, gDestList;
    matchHandler.CFMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    Frame &currentFrame = frames[currentFrame0];

    // Remove all matchs between frame0 (right displayed) to frame1(left dysplayed)
    currentFrame.removeMatchToFrame(currentFrame1);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], currentFrame1,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    loadCurrentFrame();
}

void GroundTruth::makeCurrentMosaic()
{
    Mat img8Bits_1, img8Bits_2;

    imgFr1.convertTo(img8Bits_1,CV_8UC1);
    imgFr2.convertTo(img8Bits_2,CV_8UC1);

    matchHandler.createMosaicImage(img8Bits_1,img8Bits_2);
}

void GroundTruth::createGaussian(int frame,int x, int y, Gaussian *g)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:createGaussian:: use!" << endl;
    #endif

    // Create a gaussian
    Segment seg;

    segmentation.searchThreshold = threshold1;
    segmentation.pixelThreshold = threshold2;

    if(frame==0)
    {
        segmentation.resetMask(imgFr1.rows, imgFr1.cols);
        segmentation.createSegment(&seg,imgFr1,y,x);
    }else if(frame == 1)
    {
        segmentation.resetMask(imgFr2.rows, imgFr2.cols);
        segmentation.createSegment(&seg,imgFr2,y,x);
    }
    createGaussian(&seg,g);
}

void GroundTruth::createGaussian(Frame *frame, Mat &img16Bits)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:createGaussian:: use!" << endl;
    #endif

    // Just select gaussians becouse user is change treshold tackbar
    // the selected frame need to stay selected.
    selecGauss = -1;

    // Clear old Gaussians
    frame->gaussians.clear();
    frame->match.clear();

    #ifdef GROUND_TRUTH_MEMORY_DEBUG
        cout << "GroundTruth:createGaussian: Creating Gaussians!!" << endl;
    #endif

    vector<Segment*> segs;
    Gaussian g;

    // Peaks Threshold segmentation
    segmentation.searchThreshold = threshold1;
    segmentation.pixelThreshold = threshold2;
    segmentation.segment(img16Bits,&segs);

    // Peaks Segmentation
//    segmentation.maxPeakSize = 200;
//    segmentation.peakThreshold = 0;
//    segmentation.segmentWithPeaks(img16Bits,&segs);

    // Theta Rho Segmentation (without threshold)
//    segmentation.segmentWithTR(img16Bits, &segs);

    frame->threshold1 = threshold1;
    frame->threshold2 = threshold2;

    for(unsigned i = 0 ; i < segs.size(); i++)
    {
        createGaussian(segs[i],&g);
        g.dx*=3.f; g.dy*=3.f; g.dz*=3.f;        
        frame->gaussians.push_back(g);
    }
    frame->match.resize(frame->gaussians.size());
}

void GroundTruth::createGaussian(Segment *seg, Gaussian *g)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:createGaussian:: use!" << endl;
    #endif

    Mat sample = seg->result(Rect(0,0,seg->N,3)),
        mCov, mMean, eVal, eVect;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(sample, mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE ,
                    CV_32F);

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, eVal, eVect);

    g->x = mMean.at<float>(1,0);
    g->y = mMean.at<float>(0,0);
    g->z = mMean.at<float>(2,0);

    g->dx = sqrt(eVal.at<float>(2,0));
    g->dy = sqrt(eVal.at<float>(1,0));
    g->dz = sqrt(eVal.at<float>(0,0));
    g->N = seg->N;

    float ang;
    ang = 180.0*atan2(eVect.at<float>(2,0),eVect.at<float>(2,1))/M_PI;
    if(ang<0.f) ang+= 360.f;
    if(ang>180.f)ang-=180.f;

    g->ang = ang;
}

void GroundTruth::mouseUP(int x, int y)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:mouseUP:: use!" << endl;
    #endif

    int mFrame, mGauss;
    selectGaussOnFrame(x,y,&mFrame, &mGauss);

    if(selecFrame == 0 && selecGauss >= 0 && mGauss == -1)
    {
        frames[currentFrame0].removeGaussianMatchToFrame(selecGauss,currentFrame1);
        loadCurrentFrame();
    }
    else if(mFrame > selecFrame &&
            mGauss >=0 && selecGauss >= 0)
    {
        cout << "Match between frame " << currentFrame0 << " gaussian " << selecGauss
             << " with frame " << currentFrame1 << " gaussian " << mGauss  << endl;

        frames[currentFrame0].newMatch(selecGauss,currentFrame1,mGauss);

        loadCurrentFrame();
    }
}

void GroundTruth::mouseEvent(int event, int x, int y)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
//        cout << "GroundTruth:mouseEvent:: use!" << endl;
    #endif

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:

            #ifdef GROUND_TRUTH_MOUSE_DEBUG
                cout << "LEFT DOWN" << "(" << x << " , " << y << ")" << endl;
            #endif

            select(x,y);
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

void GroundTruth::tb1(int)
{
    if(threshold1 >= threshold2)
    {
        threshold2 = threshold1 + 1;
        if(threshold2>= 2000)
        {
            threshold1=2000;
            threshold2=2001;
        }
        setTrackbarPos("Th1",windowControlName,threshold1);
        setTrackbarPos("Th2",windowControlName,threshold2);
        return; // Th2 draw new status
    }

    // Compute new Gaussians with new thresholds
    if(selecFrame == 0)
        createGaussian(&frames[currentFrame0],imgFr1);

    if(selecFrame == 1)
        createGaussian(&frames[currentFrame1],imgFr2);

    if(selecFrame >= 0)
        renderImgsOnResult();
}

void GroundTruth::tb2(int)
{
    if(threshold2 <= threshold1)
    {
        threshold1 = threshold2 - 1;
        if(threshold2<= 1)
        {
            threshold1=0;
            threshold2=1;
        }
        setTrackbarPos("Th2",windowControlName,threshold2);
        setTrackbarPos("Th1",windowControlName,threshold1);
        return; // Th1 draw new status
    }

    // Compute new Gaussians with new thresholds
    if(selecFrame == 0)
        createGaussian(&frames[currentFrame0],imgFr1);

    if(selecFrame == 1)
        createGaussian(&frames[currentFrame1],imgFr2);

    if(selecFrame >= 0)
        renderImgsOnResult();
}

void GroundTruth::tb3(int)
{
    renderImgsOnResult();
}

void GroundTruth::switchShowGaussians()
{
    showGaussians= !showGaussians;
}

void GroundTruth::switchShowMatch()
{
    showMatch= !showMatch;
}

void GroundTruth::switchShowRegionsOfInterese()
{
    showRegionsOfInterest = !showRegionsOfInterest;
}

void GroundTruth::addToMean()
{

    if(mMeam.rows == 0)
    {
        mMeam = Mat(imgFr1.rows, imgFr1.cols, CV_32S, Scalar(0));
        meanCount = Mat(imgFr1.rows, imgFr1.cols, CV_32S, Scalar(0));
    }


    for(unsigned i = 0 ; i < imgFr1.rows; i++)
    {
        for(unsigned j = 0 ; j < imgFr1.cols; j++)
        {
            if(!segmentation.imgMask.at<uchar>(i,j)) continue;
            if(imgFr1.at<ushort>(i,j) < 240)
            {
                mMeam.at<int>(i,j) += imgFr1.at<ushort>(i,j);
                meanCount.at<int>(i,j) ++;
            }
        }
    }
}

void GroundTruth::showMeam()
{
    if(mMeam.rows == 0) return;

//    normalize(mMeam,mMeam,0,255,NORM_MINMAX);
    Mat result = Mat(imgFr1.rows, imgFr1.cols, CV_32S, Scalar(0));

    for(unsigned i = 0 ; i < imgFr1.rows; i++)
    {
        for(unsigned j = 0 ; j < imgFr1.cols; j++)
        {
            if(!segmentation.imgMask.at<uchar>(i,j)) continue;
            if(meanCount.at<int>(i,j) > 0)
                result.at<int>(i,j) = mMeam.at<int>(i,j) / meanCount.at<int>(i,j);
            else
                result.at<int>(i,j) = mMeam.at<int>(i,j);
        }
    }

    result.convertTo(result,CV_8UC1);

//    cout << "Meam Count = " << meanCount << endl;

    imshow("Mean img", result);
    //    mMeam = Mat();
}

void GroundTruth::cleamMean()
{
    mMeam = Mat();
}

void GroundTruth::applyMeanCorrection(Mat &imgGray_16bits)
{    
    if(insonificationCorrection.rows == 0)
    {
        cout << "No insonification patern loaded!!" << endl;
        return;
    }

    cv::add(imgGray_16bits, Scalar(insonificationMean),imgGray_16bits,segmentation.imgMask);
    cv::subtract(imgGray_16bits, insonificationCorrection ,imgGray_16bits, segmentation.imgMask);
}

void on_tbTh1(int v, void *dt)
{
    GroundTruth *gt = (GroundTruth*) dt;
    gt->tb1(v);
}

void on_tbTh2(int v, void *dt)
{
    GroundTruth *gt = (GroundTruth*) dt;
    gt->tb2(v);
}

void on_tbTh3(int v, void* dt)
{
    GroundTruth *gt = (GroundTruth*) dt;
    gt->tb3(v);
}


void onMouse(int event, int x, int y, int, void *dt)
{
    GroundTruth *gt = (GroundTruth*) dt;
    gt->mouseEvent(event,x,y);
}

void on_GT_ShowGaussian(int state, void* userData)
{
    GroundTruth *gt = (GroundTruth*) userData;
    gt->switchShowGaussians();
}

void on_GT_ShowMatch(int state, void* userData)
{
    GroundTruth *gt = (GroundTruth*) userData;
    gt->switchShowMatch();
}

void on_GT_ShowRegionsOfInterest(int state, void* userData)
{
    GroundTruth *gt = (GroundTruth*) userData;
    gt->switchShowRegionsOfInterese();
}
