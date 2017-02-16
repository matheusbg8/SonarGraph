#include "GroundTruth.h"

GroundTruth::GroundTruth():
    threshold1(64),
    threshold2(167),
    windowName("GoundTruth"),
//    windowSize(1850,910),
    windowSize(1300,600),
    currentFrame0(0),
    currentFrame1(1),
    selecFrame(-1),
    selecGauss(-1),
    showThresholdSegmentation(false),
    showMatch(true),
    showGaussians(true),
    showKMeansResult(true),
    maxKindOfPixelAnalised(4000)
{

    segmentation.pixelThreshold = threshold2;
    segmentation.searchThreshold = threshold1;
    segmentation.minSampleSize = 20;
//    segmentation.maxSampleSize = 150000;
    segmentation.maxSampleSize = 12000;
    segmentation.searchDistance = 30;
}

void GroundTruth::saveGroundTruth(const char *fileName)
{
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
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frame, g,frameSize,gaussiansSize, ug, vFr, vG, nMatch,i;
    char imgFileName[300];

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameSize);
    frames.resize(frameSize,Frame(string("")));
    for(frame = 0 ; frame < frames.size() ; frame++)
    {
        Frame &fr = frames[frame];

        // Frame description
        // Image filename frameID NumberOfGaussians
        fscanf(f,"%s %*u %u %u %u", imgFileName, &gaussiansSize, &fr.threshold1, &fr.threshold2);
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

void GroundTruth::loadEmptyFrames(const char *fileName)
{
    FILE *f_img_names = fopen(fileName,"r");
    char img_file_name[300];

    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        frames.push_back(Frame(string(img_file_name)));
    }
    fclose(f_img_names);
}

void GroundTruth::mainLoop()
{
    char c=0;
    currentFrame0 = 0;
    currentFrame1 = 1;

    loadCurrentFrame();

    while(c != 27) // KEY_ESC
    {
        renderImgsOnResult();
//        computeThreshold();
        cout << " key " << (int) c << endl;

        c = waitKey(0);

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
            showMatch= !showMatch;
        break;
        case 'c': // Clear Selected Frame
            if(selecFrame==0)
                frames[currentFrame0].removeAllMatch();
            else if(selecFrame==1)
                frames[currentFrame1].removeAllMatch();
        break;
        case 'g': // Show Matchings
            showGaussians= !showGaussians;
        break;
        case 's': // Save GroundTruth
            saveGroundTruth("../MachadoTrue.txt");
        break;
        case 'a': // Compute Auto Threshold
            if(selecFrame>=0)
            {
                if(selecFrame==0)
                    computeAutoThreshold(imgFr1, &threshold1,&threshold2);
                else if (selecFrame==1)
                    computeAutoThreshold(imgFr2, &threshold1,&threshold2);

                setTrackbarPos("Th1",windowName,threshold1);
                setTrackbarPos("Th2",windowName,threshold2);
            }
        break;
        }
    }
}

void GroundTruth::start()
{
    namedWindow(windowName, 1);

    setMouseCallback(windowName, onMouse, this);

    createTrackbar( "Th1", windowName, &threshold1, 2000, on_tbTh1, this);
    createTrackbar( "Th2", windowName, &threshold2, 2001, on_tbTh2, this);

    mainLoop();
}

void GroundTruth::computeAutoThreshold(Mat &img16Bits, int *tr1, int *tr2)
{
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
    setTrackbarPos("Th1",windowName,threshold1);
    setTrackbarPos("Th2",windowName,threshold2);

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

void GroundTruth::renderImgsOnResult()
{
    if(result.cols != windowSize.width || result.rows != windowSize.height)
    {
        result = Mat(windowSize.height, windowSize.width,CV_8UC3);
        cout << "Allocing Img for Result display" << endl;
    }

    Size imSZ(windowSize.width/2,windowSize.height);

    Mat cimg1, cimg2;

    if(showThresholdSegmentation)
    {
        computeThreshold(imgFr1,cimg1);
        computeThreshold(imgFr2,cimg2);
    }else
    {
        cvtColor(imgFr1,cimg1,CV_GRAY2BGR);
        cvtColor(imgFr2,cimg2,CV_GRAY2BGR);
    }

    vector<Gaussian> &gFr1 = frames[currentFrame0].gaussians,
                     &gFr2 = frames[currentFrame1].gaussians;

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
        if(selecFrame == 0 && selecGauss >= 0)
        {
            drawGaussian(cimg1,gFr1[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
        }else if(selecFrame == 1 && selecGauss >= 0)
        {
            drawGaussian(cimg2,gFr2[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
        }
    }

    // Resizing
    resize(cimg1,cimg1,imSZ);
    resize(cimg2,cimg2,imSZ);

    cimg1.copyTo(result(Rect(0         , 0 , imSZ.width , imSZ.height)));
    cimg2.copyTo(result(Rect(imSZ.width, 0 , imSZ.width , imSZ.height)));

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

    // Compute scale and offset betwen two images
    e1x = (float)imSZ.width/ (float)imgFr1.cols, e1y = (float)imSZ.height / (float)imgFr1.rows;
    d1x = 0 , d1y = 0;
    e2x = (float)imSZ.width/ (float)imgFr2.cols, e2y = (float)imSZ.height / (float)imgFr2.rows;
    d2x = imSZ.width, d2y = 0;

    // Draw Matching
    if(showMatch)
    {
        vector< vector< pair<unsigned,unsigned> > >
                &matchs = frames[currentFrame0].match;

        for(unsigned u = 0 ; u < matchs.size(); u++)
        {
            for(unsigned i = 0 ; i < matchs[u].size(); i++)
            {
                if(currentFrame1 == matchs[u][i].first)
                {
                    unsigned v = matchs[u][i].second;

                    line(result,
                         Point2f(d1x + e1x*gFr1[u].x, d1y + e1y*gFr1[u].y),
                         Point2f(d2x + e2x*gFr2[v].x, d2y + e2y*gFr2[v].y),
                         Scalar(0,0,255),2);
                }
            }
        }
    }

    imshow(windowName,result);
}

void GroundTruth::loadCurrentFrame()
{
    imgFr1 = imread(frames[currentFrame0].fileName,CV_LOAD_IMAGE_ANYDEPTH);
    imgFr2 = imread(frames[currentFrame1].fileName,CV_LOAD_IMAGE_ANYDEPTH);

    selecGauss = -1;

    if(frames[currentFrame0].gaussians.size() == 0)
        createGaussian(&frames[currentFrame0],imgFr1);

    if(frames[currentFrame1].gaussians.size() == 0)
        createGaussian(&frames[currentFrame1],imgFr2);
}

void GroundTruth::loadNextFrame()
{
    // Save Current State
    saveGroundTruth("../../GroundTruth/Yatcht_05_12_2014.txt");

    // Load Imgs

    if(selecFrame == 0 && currentFrame0 < currentFrame1-1)
    {
        currentFrame0++;
    }else if(selecFrame == 1 && currentFrame1 < frames.size()-1)
    {
        currentFrame1++;
    }
    else if(selecFrame == -1 && currentFrame1 < frames.size()-1)
    {
        currentFrame1++;
        currentFrame0++;
    }

    loadCurrentFrame();

    // Load Match
}

void GroundTruth::loadPreviewFrame()
{
    // Load Imgs

    if(selecFrame == 0 && currentFrame0 > 0)
    {
        currentFrame0--;
    }else if(selecFrame == 1 && currentFrame1 > currentFrame0 +1)
    {
        currentFrame1--;
    }
    else if(selecFrame == -1 && currentFrame0 > 0)
    {
        currentFrame1--;
        currentFrame0--;
    }

    loadCurrentFrame();
}

void GroundTruth::selectGaussOnFrame(int x, int y, int *frame, int *gauss)
{
    // Try select Gaussian of Frame
    if(x < windowSize.width/2)
    {
        *frame = 0;
        frames[currentFrame0].findGaussian((x-d1x)/e1x,(y-d1y)/e1y, gauss);
    }else
    {
        *frame = 1;
        frames[currentFrame1].findGaussian((x-d2x)/e2x,(y-d2y)/e2y, gauss);
    }
}

void GroundTruth::select(int x, int y)
{
    removeSelection();
    selectGaussOnFrame(x,y,&selecFrame,&selecGauss);
}

void GroundTruth::removeSelection()
{
    selecGauss = selecFrame = -1;
}

void GroundTruth::createGaussian(int frame,int x, int y, Gaussian *g)
{
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
    // Clear old Gaussians
    frame->gaussians.clear();
    frame->match.clear();

    cout << "Creating Gaussians!!" << endl;
    vector<Segment*> segs;
    Gaussian g;

    segmentation.searchThreshold = threshold1;
    segmentation.pixelThreshold = threshold2;

    frame->threshold1 = threshold1;
    frame->threshold2 = threshold2;

    segmentation.segment(img16Bits,&segs);

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
    Mat sample = seg->result(Rect(0,0,seg->N,3)),
        mCov, mMean, eVal, eVect;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(sample, mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE ,
                    CV_32F);

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, true, eVal, eVect);

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
    int mFrame, mGauss;
    selectGaussOnFrame(x,y,&mFrame, &mGauss);

    if(selecFrame == 0 && selecGauss >= 0 && mGauss == -1)
    {
        frames[currentFrame0].removeMatch(selecGauss);
    }
    else if(mFrame > selecFrame &&
            mGauss >=0 && selecGauss >= 0)
    {
        cout << "Match de " << selecGauss << " frame " << currentFrame0 << " com " << mGauss << " frame " << currentFrame1 << endl;

        frames[currentFrame0].match[selecGauss].push_back(
                    pair<unsigned,unsigned>(currentFrame1,mGauss));
        removeSelection();
        renderImgsOnResult();
    }
}

void GroundTruth::mouseEvent(int event, int x, int y)
{
    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            cout << "LEFT DOWN" << "(" << x << " , " << y << ")" << endl;
            select(x,y);
            renderImgsOnResult();
        break;
        case CV_EVENT_LBUTTONUP:
            cout << "LEFT UP" << "(" << x << " , " << y << ")" << endl;
            mouseUP(x,y);
            renderImgsOnResult();
        break;
        case CV_EVENT_RBUTTONDOWN:
            removeSelection();
            cout << "RIGHT " << "(" << x << " , " << y << ")" << endl;
        break;
        case CV_EVENT_RBUTTONUP:
            renderImgsOnResult();
        break;
        case CV_EVENT_MOUSEMOVE:
//            cout << "MOVE " << "(" << x << " , " << y << ")" << endl;
        break;
    }
}

void GroundTruth::tb1(int)
{
    if(threshold1 > threshold2)
    {
        threshold2 = threshold1 + 1;
        setTrackbarPos("Th2",windowName,threshold2);
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
        setTrackbarPos("Th1",windowName,threshold1);
        setTrackbarPos("Th2",windowName,threshold2);
    }

    // Compute new Gaussians with new thresholds
    if(selecFrame == 0)
        createGaussian(&frames[currentFrame0],imgFr1);

    if(selecFrame == 1)
        createGaussian(&frames[currentFrame1],imgFr2);

    if(selecFrame >= 0)
        renderImgsOnResult();
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

void onMouse(int event, int x, int y, int, void *dt)
{
    GroundTruth *gt = (GroundTruth*) dt;
    gt->mouseEvent(event,x,y);
}
