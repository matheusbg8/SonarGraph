#include "WFGroudTruth.h"
#include "WindowTool/WindowTool.h"
#include "Drawing/Drawing.h"

void WFGroudTruth::switchShowGaussians()
{
    showGaussians= !showGaussians;
}

void WFGroudTruth::switchShowMatch()
{
    showMatch= !showMatch;
}

void WFGroudTruth::switchShowRegionsOfInterese()
{
    showRegionsOfInterest = !showRegionsOfInterest;
}

void WFGroudTruth::removeSelection()
{
    wt->removeSelection();
    selecGauss = -1;
}

FrameGT *WFGroudTruth::frame(unsigned id)
{
    // Yes, we are breaking some SOLID principles of OO Programming
    // if you know a better way to solve this, please send me a e-mail
    // matheusbg8@gmail.com, thank you! Maybe a vector here? But its performanece?

    // Slow and safe version
    FrameGT *fgt = dynamic_cast<FrameGT*>(wt->frames[id]);
    if(fgt == 0x0)
        cout << "Polymorphism problem!!" << endl;

    return fgt;
    // Fast and unsafe version
    return (FrameGT*) wt->frames[id];

}

WFGroudTruth::WFGroudTruth():
    threshold1(171),
    threshold2(215),
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
    selecGauss(-1),
    config("../SonarGaussian/Configs.ini"),
    segmentation(config),
    matchHandler(config)
{

}

void WFGroudTruth::createGaussian(FrameGT *frame, Mat &img16Bits)
{
    // Just select gaussians becouse user is change treshold tackbar
    // the selected frame need to stay selected.
    selecGauss = -1;

    // Clear old Gaussians
    frame->gaussians.clear();
    frame->match.clear();

    vector<Segment*> segs;

    // Peaks Threshold segmentation
    segmentation.searchThreshold = threshold1;
    segmentation.pixelThreshold = threshold2;
//    segmentation.segmentLinear(img16Bits,&segs);
    segmentation.segment(img16Bits,&segs);

    // Peaks Segmentation
//    segmentation.maxPeakSize = 200;
//    segmentation.peakThreshold = 0;
//    segmentation.segmentWithPeaks(img16Bits,&segs);

    // Theta Rho Segmentation (without threshold)
//    segmentation.segmentWithTR(img16Bits, &segs);

    frame->threshold1 = threshold1;
    frame->threshold2 = threshold2;

    Mat bgrImg;
    img16Bits.convertTo(bgrImg,CV_8UC1);
    cvtColor(bgrImg,bgrImg,CV_GRAY2BGR);
    for(unsigned i = 0 ; i < segs.size(); i++)
    {
        frame->gaussians.push_back(Gaussian(segs[i],3.f));
        segs[i]->drawSegment(bgrImg,Scalar(0,0,255));
    }
//    imshow("Test", bgrImg);
//    waitKey();

    frame->match.resize(frame->gaussians.size());
}

void WFGroudTruth::saveGroundTruth(const char *fileName)
{
    unsigned frameSize = wt->frames.size();

    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frameID=0, g, u, i, nMatch;

    fprintf(f,"%u\n", frameSize);
    for(frameID = 0 ; frameID < frameSize ; frameID++)
    {
        FrameGT &fr = *frame(frameID);

        // Frame description
        // Image filename frameID NumberOfGaussians threshold1 and threhold2
        fprintf(f,"%s %u %u %u %u\n", fr.fileName.c_str() , frameID, fr.gaussians.size(), fr.threshold1, fr.threshold2);

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        if(gs.size() == 0) continue;

        for(g = 0 ; g < gs.size(); g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fprintf(f,"%d %f %f %f %f %f %f %f %u\n", g, ga.x, ga.y , ga.intensity, ga.dx , ga.dy, ga.di, ga.ang , ga.N );
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

void WFGroudTruth::loadGroundTruth(const char *fileName)
{
    vector<Frame*> &frames = wt->frames;

    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frameID, g,frameSize,gaussiansSize, ug, vFr, vG, nMatch,i;
    char imgFileName[300];

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameSize);
    frames.resize(frameSize,0x0);
    for(frameID = 0 ; frameID < frames.size() ; frameID++)
    {
        FrameGT &fr = *frame(frameID);

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
                    &ga.x, &ga.y , &ga.intensity,
                    &ga.dx , &ga.dy, &ga.di,
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

void WFGroudTruth::makeCurrentMosaic()
{
    Mat img8Bits_left, img8Bits_right;

    wt->leftImg.convertTo(img8Bits_left,CV_8UC1);
    wt->righImg.convertTo(img8Bits_right,CV_8UC1);

    matchHandler.createMosaicImage(img8Bits_left,img8Bits_right);
}

void WFGroudTruth::acceptHomographyMatchs()
{
    vector<unsigned> gSrcList, gDestList;
    matchHandler.HMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    FrameGT &currentFrame = *frame(wt->currentLeftFrame);

    // Remove all matchs between frame0 (left displayed) to frame1(right dysplayed)
    currentFrame.removeMatchToFrame(wt->currentRightFrame);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], wt->currentRightFrame ,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    wt->loadCurrentFrame();
}

void WFGroudTruth::acceptOpticalFlowMatchs()
{
    vector<unsigned> gSrcList, gDestList;
    matchHandler.OFMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    FrameGT &currentFrame = *frame(wt->currentLeftFrame);

    // Remove all matchs between frame0 (right displayed) to frame1(left dysplayed)
    currentFrame.removeMatchToFrame(wt->currentRightFrame);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], wt->currentRightFrame,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    wt->loadCurrentFrame();
}

void WFGroudTruth::acceptContinuumMatchs()
{
    vector<unsigned> gSrcList, gDestList;
    matchHandler.CFMatchIDList(&gSrcList, &gDestList);

    if(gSrcList.size() == 0) // Stop if we don't have homography matchs
        return;

    FrameGT &currentFrame = *frame(wt->currentLeftFrame);

    // Remove all matchs between frame0 (right displayed) to frame1(left dysplayed)
    currentFrame.removeMatchToFrame(wt->currentRightFrame);

    // Do all homography matchs
    for(unsigned i = 0; i < gSrcList.size();i++)
    {
        // Do i-est homography match
        currentFrame.newMatch(gSrcList[i], wt->currentRightFrame,gDestList[i]);
    }

    // Re-load current frame on screen and update matchHander
    wt->loadCurrentFrame();
}


Frame *WFGroudTruth::newFrame(const string &fileName, unsigned frameNumber)
{
    return new FrameGT(fileName,frameNumber);
}

void WFGroudTruth::keyPress(char c)
{
    switch(c) // Preview frame
    {
        case 'v': // Change visualization
            showThresholdSegmentation= !showThresholdSegmentation;
        break;
        case 'e': // Show Matchings
            switchShowMatch();
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
        case 13: // Key enter, do all homography match
        case 10: // Key enter, do all homography match

            acceptHomographyMatchs();
        break;
        case '\\':
            // Show mouse homographic projection (just update display)
        break;
        case 'o': // Compute optical flow matchs
            matchHandler.matchWithOpticalFlow(*frame(wt->currentLeftFrame), *frame(wt->currentRightFrame),
                                              wt->leftImg,wt->righImg);
        break;
        case 'i': // Show/Hide optical flow matchs
            showOpticalFlowMatchs = !showOpticalFlowMatchs;
        break;
        case 'p': // Accept optical flow matchs
            acceptOpticalFlowMatchs();
        break;
        case 't': // Compute continuum matchs
//            matchHandler.consecutiveMatch(wt->frames,wt->currentLeftFrame,
//                                          wt->currentRighFrame);
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
        case '3':
            switchShowRegionsOfInterese();
    //            segmentation.createPeakImg(imgFr1);
        break;
    }
}

void WFGroudTruth::mouseEvent(int event, int x, int y)
{

}

void WFGroudTruth::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
//    segmentation.createPeakImg(imgFr1);
//    segmentation.createGradientImg(imgFr1);
//    segmentation.createLogImg(imgFr1);
//    segmentation.createImg8bitsTruncated(imgFr1);
//    segmentation.createRTImage(imgFr1);
//    segmentation.createRTPlot(imgFr1);

    vector<Gaussian> &gsFrLeft = frame(leftId)->gaussians,
                     &gsFrRight = frame(rightId)->gaussians;

    // Drawing Gaussians
    if(showGaussians)
    {
        for(unsigned i = 0 ; i < gsFrLeft.size(); i++)
        {
            Drawing::drawGaussian(leftImg,gsFrLeft[i],Scalar(0,0,255),Scalar(0,255,0),i);
        }

        for(unsigned i = 0 ; i < gsFrRight.size(); i++)
        {
            Drawing::drawGaussian(rightImg,gsFrRight[i],Scalar(0,0,255),Scalar(0,255,0),i);
        }
    }

    // Draw selected gaussian
    if(wt->selecFrame == 0 && selecGauss >= 0)
    {
        Drawing::drawGaussian(leftImg,gsFrLeft[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
    }else if(wt->selecFrame == 1 && selecGauss >= 0)
    {
        Drawing::drawGaussian(rightImg,gsFrRight[selecGauss],Scalar(0,255,0),Scalar(0,0,255),selecGauss);
    }

//    if(showMatch && showSpatialMatchs)
//    {
//        matchHandler.spatialMatch(frames[currentFrame0],frames[currentFrame1],
//                                  250.f,
//                                  leftImg,rightImg);
//    }

//    if(showIndividualFrameLeft)
//    {
//        imshow("Individual Frame Left", leftImg);
//    }

//    if(showIndividualFrameRight)
//    {
//        imshow("Individual Frame Right", rightImg);
//    }
}

void WFGroudTruth::renderFrameTogether(Mat &screen,
                                       const Scalar_<float> &el, unsigned leftFrameId,
                                       const Scalar_<float> &er, unsigned rightFrameId)
{
//    // Draw texts
//    char tempStr[200];

//    sprintf(tempStr,
//            "Frame %d , Th1 %u Th2 %u",
//            currentLeftFrame,frames[currentLeftFrame].threshold1,
//            frames[currentLeftFrame].threshold2);

//    putText(result,tempStr,
//           Point2f(10, result.rows-20),
//           FONT_HERSHEY_COMPLEX,0.5,
//            Scalar(255,255,255),2);


//    sprintf(tempStr,
//            "Frame %d , Th1 %u Th2 %u",
//            currentRighFrame,frames[currentRighFrame].threshold1,
//            frames[currentRighFrame].threshold2);

//    putText(result,tempStr,
//           Point2f(result.cols/2.f +10.f,result.cols-20.f),
//           FONT_HERSHEY_COMPLEX,0.5,
//            Scalar(255,255,255),2);

//    colorName(tempStr);
//    putText(result,tempStr,
//           Point2f(result.cols/2.f-100.f,20.f),
//           FONT_HERSHEY_COMPLEX,0.5,
//            Scalar(255,255,255),2);

//    // Drawing optical flow matchs
//    if(showMatch && showOpticalFlowMatchs)
//    {
//        Drawing::drawLineMatch(result,el,er,
//                               matchHandler.OFMatchSrc,matchHandler.OFMatchDest,
//                               Scalar(0,255,255),2);
//    }

//    // Draw Spatial Matchs
//    if(showMatch && showSpatialMatchs)
//    {
//        Drawing::drawLineMatch(result,el,er,
//                               matchHandler.SPMatchSrc,matchHandler.SPMatchDest,
//                               Scalar(255,255,0),1);
//    }

//    // Drawing consecutive matchs
//    if(showMatch && showContinuumFeatureMatchs)
//    {
//        Drawing::drawLineMatch(result,el,er,
//                               matchHandler.CMatchSrc,matchHandler.CMatchDest,
//                               Scalar(0,255,0),3);
//    }

//    // Draw Homographic matchs
//    if(showMatch && showHomographyMatchs && matchHandler.hasHomography())
//    {
//        Drawing::drawLineMatch(result,el,er,
//                               matchHandler.HMatchSrc,matchHandler.HMatchDest,
//                               Scalar(255,0,0),3);
//    }

//    if(showHomographyMouse && mousePosition.x < result.cols/2 && matchHandler.hasHomography())
//    {
//        Point2f tMouse;
//        matchHandler.transform(
//                    Point2f((mousePosition.x-el.val[2])/el.val[0], (mousePosition.y-el.val[3])/el.val[1]),
//                    &tMouse);

//        circle(result,
//               Point2f(er.val[2] + er.val[0]*tMouse.x, er.val[3] + er.val[1]*tMouse.y),
//               5, Scalar(0,255,0));
//    }

//    // Draw Matching
//    if(showMatch)
//    {
//        Drawing::drawLineMatch(result,el,er,
//                               matchHandler.GTMatchSrc,matchHandler.GTMatchDest,
//                               Scalar(0,0,255),2);
//    }

//    // Drawing selected Matching
//    if(selecFrame == 0 && selecGauss >= 0)
//    {
//        Point2f src, dest;
//        // Homography matching
//        if(matchHandler.HMatchSrc.size() >= 4
//           && matchHandler.findHMatch(selecGauss,&src, &dest))
//        {
//            Drawing::drawLineMatch(result,el,er,
//                                   src,dest,
//                                   Scalar(255,0,0),3);
//        }
//        // Ground Truth Matching
//        if(matchHandler.findGTMatch(selecGauss,&src, &dest))
//        {
//            Drawing::drawLineMatch(result,el,er,
//                                   src,dest,
//                                   Scalar(0,0,255),2);
//        }
//    }
}

void WFGroudTruth::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    FrameGT *fr = frame(rightId),
          *lr = frame(leftId);

    if(fr->gaussians.size() == 0)
        createGaussian(fr,rightImg);

    if(lr->gaussians.size() == 0)
        createGaussian(lr,leftImg);

    matchHandler.loadMatchs(*fr,*lr,
                            leftImg, rightImg,10.f);
}
