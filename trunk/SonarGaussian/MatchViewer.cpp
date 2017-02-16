#include "MatchViewer.h"
#include "Drawing.h"
// #include "MatchTest/MatchTest.h"

#include "WindowTool/WindowTool.h"
#include "WindowTool/WindowFeatureTest.h"
#include "WindowTool/TopologicalMatch/WFTopologicalMatch.h"
#include "WindowTool/TopologicalVertexMatch/WFTopologicalVertexMatch.h"
#include "WindowTool/WFGroudTruth.h"
#include "WindowTool/GaussianDescriptor/WFGaussianDescriptor.h"
#include "WindowTool/SVM/WFSVM.h"
#include "WindowTool/GTLoop/WFGTLoop.h"
#include "WindowTool/WindowPaint.h"
#include "WindowTool/SingleImageWindow.h"
#include "WindowTool/SingleImageWindow/TestSVMFeature.h"

#include "GraphMatcher/MatchInfo/MatchInfo.h"

#include "Sonar/GaussianTest.h"
#include "CSVMatrixGenerator.h"

#include "CloseLoopTester.h"
#include "CloseLoopAnaliseResult.h"

#include "GraphMatcher/GraphMatcher.h"

#include <set>

typedef pair<unsigned,unsigned> PUU;

bool MatchViewer::loadFileNames(const string &listFileName, vector<string> &fileNames)
{
    FILE *f = fopen(listFileName.c_str(), "r");
    char extractedFileName[500];

    if(f == 0x0)
    {
        printf("Problem when opening file %s.", listFileName.c_str());
        return false;
    }

    while(fscanf(f,"%s", extractedFileName) != -1)
    {
        fileNames.push_back(extractedFileName);
    }
    return true;
}

MatchViewer::MatchViewer()
{
}

MatchViewer::~MatchViewer()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:Destructor:: using destructor!" << endl;
#endif
}

void MatchViewer::standardExecution()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:standardExecution:: use!" << endl;
#endif

    char img_file_name[200];
    FILE *f_img_names = fopen("imgs_1.txt","r");
/// @todo change this sonarConfig for the new one
    ConfigLoader sc;
    sc.load("../SonarGaussian/MachadosConfig");
    Sonar s(sc);

    int frame = 0;
    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        cout << "Frame " << frame++ << endl;
        Mat img = imread(img_file_name,CV_LOAD_IMAGE_ANYDEPTH);
        s.newImage(img);
        waitKey();
    }
}

void MatchViewer::createGroudTruth()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:createGroudTruth:: use!" << endl;
#endif

    GroundTruth gt;
//    if(!gt.loadEmptyFrames("imgs.txt"))

//    if(!gt.loadEmptyFrames("../../../../SonarGraphData/grayData/Frames_shortDataset.txt"))
    if(!gt.loadEmptyFrames("../../../../SonarGraphData/grayData/Frames.txt"))
        return;

//    gt.loadGroundTruth("../../GroundTruth/Yacht_05_12_2014_denso.txt");
//    gt.loadGroundTruth("../../GroundTruth/Yacht_05_12_2014_especifico.txt");
    gt.start();
}

void MatchViewer::compareWithGroundTruth()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:compareWithGroundTruth:: use!" << endl;
#endif
    float compPrecision= 50.f;

//    FILE *fr = fopen("GR_Results.txt", "w");

    // Loading sonar config
    ConfigLoader sc;
    sc.load("../SonarGaussian/Configs.ini");

    // Creating
    Sonar s(sc);
    GraphMatcher gm(sc);
    GroundTruth gt;
    s.storeImgs = false;
    s.drawPixelFound = false;

    // Loading ground truth
    gt.loadGroundTruth("../../GroundTruth/Yacht_05_12_2014_especifico.txt");
    vector<FrameGT> &frs = gt.frames;

    // Loading imgs creating descriptors
    vector<SonarDescritor*> sd(frs.size() , 0x0);
    Mat imgs[frs.size()];

    cout << "Describing frames..."
         << endl;

    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        cout << "Descripting frame "
             << ifr
             << endl;
        imgs[ifr] = imread(frs[ifr].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
        sd[ifr] = s.newImageDirect(imgs[ifr]);
    }

    // Computing results from frame ifr to
    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        FrameGT &cfr = frs[ifr];
        vector< vector<pair<unsigned,unsigned> > > & match = cfr.match;
        set<unsigned> fmatch;

        // Search frames matcheds on ground truth
        for(unsigned i = 0; i < match.size(); i++)
        {
            for(unsigned j = 0 ; j < match[i].size() ; j++)
            {
                unsigned fm = match[i][j].first;
                if(fmatch.find(fm) == fmatch.end())
                {
                    fmatch.insert(fm);
                }
            }
        }

        // Compute diff between ground truth and our algorithm
        set<unsigned>::iterator ifm;
        for(ifm = fmatch.begin() ; ifm != fmatch.end() ; ifm++)
        {
            vector<MatchInfo> vertexMatch;

            // Comparing results variable
            int nMiss=0, nWrong=0, nHit=0;

            // Computing match between ifr and ifm
            gm.findMatch(sd[ifr], sd[*ifm], vertexMatch);

//            cout << "Matchs between frames " << ifr << " and " << *ifm << endl;

            // Creating temporary imgs
            Mat imgL, imgR, result;

            cvtColor(imgs[ifr],imgL,CV_GRAY2BGR);
            cvtColor(imgs[*ifm],imgR,CV_GRAY2BGR);

            Scalar_<float> el, er;

            // Drawing related imgs
            Drawing::drawImgsTogether(Size2i(1300,600),
                                      imgL,imgR,result,
                                      &el, &er);
            // Draw texts
            char tempStr[200];
            sprintf(tempStr,
                    "Frame %d-%d",
                    ifr,*ifm);

            putText(result,tempStr,
                   Point2f(20, 20),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(255,255,255),2);

            // Comparing results of each vertex
            for(unsigned mi = 0 ; mi < vertexMatch.size() ;mi++)
            {
                unsigned au = vertexMatch[mi].uID,
                         av = vertexMatch[mi].vID;

                float xau = sd[ifr]->gaussians[au].x,
                      yau = sd[ifr]->gaussians[au].y,
                      xav = sd[*ifm]->gaussians[av].x,
                      yav = sd[*ifm]->gaussians[av].y;

//                cout << "(" << xau << " , " << yau << ") -> "
//                     << "(" << xav << " , " << yav << ")" << endl;

                float xgu=xau, ygu=yau, xgv, ygv;

                if(gt.findMatch(ifr,&xgu, &ygu, *ifm, &xgv, &ygv, compPrecision))
                {
//                    cout << "(" << xgu << " , " << ygu << ") -> "
//                         << "(" << xgv << " , " << ygv << ")" << endl;
                    if(fabs(xgv-xav) >= compPrecision ||
                       fabs(ygv-yav) >= compPrecision)
                    {
//                        cout << "Match Wrong!" << endl;
                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(xau,yau),
                                               Point2f(xav,yav),
                                               Scalar(0,0,255));
                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(xgu,ygu),
                                               Point2f(xgv,ygv),
                                               Scalar(0,255,255));
                        nWrong++;
                    }else
                    {
//                        cout << "Match Hit! :" << endl;
                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(xau,yau),
                                               Point2f(xav,yav),
                                               Scalar(0,255,0));
                        nHit++;
                    }
                }else
                {
                    Drawing::drawLineMatch(result,el,er,
                                           Point2f(xau,yau),
                                           Point2f(xav,yav),
                                           Scalar(255,0,0));
//                    cout << "Match Miss!" << endl;
                    nMiss++;
                }
            }

            cout << ifr  << " , "
                 << *ifm << " , "
                 << nHit << " , "
                 << nMiss << " , "
                 << nWrong << " , "
                 << gt.nMatchs(ifr, *ifm) << endl << endl;

            imshow("Ground Truth Comparation", result);
            waitKey(0);
        }
    }
}

bool MatchViewer::computeAllMatches()
{
    // Loading All Images File Name
    vector<string> imgName;

    loadFileNames("imgs.txt", imgName);

    Sonar s;

    // Describing all imgs
    vector<SonarDescritor*> imgDescriptor;
    imgDescriptor.reserve(imgName.size());

    for(unsigned i = 0 ; i < imgName.size() ; i++)
    {
        // Loading a image
        Mat sonImg = imread(imgName[i],CV_LOAD_IMAGE_ANYDEPTH);
        imgDescriptor.push_back(s.newImageDirect(sonImg));
        cout << "Described Image " << imgName[i] << endl;
    }

    // Computing Matches
    GraphMatcher gm;
    vector< vector<MatchInfo> > matchs[imgDescriptor.size()];

    unsigned srcImgID, dstImgID, blendWIdnow = 100;
    for(srcImgID = 0 ; srcImgID < imgDescriptor.size() ; srcImgID++)
    {
        matchs[srcImgID].resize(imgDescriptor.size());

        for(dstImgID = srcImgID + blendWIdnow; dstImgID < imgDescriptor.size() ; dstImgID++)
        {
            gm.findMatch(imgDescriptor[srcImgID],
                         imgDescriptor[dstImgID],
                         matchs[srcImgID][dstImgID]);
        }
    }

    return true;
}

void MatchViewer::matchTest()
{
//    MatchTest mt;
//    mt.start();
}

void MatchViewer::gaussianTest()
{
    GaussianTest gt;
    gt.start();

}

void MatchViewer::compareWithGorundTruth2()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:computeAllMatchs:: use!" << endl;
#endif

    FILE *compResultFile = fopen("GT_Comp_Results.csv", "w");
    fprintf(compResultFile,"frameID1,frameID2,nGaussians1,nGaussians2,GTNGaussians1,GTNGaussians2,HitCount,WrongCount,MissCount,NotFoundOnGT,TotalGroundTruthMatch\n");
// frameID1 - First frame ID involved
// frameID2 - Second frame ID involved
// nGaussians1 - Number of Gaussians found in frame1 by our approach
// nGaussians2 - Number of Gaussians found in frame 2 by our approach
// GTNGaussians1 - Number of Gaussians annoted in frame1 as Ground Truth
// GTNGaussians2 - Number of Gaussians annoted in frame2 as Ground Truth
// HitCount - Number equal matchs found in Ground Truth and our approach (True Positive)
// WrongCount - Number of matchs diferent between annotation and our approach (False Positive),
// MissCount - Number of matchs annoted on Ground Truth and not found by our approach (False Positive),
// NotFoundOnGT - Number of matchs found by our approach that was not mapped in ground truth (Unknow),
// TotalGroundTruthMatch - Number of annoted matchs in Ground Truth

    FILE *describeResultFile = fopen("GT_Describe_Results.csv", "w");
    fprintf(describeResultFile,"frameID,GaussiansCount\n");

    // frId1, frId2, hit, miss, wrong

    // Loading sonar config
//    SonarConfig sonarConfig;
//    sonarConfig.load("../SonarGaussian/MachadosConfig");

    // Creating Sonar objects to automatic match
    ConfigLoader config("../SonarGaussian/Configs.ini");
    Sonar sonar(config,false);
    GraphMatcher &graphMatch = sonar.matcher;

    //Creating Ground Truth Object
    GroundTruth groundTruth;

    sonar.storeImgs = false;
    sonar.drawPixelFound = false;

    // Loading ground truth
    groundTruth.loadGroundTruth("../../GroundTruth/Yacht_05_12_2014.txt");
    vector<FrameGT> &GTFrames = groundTruth.frames;

    // Loading imgs and creating image descriptors
    vector<SonarDescritor*> sonarDescriptors(GTFrames.size() , 0x0);
    Mat img;

    // Writing used parameters in result file

    // Segmentation process
    //

    // Graph Match process
    // minSimilarEdgeToMatch
    // SIFTCutValue

    // Computing results for each iFr frame
    for(unsigned iFr = 0 ; iFr < GTFrames.size(); iFr++)
    {
        FrameGT &currentGTFr = GTFrames[iFr];
        vector< vector<pair<unsigned,unsigned> > > & currentGTMatchs = currentGTFr.match;

        set<unsigned> gtFramesIdsInvolved;

        // Search all frames that have annotation of ground truth
        for(unsigned i = 0; i < currentGTMatchs.size(); i++)
        {
            // Looking the Gaussians inside currentFrame, gaussian i match j
            for(unsigned j = 0 ; j < currentGTMatchs[i].size() ; j++)
            {
                unsigned destFrameMatchID = currentGTMatchs[i][j].first;
                if(gtFramesIdsInvolved.find(destFrameMatchID) == gtFramesIdsInvolved.end())
                {
                    gtFramesIdsInvolved.insert(destFrameMatchID);
                }
            }
        }

        // Now, gtFramesIdsInvolved has all frames that
        // have ground truth annotation related to current frame (iFr)

        // Compute diff between GroundTruth and our Automatic Match
        // for all frames that have ground truth annotation related to
        // current frame (iFr)
        set<unsigned>::iterator destFrID;
        for(destFrID = gtFramesIdsInvolved.begin();
            destFrID != gtFramesIdsInvolved.end() ;
             destFrID++)
        {
            // Here will be computed the difference between match
            // of atual frame iFr and related frame destFrID. Always destFrID > iFr

// ================== FRAME DESCRIPTIO ====================
            // Automatic describe frames
            if(sonarDescriptors[iFr] == 0x0)
            {
                cout << "Describing frame " << iFr << endl;
                img = imread(GTFrames[iFr].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
                sonarDescriptors[iFr] = sonar.newImageDirect(img);
                fprintf(describeResultFile,"%u,%u\n",
                        iFr, sonarDescriptors[iFr]->gaussians.size());
            }

            if(sonarDescriptors[*destFrID] == 0x0)
            {
                cout << "Describing frame " << (*destFrID) << endl;
                img = imread(GTFrames[*destFrID].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
                sonarDescriptors[*destFrID] = sonar.newImageDirect(img);
                fprintf(describeResultFile,"%u,%u\n",
                        (*destFrID), sonarDescriptors[(*destFrID)]->gaussians.size());
            }
// ================== END FRAME DESCRIPTIO ===============


// ================ Computing/Loading Matchings ===============

            vector<MatchInfo> automaticVertexMatch; // Has the matchs automatic found by our approach
            // Computing automatic matching between ifr and ifm
            graphMatch.findMatch(sonarDescriptors[iFr], sonarDescriptors[*destFrID], automaticVertexMatch);

            // Ground Truth Informations
            vector< vector<PUU> > &gtMatchs = GTFrames[iFr].match;
            vector<Gaussian> &gtSrcGaussians = GTFrames[iFr].gaussians;
            vector<Gaussian> &gtDstGaussians = GTFrames[*destFrID].gaussians;
            vector<char> gtGaussianEnvolved(GTFrames[iFr].gaussians.size(),0);

// =============== END Computing/Loading Matchings ===============

//            cout << "Matchs between frames " << ifr << " and " << *ifm << endl;

// =============== Drawin frames and their description ===============

            // Creating temporary imgs
            Mat imgL, imgR, result;

            img = imread(GTFrames[iFr].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
            img.convertTo(imgL,CV_8UC1);
            cvtColor(imgL,imgL,CV_GRAY2BGR);

            img = imread(GTFrames[*destFrID].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
            img.convertTo(imgR,CV_8UC1);
            cvtColor(imgR,imgR,CV_GRAY2BGR);

            // Draw Automatic Gaussians (Yellow)
            Drawing::drawGaussians(imgL,sonarDescriptors[iFr]->gaussians,
                                   Scalar(0,255,255),Scalar(0,255,255),
                                   true,false,false);
//            Drawing::drawGaussiansDots(imgL,sonarDescriptors[iFr]->gaussians,
//                                   Scalar(0,255,255),Scalar(0,255,255),
//                                   true,false,true);

            // Draw Automatic Gaussians (Yellow)
            Drawing::drawGaussians(imgR,sonarDescriptors[*destFrID]->gaussians,
                                   Scalar(0,255,255),Scalar(0,255,255),
                                   true,false,false);
//            Drawing::drawGaussiansDots(imgR,sonarDescriptors[*destFrID]->gaussians,
//                                   Scalar(0,255,255),Scalar(0,255,255),
//                                   true,true,true);

            // Draw Ground Truth Gaussians (RED)
            Drawing::drawGaussians(imgL,gtSrcGaussians,
                                   Scalar(0,0,255),Scalar(0,0,255),
                                   true,false,false);
//            Drawing::drawGaussiansDots(imgL,groundTruth.frames[iFr].gaussians,
//                                   Scalar(0,0,255),Scalar(0,0,255),
//                                   true,true,true);

            // Draw Ground Truth Gaussians (RED)
            Drawing::drawGaussians(imgR,gtDstGaussians,
                                   Scalar(0,0,255),Scalar(0,0,255),
                                   true,false,false);
//            Drawing::drawGaussiansDots(imgR,groundTruth.frames[*destFrID].gaussians,
//                                   Scalar(0,0,255),Scalar(0,0,255),
//                                   true,false,true);

//            imshow("Ground Truth Comparation Left", imgL);
//            imshow("Ground Truth Comparation Right", imgR);

            Scalar_<float> el, er;

            // Drawing related imgs
            Drawing::drawImgsTogether(Size2i(1430*2,781),
                                      imgL,imgR,result,
                                      &el, &er);
            // Draw texts
            char tempStr[200];
            sprintf(tempStr,
                    "Frame %d-%d",
                    iFr,*destFrID);

            putText(result,tempStr,
                   Point2f(20, 20),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(255,255,255),2);

// =============== END Drawin frames and their description ===============


// ======== Comparing matchs between Ground Truth and our approach ===============

            // Comparing results variable
            int nHit=0, nWrong=0, nMiss=0, nNotFoundOnGt=0;

            // Comparing results of each vertex
            for(unsigned mi = 0 ; mi < automaticVertexMatch.size() ;mi++)
            {
                unsigned au = automaticVertexMatch[mi].uID,
                         av = automaticVertexMatch[mi].vID;

                // Automatic match positions
                Gaussian &aGu = sonarDescriptors[iFr]->gaussians[au],
                         &aGv = sonarDescriptors[*destFrID]->gaussians[av];

                int gtGuId=-1, gtGvID=-1;

//                cout << "(" << xau << " , " << yau << ") -> "
//                     << "(" << xav << " , " << yav << ")" << endl;

                if(groundTruth.findMatchByIntersectGaussian(iFr,aGu,*destFrID,&gtGuId,&gtGvID)) // Find source edges
                {
                    // Source edge was found!
                    Gaussian *gtGu=&gtSrcGaussians[gtGuId], *gtGv=&gtDstGaussians[gtGvID];
                    gtGaussianEnvolved[gtGuId] = 1;

//                    cout << "(" << xgu << " , " << ygu << ") -> "
//                         << "(" << xgv << " , " << ygv << ")" << endl;
                    if(!Gaussian::hasIntersection(*gtGv,aGv))
                    {
                        cout << "Match Wrong!" << endl;

                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(aGu.x,aGu.y),
                                               Point2f(aGv.x,aGv.y),
                                               Scalar(0,0,255),2); // red  (wrong match)

                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(gtGu->x,gtGu->y),
                                               Point2f(gtGv->x,gtGv->y),
                                               Scalar(0,255, 255),2); // yellow (GT corrected match)
                        nWrong++;
                    }else
                    {
                        cout << "Match Hit! :" << endl;
                        Drawing::drawLineMatch(result,el,er,
                                               Point2f(aGu.x,aGu.y),
                                               Point2f(aGv.x,aGv.y),
                                               Scalar(0,255,0),2); // green (Correct match)

                        nHit++;
                    }
                }else
                {   // Edges not found on Ground Truth
                    Drawing::drawLineMatch(result,el,er,
                                           Point2f(aGu.x,aGu.y),
                                           Point2f(aGv.x,aGv.y),
//                                           Scalar(0,165,255),2); // orange (Ground Truth Misss match )
                                           Scalar(255,0,255),2); // magenta (Ground Truth Misss match )
                    cout << "Match Miss!" << endl;
                    nNotFoundOnGt++;
                }
            }

            unsigned gtNMatchs =0;

            // For each gaussian in frame iFr of the Ground Truth
            for(unsigned u = 0 ; u < gtMatchs.size();u++)
            {
                // For each match that this gaussian has (It can be related to any other frame)
                for(unsigned i = 0 ; i < gtMatchs[u].size(); i++)
                {
                    // Verify if the annotaion is related the envolved frame
                    if(gtMatchs[u][i].first == *destFrID)
                    {
                        gtNMatchs++;

                        if(!gtGaussianEnvolved[u])
                        {
                            Gaussian &gtGu = gtSrcGaussians[u],
                                     &gtGv = gtDstGaussians[gtMatchs[u][i].second];

                            Drawing::drawLineMatch(result,el,er,
                                                   Point2f(gtGu.x,gtGu.y),
                                                   Point2f(gtGv.x,gtGv.y),
                                                   Scalar(255,0,0),2); // blue ( Atomatic Solution Miss Match )
                            nMiss++;
                        }
                    }
                }
            }

            fprintf(compResultFile,"%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u\n",
                    iFr, *destFrID,
                    sonarDescriptors[iFr]->gaussians.size(),sonarDescriptors[*destFrID]->gaussians.size(),
                    GTFrames[iFr].gaussians.size(), GTFrames[*destFrID].gaussians.size(),
                    nHit,nWrong,nMiss,nNotFoundOnGt, gtNMatchs);

//            imshow("Ground Truth Comparation", result);

            sprintf(tempStr,
                    "result_%04d_%04d.png",
                    iFr,*destFrID);

            imwrite(tempStr,result);

//            waitKey(0);
        }
    }

    fclose(compResultFile);
    fclose(describeResultFile);
}

void MatchViewer::testWindowTool()
{
//    WindowFeatureTest *wf = new WindowFeatureTest;
//    WFGroudTruth *wf = new WFGroudTruth;
//    WFTopologicalMatch *wf = new WFTopologicalMatch;
    WFTopologicalVertexMatch *wf = new WFTopologicalVertexMatch;
    WindowTool wt("Sonar", wf);

    wt.loadEmptyFrames("../../../../SonarGraphData/grayData/Frames.txt");
    wt.start();
}

void MatchViewer::generateMatlabMatrix(const char *groundTruthFileName)
{
    CSVMatrixGenerator csvGen;

    csvGen.generate(groundTruthFileName);
}

void MatchViewer::closeLoopDetection(const string &datasetPath, unsigned start,unsigned end)
{
    cout << "Using dataset path " << datasetPath << endl
         << "Starting loop detection from " << start << " to " << endl;

    CloseLoopTester clt(datasetPath,1);
//    clt.loadFrames("Frames_shortDataset.txt");
    clt.loadFrames("Frames.txt");
    clt.describeFrames();
    clt.computeMatchs(0,start,end);
}

void MatchViewer::closeLoopAnaliseResult()
{
    CloseLoopAnaliseResult clar("../../../../SonarGraphData/grayData/");
//    CloseLoopAnaliseResult clar("/media/matheusbg8/Seagate Expansion Drive/DATASETs/Yacht_05_12_2014_last/grayShort/");

    clar.loadAnalisyAndSave();
//    clar.splitGTResults();
}

void MatchViewer::testGaussianDescriptionFeature()
{
//    WindowFeatureTest *wf = new WindowFeatureTest;
//    WFGroudTruth *wf = new WFGroudTruth;
    ConfigLoader config("../SonarGaussian/Configs.ini");

    WFGaussianDescriptor *wf = new WFGaussianDescriptor(config);
    WindowTool wt("Sonar", wf);

    WFSVM *wfsvm = new WFSVM; // Need ui control and gaussians control
    wf->setGDF(wfsvm); // gaussian control
    wt.addFeature(wfsvm); // ui control

    // Load a dataset
//    char workFile[] = "../../GroundTruth/SVM/Paper_LARS2016";
    char workFile[] = "../../GroundTruth/SVM/Paper_JINT2017_d10";
    wfsvm->setWorkFile(workFile);

    if(!wfsvm->autoLoad())
    {
        // Load empty frames to star a new ground truth dataset
        wt.loadEmptyFrames("../../Datasets/yacht_05_12_img16bits_especifico.txt");
    }

    wt.start();

    delete wf;
    delete wfsvm;

}

void MatchViewer::testPaintWindow()
{
    WindowPaint wp;

    wp.start();
}

void MatchViewer::testSVM()
{
    SingleImageWindow siw;
    TestSVMFeature *tsvmf = new TestSVMFeature;
    siw.addFeature(tsvmf);
    siw.start();

    delete tsvmf;
}

void MatchViewer::gtLoopTester()
{
    WFGTLoop *wf = new WFGTLoop;
    WindowTool wt("GT Loop", wf);

    wt.start();

    delete wf;
}

void MatchViewer::gtTopologicalMatch()
{
    WFTopologicalVertexMatch *wf = new WFTopologicalVertexMatch;
    WFGTLoop *wf2 = new WFGTLoop;
    WindowTool wt("Sonar", wf2);
    wt.addFeature(wf);

//    wt.loadEmptyFrames("../../../../SonarGraphData/grayData/Frames.txt");
    wt.start();

}
