#include "MatchViewer.h"
#include "Drawing.h"
// #include "MatchTest/MatchTest.h"

#include "GaussianTest.h"
#include <set>

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

    SonarConfig sc;
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
    if(!gt.loadEmptyFrames("imgs.txt"))
        return;

    //    gt.loadGroundTruth("../../GroundTruth/Yacht_05_12_2014_denso.txt");
//    gt.loadGroundTruth("../../GroundTruth/Yatcht_05_12_2014_especifico.txt");


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
    SonarConfig sc;
    sc.load("../SonarGaussian/MachadosConfig");

    // Creating
    Sonar s(sc);
    GraphMatcher gm(sc);
    GroundTruth gt;
    s.storeImgs = false;
    s.drawPixelFound = false;

    // Loading ground truth
    gt.loadGroundTruth("../../GroundTruth/Yatcht_05_12_2014.txt");
    vector<Frame> &frs = gt.frames;

    // Loading imgs creating descriptors
    vector<SonarDescritor*> sd(frs.size() , 0x0);
    Mat imgs[frs.size()];

    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        imgs[ifr] = imread(frs[ifr].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
        sd[ifr] = s.newImageDirect(imgs[ifr]);
    }

    // Computing results
    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        Frame &cfr = frs[ifr];
        vector< vector<pair<unsigned,unsigned> > > & match = cfr.match;
        set<unsigned> fmatch;

        // Search frames matcheds
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
            vector<pair<unsigned,unsigned> > vertexMatch;

            // Comparing results variable
            int nMiss=0, nWrong=0, nHit=0;

            // Computing match between ifr and ifm
            gm.matcheSIFTCut(sd[ifr], sd[*ifm], vertexMatch);

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
                unsigned au = vertexMatch[mi].first,
                         av = vertexMatch[mi].second;

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
            waitKey(100);
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
    vector< vector<pair<unsigned,unsigned> > > matchs[imgDescriptor.size()];

    unsigned srcImgID, dstImgID, blendWIdnow = 100;
    for(srcImgID = 0 ; srcImgID < imgDescriptor.size() ; srcImgID++)
    {
        matchs[srcImgID].resize(imgDescriptor.size());

        for(dstImgID = srcImgID + blendWIdnow; dstImgID < imgDescriptor.size() ; dstImgID++)
        {
            gm.matcheSIFTCut(imgDescriptor[srcImgID],
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

void MatchViewer::computeAllMatchs()
{
#ifdef MATCHVIEWER_TRACKING_DEBUG
    cout << "MatchViewer:computeAllMatchs:: use!" << endl;
#endif
    float compPrecision= 50.f;

//    FILE *fr = fopen("GR_Results.txt", "w");

    // Loading sonar config
    SonarConfig sc;
    sc.load("../SonarGaussian/MachadosConfig");

    // Creating
    Sonar s(sc);
    GraphMatcher gm(sc);
    GroundTruth gt;
    s.storeImgs = false;
    s.drawPixelFound = false;

    // Loading ground truth
    gt.loadGroundTruth("../../GroundTruth/Yatcht_05_12_2014.txt");
    vector<Frame> &frs = gt.frames;

    // Loading imgs creating descriptors
    vector<SonarDescritor*> sd(frs.size() , 0x0);
    Mat imgs[frs.size()];

    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        imgs[ifr] = imread(frs[ifr].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
        sd[ifr] = s.newImageDirect(imgs[ifr]);
    }

    // Computing results
    for(unsigned ifr = 0 ; ifr < frs.size(); ifr++)
    {
        Frame &cfr = frs[ifr];
        vector< vector<pair<unsigned,unsigned> > > & match = cfr.match;
        set<unsigned> fmatch;

        // Search frames matcheds
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
            vector<pair<unsigned,unsigned> > vertexMatch;

            // Comparing results variable
            int nMiss=0, nWrong=0, nHit=0;

            // Computing match between ifr and ifm
            gm.matcheSIFTCut(sd[ifr], sd[*ifm], vertexMatch);

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
                unsigned au = vertexMatch[mi].first,
                         av = vertexMatch[mi].second;

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
            waitKey(100);
        }
    }
}
