#include "Sonar.h"
#include "SonarDescritor.h"
#include <iostream>
#include <cstdio>
#include "SonarConfig.h"

using namespace std;

typedef pair<unsigned, unsigned> pii;

void Sonar::createGaussian(Mat &img, SonarDescritor *sd)
{
    /* Clear old Gaussians */
    sd->gaussians.clear();

    /* Do some image transform */
//    if(img.type() != CV_8UC3)
//    {
//        cvtColor(img, colorImg, CV_GRAY2BGR);
//    }
//    else
//    {
//        img.copyTo(colorImg);
//        cvtColor(img,img,CV_BGR2GRAY);
//    }

    float rowMean,colMean,pixelMean,ang,
          rowStdDev,colStdDev, pixelStdDev;

    Mat mCov, mMean, eVal, eVect;

    vector<Segment*> seg;

    segmentation.segment(img,&seg);

    for(unsigned i = 0 ; i < seg.size() ; i++)
    {
        Mat sample = seg[i]->result(Rect(0,0,seg[i]->N,3));

        // Calculate Covariance Matrix of the sample found
        calcCovarMatrix(sample, mCov, mMean,
                        CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE ,
                        CV_32F);

        // Calculate EigenValues and EigenVector of Covariance Matrix
        eigen(mCov, eVal, eVect);

        rowMean = mMean.at<float>(0,0);
        colMean = mMean.at<float>(1,0);
        pixelMean = mMean.at<float>(2,0);

        rowStdDev = sqrt(eVal.at<float>(1,0));
        colStdDev = sqrt(eVal.at<float>(2,0));
        pixelStdDev = sqrt(eVal.at<float>(0,0));

        ang = 180.0*atan2(eVect.at<float>(2,0),eVect.at<float>(2,1))/M_PI;

        if(ang<0.f) ang+= 360.f;
        if(ang>180.f)ang-=180.f;

/*
        cout << "Sample:" << endl << sample << endl;
        cout << "Covariance:" << endl << mCov << endl;
        cout << "Mean:" << endl << mMean << endl;
        cout << "Eigen Values:" << endl << eVal << endl;
        cout << "Eigen Vectors:" << endl << eVect << endl;
        cout << "Ang:" << endl << ang << endl;
        cout << "N:" << endl << seg[i]->N << endl;
*/
        if(drawPixelFound)
        {
            seg[i]->drawSegment(colorImg,Scalar(0,0,255));
//            seg[i]->drawSegmentBox(colorImg,Scalar(0,255,255));

//            seg[i]->rotateSegment(-ang,colMean, rowMean);
//            seg[i]->drawSegment(colorImg,Scalar(255,255,0));
        }

/*                cout << "rowStdDev: " << rowStdDev << endl
             << "coltdDev: " << colStdDev << endl
             << "pixelStdDev: " << pixelStdDev << endl
             << "ang: " << ang << endl;
*/
        sd->gaussians.push_back(Gaussian(colMean,rowMean,pixelMean,
                                         colStdDev*stdDevMultiply,rowStdDev*stdDevMultiply,pixelStdDev,ang,seg[i]->N));
    }

    //pseudoMergeGaussian(0,1);
    //mergeGaussian(0,1);
}

void Sonar::drawGaussians(Mat colorImg, SonarDescritor *sd)
{
    float ri = 0x00, rf = 0xff, dr = rf-ri, r,
          gi = 0x00, gf = 0x00, dg = gf-gi, g,
          bi = 0xff, bf = 0x00, db = bf-bi, b,
          mt=segmentation.searchThreshold, Mt=0.f, dt, t;

    // Search bigest pixel instensity
    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        if(Mt < sd->gaussians[i].z)
            Mt = sd->gaussians[i].z;
    }

//    Mt = 120;
//    cout << "Mt " << Mt << endl;
    dt = Mt-mt;

    for(unsigned i = 0; i < sd->gaussians.size(); i++)
    {
        t = (sd->gaussians[i].z - mt)/dt;
        r = dr*t + ri;
        g = dg*t + gi;
        b = db*t + bi;
//        cout << "color " << t << " ( " << r << " , " << g << " , " << b << ")" << endl;
        char tempStr[50];

        for(unsigned j =0; j < sd->graph[i].size() ; j++)
        {
            GraphLink *link = sd->graph[i][j];

            if(drawEdges)
            {
                line(colorImg,Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[link->dest].x, sd->gaussians[link->dest].y),
                     Scalar(b,g,r),2);
            }

            if(drawEdgesAngle)
            {
                sprintf(tempStr,"%.2f", link->ang);
                putText(colorImg, tempStr,
                        Point2f(sd->gaussians[i].x + 0.25*link->p*sin(link->ang*M_PI/180.f),
                                sd->gaussians[i].y - 0.25*link->p*cos(link->ang*M_PI/180.f)),
                        FONT_HERSHEY_COMPLEX, 0.4,
                        Scalar(b, g, r), 1, 8);
            }

            if(drawEdgesInvAngle)
            {
                line(colorImg,
                     Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                     Point2f(sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f),
                             sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f)),
                     Scalar(0,0,255),2);

                sprintf(tempStr,"%.2f", link->invAngle);
                int baseLine = 0;
                Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.5,1,&baseLine);

                putText(colorImg, tempStr,
                        Point2f( sd->gaussians[i].x + 0.25*link->p*sin((link->ang-link->invAngle/2.f)*M_PI/180.f) - (textSize.width / 2.f),
                                 sd->gaussians[i].y + 0.25*-link->p*cos((link->ang-link->invAngle/2.f)*M_PI/180.f) + (textSize.height /2.f) ),
                        FONT_HERSHEY_COMPLEX, 0.5,
                        Scalar(0, 255, 0), 1, 8);
            }
        }

        if(drawElipses)
        {
            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    sd->gaussians[i].ang,//180.0*atan2(sd->gaussians[i].dy,sd->gaussians[i].dx)/M_PI,
                    0.0 , 360.0,
                    Scalar(b,g,r),
                    2 , 8 , 0);
        }

        if(drawVertexID)
        {
            sprintf(tempStr, "ID %d", i);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+20, sd->gaussians[i].y+20),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(0,0,255),2);
        }

        if(drawElipsesAngle)
        {
            sprintf(tempStr, "%.2f", sd->gaussians[i].ang);
            putText(colorImg,tempStr,
                   Point2f(sd->gaussians[i].x+40, sd->gaussians[i].y),
                   FONT_HERSHEY_COMPLEX,0.5,
                    Scalar(b,g,r),2);

            line(colorImg,
                 Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                 Point2f(sd->gaussians[i].x + 50.f*sin(sd->gaussians[i].ang*M_PI/180.f),
                         sd->gaussians[i].y - 50.f*cos(sd->gaussians[i].ang*M_PI/180.f)),
                 Scalar(0,255,0),2);

        }

    }
}

bool Sonar::intersec(Gaussian &a, Gaussian &b)
{
   /*
    float dx = a.x - b.x,
          dy = a.y - b.y,
          dist = sqrt(dx*dx + dy*dy);

    if( dist > a.dx + )
    {

    }
    */
}

void Sonar::mergeGaussian(SonarDescritor *sd, unsigned a, unsigned b)
{
    Gaussian &ga = sd->gaussians[a],
             &gb = sd->gaussians[b];

    unsigned N_ab = ga.N + gb.N;
    float xMean_ab = (ga.x * ga.N + gb.x * gb.N)/ ( N_ab ),
          yMean_ab = (ga.y * ga.N + gb.y * gb.y)/ ( N_ab ),
          zMean_ab = (ga.z * ga.N + gb.z * gb.N)/ ( N_ab ),
          xMeanDiff = ga.x-gb.x , yMeanDiff = ga.y - gb.y, zMeanDiff = ga.z - gb.z,
          xStdDev_ab = sqrt(
              ( ( ga.N * (ga.dx*ga.dx) + gb.N * (gb.dx * gb.dx) ) / N_ab  ) +
              ((ga.N * gb.N)/(N_ab*N_ab)) * (xMeanDiff*xMeanDiff) ),
          yStdDev_ab = sqrt(
              ( ( ga.N * (ga.dy*ga.dy) + gb.N * (gb.dy * gb.dy) ) / N_ab  ) +
              ((ga.N * gb.N)/(N_ab*N_ab)) * (yMeanDiff*yMeanDiff) ),
          zStdDev_ab = sqrt(
              ( ( ga.N * (ga.dz*ga.dz) + gb.N * (gb.dz * gb.dz) ) / N_ab  ) +
              ((ga.N * gb.N)/(N_ab*N_ab)) * (zMeanDiff*zMeanDiff) ),
          ang;

    float Mx, MXy , My , MYx;

    if(ga.x > gb.x)
    {
        Mx = ga.x;     MXy = ga.y;
    }else
    {
        Mx = gb.x;     MXy = gb.y;
    }
    if(ga.y > gb.y)
    {
        My = ga.y;     MYx = ga.x;
    }else
    {
        My = gb.y;     MYx = gb.x;
    }

//    ang = calcGaussianAng(xMean_ab, yMean_ab,xStdDev_ab, yStdDev_ab,
//                          Mx,MXy,My, MYx);

    sd->gaussians.push_back(Gaussian(xMean_ab, yMean_ab, zMean_ab,
                                 xStdDev_ab, yStdDev_ab, zStdDev_ab,
                                 ang, N_ab));
}

void Sonar::pseudoMergeGaussian(SonarDescritor *sd, unsigned a, unsigned b)
{
    Gaussian &ga = sd->gaussians[a],
             &gb = sd->gaussians[b];

    unsigned N_ab = ga.N + gb.N;
    float xMean_ab ,yMean_ab,zMean_ab,
          xMeanDiff = ga.x-gb.x , yMeanDiff = ga.y - gb.y, zMeanDiff = ga.z - gb.z,
          xStdDev_ab,yStdDev_ab,zStdDev_ab;

    xMean_ab = (ga.x + gb.x)/2.f;

    if(ga.dx > gb.dx)
    {
        if(ga.x > gb.x)
            xMean_ab += (ga.dx-gb.dx)/2.f;
        else xMean_ab -= (ga.dx-gb.dx)/2.f;
    }
    else
    {
        if(gb.x > ga.x)
            xMean_ab += (gb.dx-ga.dx)/2.f;
        else xMean_ab -= (gb.dx-ga.dx)/2.f;
    }

    yMean_ab = (ga.y + gb.y)/2.f;

    if(ga.dy > gb.dy)
    {
        if(ga.y > gb.y)
            yMean_ab += (ga.dy-gb.dy)/2.f;
        else yMean_ab -= (ga.dy-gb.dy)/2.f;
    }
    else
    {
        if(gb.y > ga.y)
            yMean_ab += (gb.dy-ga.dy)/2.f;
        else yMean_ab -= (gb.dy-ga.dy)/2.f;
    }

    zMean_ab = (ga.z * ga.N + gb.z * gb.N)/ ( N_ab );
    xMeanDiff = ga.x-gb.x , yMeanDiff = ga.y - gb.y, zMeanDiff = ga.z - gb.z;
    xStdDev_ab = min(ga.dx,gb.dx) + abs(ga.dx-gb.dx)/2.f + abs(xMeanDiff)/2.f;
    yStdDev_ab = min(ga.dy,gb.dy) + abs(ga.dy-gb.dy)/2.f + abs(yMeanDiff)/2.f;
    zStdDev_ab = sqrt(
      ( ( ga.N * (ga.dz*ga.dz) + gb.N * (gb.dz * gb.dz) ) / N_ab  ) +
      ((ga.N * gb.N)/(N_ab*N_ab)) * (zMeanDiff*zMeanDiff) );

    float Mx, MXy , My , MYx,ang;

    if(ga.x > gb.x)
    {
        Mx = ga.x;     MXy = ga.y;
    }else
    {
        Mx = gb.x;     MXy = gb.y;
    }
    if(ga.y > gb.y)
    {
        My = ga.y;     MYx = ga.x;
    }else
    {
        My = gb.y;     MYx = gb.x;
    }

//    ang = calcGaussianAng(xMean_ab, yMean_ab,abs(ga.x-gb.x), abs(ga.y-gb.y),
//                          Mx,MXy,My, MYx);
    cout << "Merge a:" << ga.x << " , " << ga.y << " , " << ga.dx << " , " << ga.dy << endl;
    cout << "Merge b:" << gb.x << " , " << gb.y << " , " << gb.dx << " , " << gb.dy << endl;
    cout << "Resp   :" << xMean_ab << " , " << yMean_ab << " , " << xStdDev_ab << " , " << yStdDev_ab << endl;

    sd->gaussians.push_back(Gaussian(xMean_ab, yMean_ab, zMean_ab,
                                 xStdDev_ab, yStdDev_ab, zStdDev_ab,
                                 ang, N_ab));
}

void Sonar::createGraph(SonarDescritor *sd)
{
    sd->createGraph(graphLinkDistance);
}

void Sonar::createGraphNeighborRelative(SonarDescritor *sd)
{
    sd->createGraphNeighborRelative(graphNeigborDistanceRelativeLink);
}

Sonar::Sonar(const SonarConfig &config):
    matcher(config)
{
    segmentation.computeHistogranThreshold = config.sonar_computeHistogranThreshold;
    segmentation.computeMaxPixel = config.sonar_computeMaxPixel;
    segmentation.maxPixel = config.sonar_maxPixel;
    stdDevMultiply = config.sonar_stdDevMultiply;

    graphLinkDistance = config.sonar_graphLinkDistance;
    graphNeigborDistanceRelativeLink = config.sonar_graphNeigborDistanceRelativeLink;
    graphCreatorMode = config.sonar_graphCreatorMode;

    segmentation.pixelThreshold = config.sonar_pixelThreshold;
    segmentation.searchThreshold = config.sonar_searchThreshold;
    segmentation.minSampleSize = config.sonar_minSampleSize;
    segmentation.maxSampleSize = config.sonar_maxSampleSize;

    drawPixelFound = config.sonar_drawPixelFound;
    drawElipses = config.sonar_drawElipses;
    drawElipsesAngle = config.sonar_drawElipsesAngle;
    drawEdges = config.sonar_drawEdges;
    drawEdgesAngle = config.sonar_drawEdgesAngle;
    drawEdgesInvAngle = config.sonar_drawEdgesInvAngle;
    drawVertexID = config.sonar_drawVertexID;
    drawStdDevValue = config.sonar_drawStdDevValue;

    drawEachVetex = false;

    storeImgs = true;

    saveGraphImg = config.sonar_saveGraphImg;
    saveTruncateImg = config.sonar_saveTruncateImg;
    saveMatchImgs = config.sonar_saveMatchImgs;
}

Sonar::~Sonar()
{
    clearDescriptors();
}

SonarDescritor *Sonar::newImage(Mat img)
{
    cronometer.reset();

    // Save reference to 16bits image
    img16bits = img;

    // Reset Color Mat for new draws
    colorImg = Mat::zeros(img16bits.rows,img16bits.cols, CV_8UC3);

    cout << "PDI:: Using Threshold = " << (unsigned) segmentation.pixelThreshold << endl;

    SonarDescritor *sd = new SonarDescritor;

    createGaussian(img16bits, sd);

    if(graphCreatorMode == CLOSEST_NEIGHBOR_RELATIVE_DISTANCE)
        createGraphNeighborRelative(sd);
    else if(graphCreatorMode == FIXED_DISTANCE)
        createGraph(sd);

    char fileName[200];

    // LSB IMAGE
    Mat LSBImg;
    //    subtract(img,0,img);
    img16bits.convertTo(LSBImg,CV_8U);

    if(saveTruncateImg)
    {
    sprintf(fileName,"frame_%.5d_8bitsTrucate.png", descriptors.size());
    imwrite(fileName, LSBImg);
    }

    if(storeImgs)
        storedImgs.push_back(LSBImg);

    resize(LSBImg,LSBImg,Size(800,600));
    imshow("Least Significant Bit Truncate Image", LSBImg);

    img16bits.convertTo(colorImg,CV_8U);
    cvtColor(colorImg,colorImg,CV_GRAY2BGR);

    imwrite("f4a.png", colorImg);
    drawGaussians(colorImg,sd);
    imwrite("f4b.png", colorImg);

    if(saveGraphImg)
    {
    sprintf(fileName,"frame_%.5d_Graph.png", descriptors.size());
    imwrite(fileName, colorImg);
    }

    resize(colorImg,colorImg,Size(800,600));
    imshow("Color Img", colorImg);

    descriptors.push_back(sd);

    if(drawEachVetex)
    {
        for(unsigned i = 0 ; i < sd->gaussians.size() ; i ++)
        {
            Mat dVertex = Mat::zeros(img.rows, img.cols, CV_8UC3);
            SonarDescritor::drawVertex(dVertex, sd, i, Rect(0,0,dVertex.cols, dVertex.rows),
                                       Scalar(0,255,0),Scalar(255,0,255));
            imshow("Vertex", dVertex);
            waitKey();
        }
    }

    if(descriptors.size() > 1)
    {
        for(int frame = descriptors.size()-2; frame >= 0; frame--)
        {
           vector<pair<unsigned,unsigned> > matches;
           Mat sdImg1 = Mat::zeros(img.rows,img.cols,CV_8UC3),
            sdImg2 = Mat::zeros(img.rows,img.cols,CV_8UC3),
            matchImg = Mat::zeros(img.rows,img.cols,CV_8UC3);

//        matcher.matche(descriptors[descriptors.size()-1], sd,matches);
        matcher.matcheSIFTCut(descriptors[frame], sd,matches);
//        matcher.matcheM(descriptors[frame], sd,matches);

        if(storeImgs)
            matcher.drawMatchOnImgs(matchImg,Size2i(img.cols*1.2,img.rows),
                                    descriptors[frame], storedImgs[frame],
                                    sd, storedImgs[descriptors.size()-1],
                                    matches);
        else
            matcher.drawMatch(descriptors[frame], sd,matches, matchImg);

        drawGaussians(sdImg1,descriptors[frame]);
        drawGaussians(sdImg2,sd);

//        SonarDescritor::drawDescriptor(sdImg1,descriptors[descriptors.size()-1]);
//        SonarDescritor::drawDescriptor(sdImg2,sd);

        if(saveMatchImgs)
        {
        sprintf(fileName,"frame_%.5d-%.5d_Graph_1.png", descriptors.size()-1, descriptors.size());
        imwrite(fileName, sdImg1);

        sprintf(fileName,"frame_%.5d-%.5d_Graph_2.png", descriptors.size()-1, descriptors.size());
        imwrite(fileName, sdImg2);

        sprintf(fileName,"frame_%.5d-%.5d_Match.png", descriptors.size()-1, descriptors.size());
        imwrite(fileName, matchImg);
        }

        resize(sdImg1,sdImg1,Size(800,600));
        imshow("SD before", sdImg1);

        resize(sdImg2,sdImg2,Size(800,600));
        imshow("SD after", sdImg2);

        resize(matchImg,matchImg,Size(1400,600));
        imshow("Graph Match", matchImg);

//            if(descriptors.size()%10 != 0)
                break;
            waitKey();
        }
    }

    cout << "Execution time = " << cronometer.read() << " usec" << endl;
    return sd;
}

SonarDescritor *Sonar::newImageDebug(Mat &imgGray16bits, Mat &bgr_imgResult)
{
    cronometer.reset();

    // Save reference to 16bits image
    img16bits = imgGray16bits;

    cout << "PDI:: Using Threshold = " << (unsigned) segmentation.pixelThreshold << endl;

    SonarDescritor *sd = new SonarDescritor;

    createGaussian(img16bits, sd);


    if(graphCreatorMode == CLOSEST_NEIGHBOR_RELATIVE_DISTANCE)
        createGraphNeighborRelative(sd);
    else if(graphCreatorMode == FIXED_DISTANCE)
        createGraph(sd);

    descriptors.push_back(sd);

    img16bits.convertTo(bgr_imgResult,CV_8UC1);
    bgr_imgResult.convertTo(bgr_imgResult,CV_8UC3);

    SonarDescritor::drawDescriptor(bgr_imgResult,sd);

    cout << "Execution time = " << cronometer.read() << " usec" << endl;
    return sd;
}

SonarDescritor *Sonar::newImageDirect(Mat &img)
{
    cronometer.reset();

    // Save reference to 16bits image
    img16bits = img;

    cout << "PDI:: Using Threshold = " << (unsigned) segmentation.pixelThreshold << endl;

    SonarDescritor *sd = new SonarDescritor;

    createGaussian(img16bits, sd);



    if(graphCreatorMode == CLOSEST_NEIGHBOR_RELATIVE_DISTANCE)
        createGraphNeighborRelative(sd);
    else if(graphCreatorMode == FIXED_DISTANCE)
        createGraph(sd);

    descriptors.push_back(sd);

    cout << "Execution time = " << cronometer.read() << " usec" << endl;
    return sd;
}

void Sonar::clearDescriptors()
{
    for(unsigned i = 0 ; i < descriptors.size() ; i++)
    {
        delete descriptors[i];
    }
    descriptors.clear();
}

/*
  Anotacoes:
     Correção de setore:
        - Existem setores com ganho maior que outros talvez por causa da sujeira do sensor
     Problema de "borrar" a imagem por causa do mov do robo.
        - Testar calcular média e desvio padrao em funcao de coordenadas polares

     Threshold com decaimento não é bom porque acaba detectando coisas que não deveriam ser detectadas

*/
