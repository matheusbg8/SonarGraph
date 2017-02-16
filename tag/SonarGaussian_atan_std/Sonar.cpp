#include "Sonar.h"
#include "SonarDescritor.h"
#include <iostream>
#include <cstdio>
#include "SonarConfig.h"

using namespace std;

typedef pair<unsigned, unsigned> pii;

const int Sonar::neighborRow[8] = { -1, -1, -1,  0,  0,  1, 1, 1};
const int Sonar::neighborCol[8] = { -1,  0,  1, -1,  1, -1, 0, 1};

void Sonar::floodFill(unsigned row, unsigned col)
{
    if(floodFillResultCount > maxSampleSize)
    {
        cout << "Warning, searching was stoped because a large pixel founded!!" << endl;
        return ;
    }
    searchMask.at<uchar>(row,col) = 255;

    /* Save pixel result */
    /*
      Channel 1 - Pixel Row
      Channel 2 - Pixel Col
      Channel 3 - Pixel Intensity
    */
    unsigned resultElemmentPosition = floodFillResultCount*3;
    floodFillResult.at<ushort>(0,resultElemmentPosition) = row;
    floodFillResult.at<ushort>(0,resultElemmentPosition+1) = col;
    floodFillResult.at<ushort>(0,resultElemmentPosition+2) = img16bits.at<ushort>(row,col);
    floodFillResultCount++;

    if(MRow < row)
    {
        MRCol = col;
        MRow = row;
    }
    if(MCol < col)
    {
        MCol = col;
        MCRow = row;
    }

    for(unsigned neighbor = 0 ; neighbor < 8 ; neighbor++)
    {
        unsigned nextRow = row + neighborRow[neighbor],
                 nextCol = col + neighborCol[neighbor];

        if(nextRow >= img8bits.rows ||
           nextCol >= img8bits.cols)
            continue;

        if(img16bits.at<ushort>(nextRow , nextCol) >= searchThreshold
           && searchMask.at<uchar>(nextRow,nextCol) != 255)
        {
            floodFill(nextRow , nextCol);
        }
    }
}

float Sonar::calcGaussianAng(float x, float y, float dx, float dy, float Mx, float MXy, float My, float MYx)
{
    float ang;

    ang = 180.0*atan2(dx,dy)/M_PI;
    if(dx < dy && MYx > x) // Change rotation to counterclockwiser
            ang = -ang;

    if(dx > dy && MXy > y)// Change rotation to counterclockwiser
            ang = -ang;
    return ang;
}

void Sonar::calculateMeanAndStdDerivation(Mat &img, int row, int col,
                                          float &rowMean, float &colMean, float &pixelMean,
                                          float &rowStdDev, float &colStdDev, float &pixelStdDev,
                                          float &ang, unsigned &N)
{
    // Clear old floodfill results
    if(floodFillResult.rows != 1 || floodFillResult.cols != img.rows * img.cols)
    {
        cout << "Realoc!!" << endl;
        floodFillResult = Mat(1, img.rows * img.cols, CV_16UC3, Scalar(0,0,0));
    }
    floodFillResultCount = 0;
    MCol = MRow = 0;
    MCRow = MRCol = 99999;

    // Search pixels
    floodFill(row , col);

    // Calculate first mean and standard deviation
    Mat segmentResult = floodFillResult(Rect(0,0,floodFillResultCount,1));
    Scalar mean, dev;
    meanStdDev(segmentResult, mean, dev);

    ang = calcGaussianAng(mean.val[1],mean.val[0],dev.val[1],dev.val[0],
                          MCol,MCRow,MRow,MRCol);

    if(drawPixelFound)
        drawResultPixels(colorImg,Scalar(0,0,255));

    rotateResult(floodFillResultCount, -ang ,mean.val[1],mean.val[0]);

    // Calculate final mean and standard deviation
    meanStdDev(segmentResult, mean, dev);

    rowMean = static_cast<float>(mean.val[0]);
    colMean = static_cast<float>(mean.val[1]);
    pixelMean = static_cast<float>(mean.val[2]);
    rowStdDev = static_cast<float>(dev.val[0]);
    colStdDev = static_cast<float>(dev.val[1]);
    pixelStdDev = static_cast<float>(dev.val[2]);
    N = floodFillResultCount;


    if(drawPixelFound)
        drawResultPixels(colorImg,Scalar(255,0,255));

}

void Sonar::drawResultPixels(Mat &img, Scalar color)
{
    for(unsigned i = 0 ; i < floodFillResultCount ; i++)
    {
        // Here:
        //   loodFillResult.at<ushort>(0, k ) - Acess the row of K-th pixel found by floodfill search
        //   floodFillResult.at<ushort>(0, k+1 ) - Acess the col of K-th pixel found by floodfill search
        //   floodFillResult.at<ushort>(0, k+2 ) - Acess the pixel intensity of K-th pixel found by floodfill search
        /// @todo - Acessing uchar Mat::data[] is faster than at<type>()

        unsigned resultStartMemo = i*3,
                pixelRow = floodFillResult.at<ushort>(0,resultStartMemo),
                pixelCol = floodFillResult.at<ushort>(0,resultStartMemo+1),
                pixelColStart = pixelCol*3;
        if(pixelRow >= 0 && pixelRow < img.rows &&
           pixelCol >= 0 && pixelCol < img.cols)
        {
            img.at<uchar>(pixelRow, pixelColStart) = color.val[0];
            img.at<uchar>(pixelRow, pixelColStart+1) = color.val[1];
            img.at<uchar>(pixelRow, pixelColStart+2) = color.val[2];
        }else
        {
            cout << "Pixel out of image( " <<
                    img.rows << " , " << img.cols << " )!!! ( "
                 << pixelRow << " , " << pixelCol << ")" << endl;
        }
    }
}

void Sonar::rotateResult(unsigned N, float angDegree, float ox, float oy)
{
    float angRad = (angDegree*M_PI)/180.f,
          cosA = cos(angRad),
          sinA = sin(angRad);

    for(unsigned i=0; i < N ; i++)
    {
        unsigned resultPostiotion = i*3;
        float ny = floodFillResult.at<ushort>(0,resultPostiotion)-oy, // Acess pixel's row
              nx = floodFillResult.at<ushort>(0,resultPostiotion+1)-ox, // Acess pixel's col
              rx = nx*cosA - ny*sinA,
              ry = nx*sinA + ny*cosA;

        floodFillResult.at<ushort>(0,resultPostiotion) = // Access the row
                roundf(  // It's important! round the values!!
                         nx*sinA + ny*cosA + oy  // Row <-> y
                      );

        floodFillResult.at<ushort>(0,resultPostiotion+1) = // Access the col
                roundf(  // It's important! round the values!!
                         nx*cosA - ny*sinA + ox // Col <-> x
                    );
    }
}

void Sonar::createGaussian(Mat &img, SonarDescritor *sd)
{
    /* Initialize Mask of Visit */
    searchMask = Mat(img.rows, img.cols, CV_8UC1, Scalar(0));

    /* Clear old Gaussians */
    sd->gaussians.clear();

    /* Do some image transform */
    if(img.type() != CV_8UC3)
    {
        cvtColor(img, colorImg, CV_GRAY2BGR);
    }
    else
    {
        img.copyTo(colorImg);
        cvtColor(img,img,CV_BGR2GRAY);
    }

    float rowMean,colMean,pixelMean,ang,
          rowStdDev,colStdDev, pixelStdDev;

    /* Search for high intensity pixels */
    unsigned linJump = 5, colJump = 5,N;
    for(unsigned lin = 0 ; lin < img.rows ; lin+=linJump)
    {
        for(unsigned col = 0 ; col < img.cols ; col+=colJump)
        {
            if(searchMask.at<uchar>(lin,col) == 0 &&
               img16bits.at<ushort>(lin,col) >= pixelThreshold)
            {
                calculateMeanAndStdDerivation(img, lin, col,
                                            rowMean,colMean,pixelMean,
                                            rowStdDev, colStdDev, pixelStdDev,ang,N);

//                cout << "( " << rowMean << " , " << colMean << " , " << pixelMean <<
//                 ") ( " << rowStdDev << " , " << colStdDev <<" , " << pixelStdDev <<
//                        ") (" << ang  << " , " << N << " )" << endl;
                if(N >= minSampleSize)
                    sd->gaussians.push_back(Gaussian(colMean,rowMean,pixelMean,
                                             colStdDev*stdDevMultiply,rowStdDev*stdDevMultiply,pixelStdDev,ang,N));
            }
        }
    }
    //pseudoMergeGaussian(0,1);
    //mergeGaussian(0,1);
}

void Sonar::drawGaussians(Mat colorImg, SonarDescritor *sd)
{
    float ri = 0x00, rf = 0xff, dr = rf-ri, r,
          gi = 0x00, gf = 0x00, dg = gf-gi, g,
          bi = 0xff, bf = 0x00, db = bf-bi, b,
          mt=searchThreshold, Mt=0.f, dt, t;

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
            float drawAng = sd->gaussians[i].ang;
            if(sd->gaussians[i].dx > sd->gaussians[i].dy)
            {
                if(drawAng<0.f)
                    drawAng =   90.f + sd->gaussians[i].ang;
                else drawAng = -90.f + sd->gaussians[i].ang;
            }

            ellipse(colorImg,
                    Point2f(sd->gaussians[i].x, sd->gaussians[i].y),
                    Size2f(sd->gaussians[i].dx, sd->gaussians[i].dy),
                    drawAng,//180.0*atan2(sd->gaussians[i].dy,sd->gaussians[i].dx)/M_PI,
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

    ang = calcGaussianAng(xMean_ab, yMean_ab,xStdDev_ab, yStdDev_ab,
                          Mx,MXy,My, MYx);

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

    ang = calcGaussianAng(xMean_ab, yMean_ab,abs(ga.x-gb.x), abs(ga.y-gb.y),
                          Mx,MXy,My, MYx);
    cout << "Merge a:" << ga.x << " , " << ga.y << " , " << ga.dx << " , " << ga.dy << endl;
    cout << "Merge b:" << gb.x << " , " << gb.y << " , " << gb.dx << " , " << gb.dy << endl;
    cout << "Resp   :" << xMean_ab << " , " << yMean_ab << " , " << xStdDev_ab << " , " << yStdDev_ab << endl;

    sd->gaussians.push_back(Gaussian(xMean_ab, yMean_ab, zMean_ab,
                                 xStdDev_ab, yStdDev_ab, zStdDev_ab,
                                 ang, N_ab));
}

void Sonar::createGraph(SonarDescritor *sd)
{
    sd->graph.clear();
    sd->graph.resize(sd->gaussians.size());

    for(unsigned i = 0 ; i < sd->gaussians.size() ; i++)
    {
        float cx = sd->gaussians[i].x,
              cy = sd->gaussians[i].y;

        for(unsigned j = i+1 ; j < sd->gaussians.size() ; j++)
        {
            float x = sd->gaussians[j].x,
                  y = sd->gaussians[j].y,
                  dx=x-cx, dy=y-cy,
                  d = sqrt(dx*dx + dy*dy);

            if(d <= graphLinkDistance)
            {
                float dt = 180.f*atan2f(dx,-dy)/M_PI;
                if(dt<0.f) dt+=360.f;

                sd->graph[i].push_back(new GraphLink(dt,d,j));

                if(dt > 180.f) dt-=180.f;
                else dt+= 180.f;

                sd->graph[j].push_back(new GraphLink(dt,d,i));
            }
        }

        GraphLink::computeInvAng(sd->graph[i]);
    }
}

void Sonar::createGraphNeighborRelative(SonarDescritor *sd)
{
    /// @todo - Consider using KD-Tree for optimized searchers (you need to change a heart of this code for do this)

    sd->graph.clear();
    sd->graph.resize(sd->gaussians.size());

    vector<vector<float> > distances(sd->gaussians.size(), vector<float>(sd->gaussians.size() ));
    vector<vector<uchar> > linkDone(sd->gaussians.size(), vector<uchar>(sd->gaussians.size(),0));

    for(unsigned i = 0 ; i < sd->gaussians.size() ; i++)
    {
        float cx = sd->gaussians[i].x,
              cy = sd->gaussians[i].y;

        // Compute the distances to vertex i.
        for(unsigned j = i+1 ; j < sd->gaussians.size() ; j++)
        {
            float x = sd->gaussians[j].x,
                  y = sd->gaussians[j].y,
                  dx=x-cx, dy=y-cy,
                  d = sqrt(dx*dx + dy*dy);

            distances[i][j] = d;
            distances[j][i] = d;
        }
        distances[i][i] = FLT_MAX;

        // Search the closest neighbord vertex to i
        float closestNeighborDistance=FLT_MAX;/* *min_element(distances[i].begin(), distances[i].end());*/

        for(unsigned j = 0; j < sd->gaussians.size() ; j++)
        {
            if(closestNeighborDistance > distances[i][j])
                closestNeighborDistance = distances[i][j];
        }

        // Create the edges relative to vertex i
        for(unsigned j = 0; j < sd->gaussians.size() ; j++)
        {
            if(i==j) continue;
            float dist_ij = distances[i][j],
                  dx=sd->gaussians[j].x-cx, dy=sd->gaussians[j].y-cy;

            if(linkDone[i][j] == 0 && dist_ij-closestNeighborDistance <= graphNeigborDistanceRelativeLink)
            {
//                cout << "link between " << i << " and " << j << " distance " << dist_ij << " cDistance " << closestNeighborDistance
//                     << " (" << sd->gaussians[i].x << " , " << sd->gaussians[i].y << ") ("
//                     <<  sd->gaussians[j].x << " , " << sd->gaussians[j].y << ")" << endl;
                // Link Happen's Mark

                linkDone[i][j] = 1;
                linkDone[j][i] = 1;

                // Calculate Edge wight (Edge degree relative to North, Vertex Distance)
                float dt = 180.f*atan2f(dx,-dy)/M_PI;
                if(dt<0.f) dt+=360.f;

                sd->graph[i].push_back(new GraphLink(dt,dist_ij,j));

                if(dt > 180.f) dt-=180.f;
                else dt+= 180.f;

                sd->graph[j].push_back(new GraphLink(dt,dist_ij,i));
            }
        }

        GraphLink::computeInvAng(sd->graph[i]);
    }
}

Sonar::Sonar(const SonarConfig &config):
    matcher(config)
{
    computeHistogranThreshold = config.sonar_computeHistogranThreshold;
    computeMaxPixel = config.sonar_computeMaxPixel;
    maxPixel = config.sonar_maxPixel;
    stdDevMultiply = config.sonar_stdDevMultiply;

    graphLinkDistance = config.sonar_graphLinkDistance;
    graphNeigborDistanceRelativeLink = config.sonar_graphNeigborDistanceRelativeLink;
    graphCreatorMode = config.sonar_graphCreatorMode;

    pixelThreshold = config.sonar_pixelThreshold;
    searchThreshold = config.sonar_searchThreshold;
    minSampleSize = config.sonar_minSampleSize;
    maxSampleSize = config.sonar_maxSampleSize;

    drawPixelFound = config.sonar_drawPixelFound;
    drawElipses = config.sonar_drawElipses;
    drawElipsesAngle = config.sonar_drawElipsesAngle;
    drawEdges = config.sonar_drawEdges;
    drawEdgesAngle = config.sonar_drawEdgesAngle;
    drawEdgesInvAngle = config.sonar_drawEdgesInvAngle;
    drawVertexID = config.sonar_drawVertexID;
    drawStdDevValue = config.sonar_drawStdDevValue;

    saveGraphImg = config.sonar_saveGraphImg;
    saveTruncateImg = config.sonar_saveTruncateImg;
    saveMatchImgs = config.sonar_saveMatchImgs;
}

SonarDescritor *Sonar::newImage(Mat img)
{
    cronometer.reset();

    // Save reference to 16bits image
    img16bits = img;

    // Normalize image to convert from 16 bits to 8 bits
    normalize(img16bits,img8bits, 0 , 255,NORM_MINMAX, CV_8UC1);

    // Reset Color Mat for new draws
    colorImg = Mat::zeros(img16bits.rows,img16bits.cols, CV_8UC3);

    if(computeMaxPixel)
        maxPixel = img16bits.rows * img16bits.cols * 0.005;

    if(computeHistogranThreshold)
    {
        cout << "maxPixel = " << maxPixel << endl;

        // Config histogram
        int grayBins = 256;  // Number of beans
        int histSize[] = { grayBins };

        float grayRange[] = { 0, 256 }; // Range gray scale channel
        const float* histRange[] = { grayRange };

        Mat hist;
        int channels[] = {0}; // Channels that we will process

        calcHist(&img8bits, 1 , channels , Mat(),
                 hist , 1 , histSize , histRange,
                 true, false);

        // Analyzing histogram and definig threshold
        unsigned sum = 0;
        for(pixelThreshold = 255 ; pixelThreshold > 0 && sum <= maxPixel; pixelThreshold--)
            sum += hist.at<float>(0,pixelThreshold); // Yes, it's float I don't know why

        cout << " sum = " << sum << " Threshold = " << (unsigned) pixelThreshold << endl;

        if(pixelThreshold == 0) pixelThreshold =1;

        if(pixelThreshold > 2)
            searchThreshold = pixelThreshold / 2;
        else searchThreshold = pixelThreshold;
        searchThreshold = pixelThreshold;
    }
    cout << "PDI:: Using Threshold = " << (unsigned) pixelThreshold << endl;


    SonarDescritor *sd = new SonarDescritor;

    createGaussian(img8bits, sd);

    if(graphCreatorMode == CLOSEST_NEIGHBOR_RELATIVE_DISTANCE)
        createGraphNeighborRelative(sd);
    else if(graphCreatorMode == FIXED_DISTANCE)
        createGraph(sd);

    vector<pair<unsigned,unsigned> > matches;
char fileName[200];
    if(descriptors.size() > 0)
    {
        Mat sdImg1 = Mat::zeros(colorImg.rows,colorImg.cols,CV_8UC3),
            sdImg2 = Mat::zeros(colorImg.rows,colorImg.cols,CV_8UC3),
            matchImg = Mat::zeros(colorImg.rows,colorImg.cols,CV_8UC3);

        matcher.matche(descriptors[descriptors.size()-1], sd,matches);
        matcher.drawMatch(descriptors[descriptors.size()-1], sd,matches, matchImg);

        drawGaussians(sdImg1,descriptors[descriptors.size()-1]);
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

        resize(matchImg,matchImg,Size(800,600));
        imshow("Graph Match", matchImg);
    }

    Mat LSBImg;
    //    subtract(img,0,img);
    img16bits.convertTo(LSBImg,CV_8U);

    if(saveTruncateImg)
    {
    sprintf(fileName,"frame_%.5d_8bitsTrucate.png", descriptors.size());
    imwrite(fileName, LSBImg);
    }

    resize(LSBImg,LSBImg,Size(800,600));
    imshow("Least Significant Bit Truncate Image", LSBImg);

    drawGaussians(colorImg,sd);

    if(saveGraphImg)
    {
    sprintf(fileName,"frame_%.5d_Graph.png", descriptors.size());
    imwrite(fileName, colorImg);
    }

    resize(colorImg,colorImg,Size(800,600));
    imshow("Color Img", colorImg);

    descriptors.push_back(sd);

    cout << "Execution time = " << cronometer.read() << " usec" << endl;
    return sd;
}

/*
  Anotacoes:
     Correção de setore:
        - Existem setores com ganho maior que outros talvez por causa da sujeira do sensor
     Problema de "borrar" a imagem por causa do mov do robo.
        - Testar calcular média e desvio padrao em funcao de coordenadas polares

     Threshold com decaimento não é bom porque acaba detectando coisas que não deveriam ser detectadas

*/
