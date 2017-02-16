#include "Gaussian.h"
#include "Drawing/Drawing.h"


#include <iostream>
using namespace std;
using namespace cv;

Gaussian::Gaussian():
    x(0.f),y(0.f),intensity(0.f),
    dx(0.f),dy(0.f),di(0.f),
    ang(0.f),N(0u)
{

}

Gaussian::Gaussian(float x, float y, float intensity,
                   float dx, float dy, float di,
                   float ang, unsigned N):
    x(x),y(y),intensity(intensity),
    dx(dx),dy(dy),di(di),
    ang(ang),N(N)
{

}

Gaussian::Gaussian(Segment *seg, float std):
    x(0.f),y(0.f),intensity(0.f),
    dx(0.f),dy(0.f),di(0.f),
    ang(0.f),N(0u)
{
    createGaussian2x2(seg,std);
//    createGaussian3x3(seg, std);
}

void Gaussian::createGaussian3x3(Segment *seg, float std)
{
    Mat sample = seg->result(Rect(0,0,seg->N,3)),
        mCov, mMean, eVal, eVect;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(sample, mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE ,
                    CV_32F);

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, eVal, eVect);

    y = mMean.at<float>(0,0); // Row mean
    x = mMean.at<float>(1,0); // Columns mean
    intensity = mMean.at<float>(2,0);

    dx = sqrt(eVal.at<float>(2,0)) * std;
    dy = sqrt(eVal.at<float>(1,0)) * std;
    di = sqrt(eVal.at<float>(0,0)) * std;
    N = seg->N;

    float fAng;
    fAng = 180.0*atan2(eVect.at<float>(1,1),-eVect.at<float>(1,0))/M_PI;
    if(fAng<0.f) fAng+= 180.f;

    ang = fAng;

//    cout << "Computed Gaussian " << *this << endl;

}

void Gaussian::createGaussian2x2(Segment *seg, float std)
{
    Mat mCov, mMean, eVal, eVect;

    // Intensity mean and std
    Scalar iMean, iStd;
    // Rect(x,y, widht, height) (result line 2 are intensitys found)
    meanStdDev(seg->result(Rect(0,2,seg->N,1)),iMean ,iStd);

//    cout << "Intensity mean and standard derivation (STD): "
//         << iMean.val[0] << " " << iStd.val[0]
//         << endl;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(seg->result(Rect(0,0,seg->N,2)), mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE,
                    CV_32F);

//    cout << "CovarianceMatrix (XvsY)"
//         << endl
//         << mCov
//         << endl
//         << "Mean Matrix"
//         << mMean
//         << endl;

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, eVal, eVect);

    y = mMean.at<float>(0,0);
    x = mMean.at<float>(1,0);
    intensity = iMean.val[0];

    dx = sqrt(eVal.at<float>(1,0)) * std;
    dy = sqrt(eVal.at<float>(0,0)) * std;// Greater eigenValue
    di = iStd.val[0] * std;
    N = seg->N;

    // Compute Gaussian angle using the tangent of greater eigenVector
    float fAng;
    fAng = 180.0*atan2(eVect.at<float>(0,1),-eVect.at<float>(0,0))/M_PI; // Horizontal (to north) image reference
    if(fAng<0.f) fAng+= 180.f;

    ang = fAng;

    return;
}

void Gaussian::clockWisePoints(Point2f &a, Point2f &b, Point2f &c, Point2f &d) const
{
    float radAng = ang*M_PI/180.f;
    Point2f p(x,y), // Our position
            pc(sin(radAng) , -cos(radAng)), // Principal component
            spc(pc.y, -pc.x), // Second principal component (ortogonal principal component)
            vp(pc*dy), // Principal displacement vector
            vsp(spc*dx); // Second principal displacement vector

    // We consider principal component is horizontal from left to right
    // dy always is the biggest standard deviation

    a = p+vsp; // at noon (north)
    b = p+vp; // three o'clock (east)
    c = p-vsp; // half hour (suth)
    d = p-vp;// nine o'clock (west)
}

void Gaussian::merge(const Gaussian &g, float std)
{
    if(dx+dy < g.dx+g.dy)
        *this = g;

//    x = (x + g.x) / 2.f;
//    y = (y + g.y) / 2.f;
    cout << "Gaussian Merge is not implemented yet" << endl;
    /*
    Segment seg;

    Point2f ps[8];

    clockWisePoints(ps[0],ps[1],ps[2],ps[3]);
    g.clockWisePoints(ps[4],ps[5],ps[6],ps[7]);

    seg.N = 8;
    seg.result =
      (Mat_<float>(3,8) <<
        ps[0].x,ps[1].x,ps[2].x,ps[3].x,ps[4].x,ps[5].x,ps[6].x,ps[7].x,
        ps[0].y,ps[0].y,ps[0].y,ps[0].y,ps[0].y,ps[0].y,ps[0].y,ps[0].y,
        intensity,intensity,intensity-di/2.f,intensity+di/2.f,g.intensity,g.intensity,g.intensity-g.di/2.f,g.intensity+g.di/2.f
       );

    createGaussian2x2(&seg,std);
    */
}


/**
 * @brief
 *  Compute Z (orthogonal value) between vectors:
 *   a->b x a->c = answer
 *
 *  the signal follow the left hand law.
 * @param a - Reference point of cross dot.
 * @param b - First extreme point.
 * @param c - Second extreme point.
 * @return float
 */
inline float crossDot2D(Point2f &a,Point2f &b,Point2f &c)
{
/*
 * | i    j    k|
 * | abx  aby  0| a->b
 * | acx  acy  0| a->c
 *
 * k = abx * acy - aby * acy
 *
 */

    return (b.x-a.x) * (c.y-a.y) - (c.x - a.x) * (b.y - a.y);
}

bool Gaussian::hasIntersection(const Gaussian &a, const Gaussian &b)
{
    // Points on clock wise
    Point2f ap[4],bp[4];

    a.clockWisePoints(ap[0],ap[1],ap[2],ap[3]);
    b.clockWisePoints(bp[0],bp[1],bp[2],bp[3]);


//    Mat mat(781,1430,CV_8UC3,Scalar(0,0,0));
//    for(unsigned i = 0 ; i < 4; i++)
//    {
//        circle(mat,ap[i],2,Scalar(0,0,255));
//        circle(mat,bp[i],2,Scalar(0,255,0));
//        imshow("instersection test", mat);
//        waitKey();

//    }
//    imshow("instersection test", mat);
//    waitKey();

    char fttt=0;
    for(unsigned i = 0 ; i < 4 ; i++)
    {
        char r1 = crossDot2D(bp[0],bp[1],ap[i]) >= 0.f,
             r2 = crossDot2D(bp[1],bp[2],ap[i]) >= 0.f,
             r3 = crossDot2D(bp[2],bp[3],ap[i]) >= 0.f,
             r4 = crossDot2D(bp[3],bp[0],ap[i]) >= 0.f,
             r = r1+r2+r3+r4;

        // true = inside
        if(r == 4)
        {
            cout << "intersection " << ap[i]
                 << " inside "
                 << bp[0] << ' '
                 << bp[1] << ' '
                 << bp[2] << ' '
                 << bp[3];
            return true;
        }
        if(r==3) fttt++;
    }

    for(unsigned i = 0 ; i < 4 ; i++)
    {
        char r1 = crossDot2D(ap[0],ap[1],bp[i]) >= 0.f,
             r2 = crossDot2D(ap[1],ap[2],bp[i]) >= 0.f,
             r3 = crossDot2D(ap[2],ap[3],bp[i]) >= 0.f,
             r4 = crossDot2D(ap[3],ap[0],bp[i]) >= 0.f,
             r = r1+r2+r3+r4;

        if(r==4)
        {
            cout << "intersection " << bp[i]
                 << " inside "
                 << ap[0] << ' '
                 << ap[1] << ' '
                 << ap[2] << ' '
                 << ap[3];
            return true;
        }
        if(r==3) fttt++;
    }
    if(fttt>=6) // Special Star Case
        return true;

    return false;
}

void Gaussian::createCrazzyGaussian(Mat &img16Bits, Segment *seg, float std)
{
    Mat mCov, mMean, eVal, eVect;

    // Intensity mean and std
    Scalar iMean, iStd;
    // Rect(x,y, widht, height) (result line 2 are intensitys found)
    meanStdDev(seg->result(Rect(0,2,seg->N,1)),iMean ,iStd);

//    cout << "Intensity mean and standard derivation (STD): "
//         << iMean.val[0] << " " << iStd.val[0]
//         << endl;

    // Calculate Covariance Matrix of the sample found
    calcCovarMatrix(seg->result(Rect(0,0,seg->N,2)), mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE,
                    CV_32F);

//    cout << "CovarianceMatrix (XvsY)"
//         << endl
//         << mCov
//         << endl
//         << "Mean Matrix"
//         << mMean
//         << endl;

    // Calculate EigenValues and EigenVector of Covariance Matrix
    eigen(mCov, eVal, eVect);

    y = mMean.at<float>(0,0);
    x = mMean.at<float>(1,0);
    intensity = iMean.val[0];

    dx = sqrt(eVal.at<float>(1,0)) * std;
    dy = sqrt(eVal.at<float>(0,0)) * std;// Greater eigenValue
    di = iStd.val[0] * std;
    N = seg->N;

    // Compute Gaussian angle using the tangent of greater eigenVector
    float fAng;
    fAng = 180.0*atan2(eVect.at<float>(0,1),-eVect.at<float>(0,0))/M_PI; // Horizontal (to north) image reference
    if(fAng<0.f) fAng+= 180.f;

    ang = fAng;

    float mdx=dx, mdy=dy;


    // ============= First cut ==========================
    RotatedRect rotRect( Point2f(x,y),Size2f(mdy,mdx), ang-90.f );
    Rect rect(rotRect.boundingRect()); // Crop to extrac segment of image
    img = img16Bits(rect).clone();
    // ==================================================

    // Our new center after crop
    Point2f centerBeforTransform(img.cols/2.f,img.rows/2.f);

    // Compute our afim transform, rotation values
    Mat afimTransformMatrix = getRotationMatrix2D( centerBeforTransform , rotRect.angle,1.0);

    // Compute our image size after afim transform (becase of rotation)
    Rect afterRotatioBox = RotatedRect(centerBeforTransform, img.size(), rotRect.angle).boundingRect();

    // Our new center after afim transform
    Point2f centerAfterTransform(afterRotatioBox.width/2.f, afterRotatioBox.height/2.f);


    // Translate image to be at same center of result image
    // Our image will grow to left and to up becouse the rotation
    float tx = centerAfterTransform.x - centerBeforTransform.x,
          ty = centerAfterTransform.y - centerBeforTransform.y;

    // Adjust transformation matrix to translate
    afimTransformMatrix.at<double>(0,2) += tx;
    afimTransformMatrix.at<double>(1,2) += ty;

    // Apply afim transform the warped image
    warpAffine( img,
                img,
                afimTransformMatrix,
                afterRotatioBox.size()
               );


    // ===========  Last cut (ajust to rotated feature) =====
    Rect lastCut(Rect(centerAfterTransform.x - mdy/2.f, centerAfterTransform.y - mdx/2.f , mdy, mdx));
    img = img(lastCut);

    // ============   Compute image moments ==================
    // Convert img 16 bits gray scale to float 32 bits
    Mat imf32F;
    img.convertTo(imf32F,CV_32FC1);

    Moments m = moments(imf32F,false);

    // ============== Compute hu moments ============
//    Point2f centerOfMass(m.m10/m.m00, m.m01/m.m00);

//    Mat tmpImg;
//    img.convertTo(tmpImg,CV_8UC1);
//    cvtColor(tmpImg,tmpImg,CV_GRAY2BGR);

//    circle(tmpImg,Point2f(img.cols/2, img.rows/2),2,Scalar(0,255,0),-1);
//    circle(tmpImg,centerOfMass,2,Scalar(0,0,255),-1);

//    resize(tmpImg,tmpImg,Size(tmpImg.cols*5,tmpImg.rows*5));
//    namedWindow("Extracted Feature");
//    imshow("Extracted Feature", tmpImg);

    // ==============================================
    HuMoments(m, hu);

//    cout << "HuMoments:"
//         << endl;
//    for(unsigned i = 0 ; i < 7;i++)
//        cout << "h[" << i << "]= "
//             << hu[i] << endl;
}

void Gaussian::createCrazzyGaussian2(Mat &img16Bits, Segment *seg, float std)
{
    N = seg->N;

    // Compute moments
    vector<Mat> contours;
    seg->toOrderedContours(contours);

    unsigned rawContourId = contours.size(),
             ConvHullId = rawContourId+1;
    contours.resize(contours.size()+2);

    seg->toContour(contours[rawContourId]);
    convexHull(contours[rawContourId],contours[ConvHullId]);

    Moments m = moments(contours[rawContourId]);
    HuMoments(m, hu);

    area = perimeter= 0;
    for(unsigned i= 0; i < rawContourId;i++)
    {
        area += contourArea(contours[i]);
        perimeter = arcLength(contours[i],false);
    }

    convexHullArea = contourArea(contours[ConvHullId]);

    RotatedRect rec = fitEllipse(contours[rawContourId]);

    x = rec.center.x;
    y = rec.center.y;

    dx = rec.size.width;
    dy = rec.size.height;

    ang = rec.angle;

    // Intensity mean and std
    Scalar iMean, iStd;

    // Rect(x,y, widht, height) (result line 2 are intensitys found)
    meanStdDev(seg->result(Rect(0,2,seg->N,1)),iMean ,iStd);
    intensity = iMean.val[0];
    di = iStd.val[0] * std;

    cout << "I= " << intensity << endl
         << "Dx= " << dx << endl
         << "Dy= " << dy << endl
         << "DI= " << di << endl
         << "ang= " << ang << endl
         << "N= " << N << endl
         << "Contour Found = " << (contours.size() - 2) << endl;
    for(unsigned i= 0; i < rawContourId;i++)
    {
        cout << "Contour " << i << " size = " << contours[i].size().height << endl;
    }
    cout << "Hull size = " << contours[ConvHullId].size().height << endl
         << "Perimeter = " << perimeter << endl
         << "Raw size = " << N << endl
         << "Area = " << area << endl
         << "Convex Hull Area = " << convexHullArea << endl << endl;

    #ifdef GAUSSIAN_DEBUG
    Mat result,colorImg;

    img16Bits.convertTo(colorImg,CV_8UC1);
    cvtColor(colorImg,colorImg,CV_GRAY2BGR);

    drawContours(colorImg,contours,ConvHullId,Scalar(0.f,255.f,0.f)); // Green Convex Hull
    seg->drawSegment(colorImg,Scalar(0.f,0.f,255.f)); // Red
//    for(unsigned i= 0; i < rawContourId;i++)
//    {
//        drawContours(colorImg,contours,i,Drawing::color[i%Drawing::nColor],1); // Segment Contourns
//    }

    // Crop to extrac segment of image
    result = colorImg(rec.boundingRect()).clone();
//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_NEAREST);
    imshow("Feature2", result);
    imwrite("Feature2.png", result);

    rotatedRectCrop(colorImg,rec,result,10);

//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_AREA);
//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_LINEAR);
//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_NEAREST);
//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_CUBIC);
//    resize(result,result,Size(result.cols*10,result.rows*10),0,0,INTER_LANCZOS4);

    imwrite("Feature.png", result);
    namedWindow("Feature",WINDOW_AUTOSIZE);
    imshow("Feature", result);

    Drawing::drawGaussian(colorImg,*this, Scalar(0,0,255),Scalar(0,255,0),0,true,false,true);
    imshow("Color Img", colorImg);
    imwrite("Color Img.png", colorImg);

    waitKey();
    #endif
}

void Gaussian::createCrazzyGaussian3(Mat &img16Bits, Segment *seg, float std)
{
    N = seg->N;

    // Compute moments
    vector<Mat> contours;
    seg->toOrderedContours(contours);

    unsigned rawContourId = contours.size(),
             ConvHullId = rawContourId+1;
    contours.resize(contours.size()+2);

    seg->toContour(contours[rawContourId]);
    convexHull(contours[rawContourId],contours[ConvHullId]);

    Moments m = moments(contours[rawContourId]);
    HuMoments(m, hu);

    area = perimeter= 0;
    for(unsigned i= 0; i < rawContourId;i++)
    {
        area += contourArea(contours[i]);
        perimeter = arcLength(contours[i],false);
    }

    convexHullArea = contourArea(contours[ConvHullId]);

    Mat mCov, mMean, eVal, eVect;
    calcCovarMatrix(seg->result(Rect(0,0,seg->N,2)), mCov, mMean,
                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE,
                    CV_32F);
    eigen(mCov, eVal, eVect);

    x = mMean.at<float>(1,0);
    y = mMean.at<float>(0,0);

    dx = sqrt(eVal.at<float>(1,0)) * std;
    dy = sqrt(eVal.at<float>(0,0)) * std;

    float fAng;
    fAng = 180.0*atan2(eVect.at<float>(0,1),-eVect.at<float>(0,0))/M_PI; // Horizontal (to north) image reference
    if(fAng<0.f) fAng+= 180.f;

    ang = fAng;

    // Intensity mean and std
    Scalar iMean, iStd;

    // Rect(x,y, widht, height) (result line 2 are intensitys found)
    meanStdDev(seg->result(Rect(0,2,seg->N,1)),iMean ,iStd);
    intensity = iMean.val[0];
    di = iStd.val[0] * std;

    cout << "I= " << intensity << endl
         << "ang= " << ang << endl
         << "Contour Found = " << (contours.size() - 2) << endl;
    for(unsigned i= 0; i < rawContourId;i++)
    {
        cout << "Contour " << i << " size = " << contours[i].size().height << endl;
    }

    cout << "Hull size = " << contours[ConvHullId].size().height << endl
         << "==== 10D Features ==========" << endl
         << "Area = " << area << endl
         << "Convex Hull Area = " << convexHullArea << endl
         << "Width = " << dx << endl  // Width
         << "Height = " << dy << endl  // Height
         << "Inertia Ratio = " << dx / dy << endl // Inertia Ratio - circularty (circle = 1 line = 0)
         << "Std. Intensity = " << di << endl // Std Intensity
         << "Mean Intensity = " << intensity << endl // Mean Intensity
         << "Area = " << area << endl  // Area
         << "Hull Area = " << convexHullArea << endl  // Hull Area
         << "Convexity = " << area / convexHullArea << endl //Convexity
         << "Perimeter = " << perimeter << endl  // Perimeter
         << "Pixel Count = " << N  // Pixel Count
         << endl << endl;


    #ifdef GAUSSIAN_DEBUG
    Mat result,colorImg;

    Point boxC((seg->mCol+seg->MCol)/2.f, (seg->mRow+seg->MRow)/2.f);
    Size boxSz(seg->MRow-seg->mRow , seg->MCol-seg->mCol );

    // const Point2f& center, const Size2f& size, float angle
    RotatedRect rec(Point(x,y), boxSz,ang);

    img16Bits.convertTo(colorImg,CV_8UC1);
    rotatedRectCrop(colorImg,rec,result,400/dy);

    imwrite("FeatureGray.png", result);
    waitKey(1);

    cvtColor(colorImg,colorImg,CV_GRAY2BGR);

    drawContours(colorImg,contours,ConvHullId,Scalar(0.f,255.f,0.f)); // Green Convex Hull
    seg->drawSegment(colorImg,Scalar(0.f,0.f,255.f)); // Red
//    for(unsigned i= 0; i < rawContourId;i++)
//    {
//        drawContours(colorImg,contours,i,Drawing::color[i%Drawing::nColor],1); // Segment Contourns
//    }

    rotatedRectCrop(colorImg,rec,result,400/dy);
    imwrite("FeatureColor.png", result);

//    namedWindow("Feature",WINDOW_AUTOSIZE);
    imshow("Feature", result);

//    Drawing::drawGaussian(colorImg,*this, Scalar(0,0,255),Scalar(0,255,0),0,true,false,true);
    imshow("Color Img", colorImg);
//    imwrite("Color Img.png", colorImg);

    waitKey();
    #endif
}

string Gaussian::type2str(int type)
{
      string r;

      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);

      switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
      }

      r += "C";
      r += (chans+'0');

      return r;
}

void Gaussian::findOrderedContour(Segment *seg, vector<Mat> &contour)
{
    Mat m(seg->MRow-seg->mRow+1 , seg->MCol-seg->mCol+1, CV_8UC1,Scalar(0));

    for(unsigned i = 0; i < seg->N;i++)
    {
        m.at<uchar>( seg->result.at<ushort>(0,i) -seg->mRow,
                     seg->result.at<ushort>(1,i) -seg->mCol) = 255u;
    }

    Mat view;
    resize(m,view,Size(m.rows*10,m.cols*10),0,0,INTER_NEAREST);
    imshow("Bim img",view);

    findContours(m,contour,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE,Point(seg->mCol,seg->mRow));
    cout << type2str(contour[0].type()) << endl;
    waitKey();
}

void Gaussian::rotatedRectCrop(Mat &img16Bits, const RotatedRect &rotRect, Mat &result, int scale)
{
    // ============= First cut ==========================
    Rect rect(rotRect.boundingRect()); // Crop to extrac segment of image
    rect.width+=4; rect.height+=4;
    rect.x-=2; rect.y-=2;

    result = img16Bits(rect).clone();
    // ==================================================

    // Our new center after crop
    Point2f centerBeforTransform(result.cols/2.f,result.rows/2.f);

    // Compute our afim transform, rotation values
    Mat afimTransformMatrix = getRotationMatrix2D( centerBeforTransform, rotRect.angle-90.f,scale);

    // Compute our image size after afim transform (becase of rotation)
    Rect afterRotatioBox = RotatedRect(centerBeforTransform, result.size()*scale, rotRect.angle-90.f).boundingRect();

    // Our new center after afim transform
    Point2f centerAfterTransform(afterRotatioBox.width/2.f, afterRotatioBox.height/2.f);

    // Translate image to be at same center of result image
    // Our image will grow to left and to up becouse the rotation
    float tx = centerAfterTransform.x - centerBeforTransform.x,
          ty = centerAfterTransform.y - centerBeforTransform.y;

    // Adjust transformation matrix to translate
    afimTransformMatrix.at<double>(0,2) += tx;
    afimTransformMatrix.at<double>(1,2) += ty;

    // Apply afim transform the warped image
    warpAffine( result,
                result,
                afimTransformMatrix,
                afterRotatioBox.size(),
                INTER_NEAREST
               );

    // ===========  Last cut (ajust to rotated feature) =====
//    Rect lastCut(Rect(centerAfterTransform.x - mdy/2.f, centerAfterTransform.y - mdx/2.f , mdy, mdx));
    //    result = result(lastCut);
}

ostream & operator <<(ostream &os, const Gaussian& g)
{
    os << " X= " << g.x
       << " Y= " << g.y
       << " I= " << g.intensity
       << " Dx= " << g.dx
       << " Dy= " << g.dy
       << " DI= " << g.di
       << " ang= " << g.ang
       << " N= " << g.N;
    return os;
}



// ========== BACKUP FOR DEBUG !! ===============

//void Gaussian::createCrazzyGaussian(Mat &img16Bits, Segment *seg, float std)
//{
//    Mat mCov, mMean, eVal, eVect;

//    // Intensity mean and std
//    Scalar iMean, iStd;
//    // Rect(x,y, widht, height) (result line 2 are intensitys found)
//    meanStdDev(seg->result(Rect(0,2,seg->N,1)),iMean ,iStd);

////    cout << "Intensity mean and standard derivation (STD): "
////         << iMean.val[0] << " " << iStd.val[0]
////         << endl;

//    // Calculate Covariance Matrix of the sample found
//    calcCovarMatrix(seg->result(Rect(0,0,seg->N,2)), mCov, mMean,
//                    CV_COVAR_NORMAL | CV_COVAR_COLS | CV_COVAR_SCALE,
//                    CV_32F);

////    cout << "CovarianceMatrix (XvsY)"
////         << endl
////         << mCov
////         << endl
////         << "Mean Matrix"
////         << mMean
////         << endl;

//    // Calculate EigenValues and EigenVector of Covariance Matrix
//    eigen(mCov, eVal, eVect);

//    y = mMean.at<float>(0,0);
//    x = mMean.at<float>(1,0);
//    intensity = iMean.val[0];

//    dx = sqrt(eVal.at<float>(1,0)) * std;
//    dy = sqrt(eVal.at<float>(0,0)) * std;// Greater eigenValue
//    di = iStd.val[0] * std;
//    N = seg->N;

//    // Compute Gaussian angle using the tangent of greater eigenVector
//    float fAng;
//    fAng = 180.0*atan2(eVect.at<float>(0,1),-eVect.at<float>(0,0))/M_PI; // Horizontal (to north) image reference
//    if(fAng<0.f) fAng+= 180.f;

//    ang = fAng;

//    float mdx=dx*2.f, mdy=dy*2.f;


//    // ============= First cut ==========================
//    RotatedRect rotRect( Point2f(x,y),Size2f(mdy,mdx), ang-90.f );
//    Rect rect(rotRect.boundingRect());

//    Mat imgTmp;

//    img16Bits.convertTo(imgTmp,CV_8UC1);
//    cvtColor(imgTmp,imgTmp,CV_GRAY2BGR);

//    rectangle(imgTmp,rect,
//              Scalar(0.f,0.f,255.f));


//    Point2f vertices[4];
//    rotRect.points(vertices);
//    for (int i = 0; i < 4; i++)
//        line(imgTmp, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));


//    namedWindow("Extracted Feature");
//    imshow("Extracted Feature", imgTmp);
//    waitKey();

//    imgTmp = imgTmp(rect);

//    namedWindow("Extracted Feature");
//    imshow("Extracted Feature", imgTmp);
//    imwrite("teste.png", imgTmp);
//    waitKey();

//    //    Mat imgTmp = img16Bits(rec).clone();

//    Point2f rCenter= Point2f(rect.width/2.f,rect.height/2.f);

//    img = imgTmp;

//    Mat rot = getRotationMatrix2D( rCenter , rotRect.angle,1.0);
//    Rect bbox = cv::RotatedRect(rCenter, img.size(), rotRect.angle).boundingRect();

//    float tx = bbox.width/2.0 - rCenter.x,
//          ty = bbox.height/2.0 - rCenter.y;

//    // adjust transformation matrix
//    rot.at<double>(0,2) += tx;
//    rot.at<double>(1,2) += ty;

//    /// Rotate the warped image
//    warpAffine( img,
//                img,
//                rot,
//                bbox.size()
//               );

////    rCenter.x +=tx;
////    rCenter.y +=ty;
//    rCenter = Point2f(bbox.width/2.f, bbox.height/2.f);

//    namedWindow("Extracted Feature");
//    imshow("Extracted Feature", img);
//    waitKey();

//    // Last cut
//    Rect lastCut(Rect(rCenter.x - mdy/2.f, rCenter.y - mdx/2.f , mdy, mdx));

////    img = img(Rect(rCenter.x - dx*0.5f, rCenter.y-dy*0.5f, dx, dy));

////    img.convertTo(img,CV_8UC1);
////    cvtColor(img,img,CV_GRAY2BGR);

////    rectangle(img,rec,
////              Scalar(0.f,0.f,255.f));

//    rectangle(img,lastCut,
//              Scalar(255.f,0.f,0.f));

//    circle(img,rCenter,3,Scalar(0.f,0.f,255.f),-1);
////    circle(img,Point2f(rec.width/2.f,rec.height/2.f),3,Scalar(0.f,255.f,0.f),-1);
////    img.convertTo(img,CV_32FC1);

//    namedWindow("Extracted Feature");
//    imshow("Extracted Feature", img);
//    imwrite("teste2.png", img);

//    return;
//}


float dist(const Gaussian &a, const Gaussian &b)
{
    float dx = a.x - b.x,
          dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}


float dist2(const Gaussian &a, const Gaussian &b)
{
    float dx = a.x - b.x,
          dy = a.y - b.y;
    return dx*dx + dy*dy;
}

float angleBetween(const Gaussian &a, const Gaussian &center, const Gaussian &b)
{
    float x1 = a.x - center.x, y1 = a.y - center.y,
          x2 = b.x - center.x, y2 = b.y - center.y,
          dot = x1*x2 + y1*y2,      // dot product
          det = x1*y2 - y1*x2;      // determinant
    return atan2(det, dot)*(180/M_PI);
}
