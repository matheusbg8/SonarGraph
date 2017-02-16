#include "Gaussian.h"

#include <iostream>
using namespace std;


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
    float abx = b.x-a.x,
          aby = b.y - a.y,
          acx = c.x - a.x,
          acy = c.y-a.y,
    r = abx * acy - aby * acx;

    return r;

//    return (b.x-a.x) * (c.y-a.y) - (c.x - a.x) * (b.y - a.y);
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
