#include "MapPloter.h"
#include "Sonar/SonarConfig/ConfigLoader.h"
#include <iostream>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>


MapPloter::MapPloter()
{
    resetColors(COLORMAP_JET);
}

void MapPloter::loadMap(const string &mapName)
{
    ConfigLoader config(mapName.c_str());

    string str;

    if(config.getString("General", "MapName", &str))
    {
        cout << "MapName " << str << endl;
        mapImg = imread(str);

        if(mapImg.rows == 0)
        {
            cout << "Error when loading map " << str << endl;
            assert("Problem! Image map not found");
        }
        clearImg();
    }

    if(config.getString("P_1", "img_x_y", &str))
    {
        cout << "P_1: img_x_y " << str << endl;
        parse2DPoint(str,imgRef[0]);
    }

    if(config.getString("P_1", "utm_x_y", &str))
    {
        cout << "P_1: utm_x_y " << str << endl;
        parse2DPoint(str,UTMRef[0]);
    }

    if(config.getString("P_2", "img_x_y", &str))
    {
        cout << "P_2: img_x_y " << str << endl;
        parse2DPoint(str,imgRef[1]);
    }

    if(config.getString("P_2", "utm_x_y", &str))
    {
        cout << "P_2: utm_x_y " << str << endl;
        parse2DPoint(str,UTMRef[1]);
    }

    Point2f diffImg = imgRef[1] - imgRef[0],
            diffUTM = UTMRef[1] - UTMRef[0];

    // Compute conversion for small and initial square
    UTM2Img_.x = diffImg.x/diffUTM.x;
    UTM2Img_.y = diffImg.y/diffUTM.y;

    printf("imgRef1 %f %f\n", imgRef[0].x , imgRef[0].y);
    printf("imgRef2 %f %f\n", imgRef[1].x , imgRef[1].y);
    printf("utmRef1 %f %f\n", UTMRef[0].x , UTMRef[0].y);
    printf("utmRef2 %f %f\n", UTMRef[1].x , UTMRef[1].y);
    printf("utmDiff %f %f\n", diffUTM.x , diffUTM.y);
    printf("imgDiff %f %f\n", diffImg.x , diffImg.y);

    printf("utm2img %f %f\n", UTM2Img_.x , UTM2Img_.y);

    cout << "Map loaded!" << endl;
}

Point2f MapPloter::UTM2Img(const Point2f &UTMp)
{
    Point2f p(imgRef[0].x + (UTMp.x - UTMRef[0].x)*UTM2Img_.x,
              imgRef[0].y + (UTMp.y - UTMRef[0].y)*UTM2Img_.y);
    return p;
}

void MapPloter::parse2DPoint(const string &str, Point2f &p)
{
    double x,y;
    sscanf(str.c_str(),"%lf %lf", &x, &y);
    p.x =x;
    p.y =y;
}

void MapPloter::drawLine(const Point2f &p1, const Point2f &p2,
                         const Scalar &color)
{
    line(screen,UTM2Img(p1),UTM2Img(p2),color,10);
}

void MapPloter::drawLine(const Point2f &p1, const Point2f &p2, float colorPercent)
{
    // colorPercent need to be between 0 and 1 .
    int colorId = round(colorPercent*100)*2.55f;
    Vec3b &c = colors.at<Vec3b>(0,colorId);

    line(screen,UTM2Img(p1),UTM2Img(p2),
         Scalar(c[0],c[1],c[2]),10);
}

void MapPloter::clearImg()
{
    screen = mapImg.clone();
}

void MapPloter::showImg()
{
    namedWindow("MapPloter",WINDOW_NORMAL);
    imshow("MapPloter", screen);

}

void MapPloter::saveImg(const string &imagName)
{
    imwrite(imagName,screen);
    Mat c;
    resize(colors,c,Size(1200,60),0,0,INTER_NEAREST);
    imwrite("Colormap.png",c);
}

void MapPloter::resetColors(int colorMap)
{
    colors = Mat(1,255, CV_8UC1);
    for(unsigned i = 0 ; i < 256; i++)
    {
        colors.at<uchar>(0,i) = i;
    }
    applyColorMap(colors,colors,colorMap);
}
