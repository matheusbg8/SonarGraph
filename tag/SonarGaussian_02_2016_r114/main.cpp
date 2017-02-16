#include <iostream>
#include <clocale>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Sonar/Sonar.h"
#include "Sonar/SonarConfig.h"
#include "GroundTruth/GroundTruth.h"
#include "GraphMatcher/GraphMatcher.h"
#include "MatchViewer.h"

#include "CSVMatrixGenerator.h"

using namespace std;
using namespace cv;


//// Edge Matching Test
//int main(int argc, char* argv[])
//{
//    GraphMatcher gm;

//    vector<GraphLink *> u,v;

//    u.push_back(new GraphLink(0,0,1,10,1));
//    u.push_back(new GraphLink(0,0,1,50,2));
//    u.push_back(new GraphLink(0,0,1,30,3));
//    u.push_back(new GraphLink(0,0,1,10,4));
//    u.push_back(new GraphLink(0,0,1,20,5));

//    v.push_back(new GraphLink(0,0,1,10,1));
//    v.push_back(new GraphLink(0,0,1,11,2));
//    v.push_back(new GraphLink(0,0,1,30,3));
//    v.push_back(new GraphLink(0,0,1,50,4));
//    v.push_back(new GraphLink(0,0,1,15,5));
//    v.push_back(new GraphLink(0,0,1,20,6));

//    gm.byDistanceCompare(u,v);

//    return 0;
//}


int main(int argc, char* argv[])
{
//    setlocale (LC_ALL,"C");
//    std::setlocale (LC_ALL,"C");
//    std::locale::global(std::locale("C"));
//     setlocale(LC_ALL, "en_US.UTF-8");
//    setlocale(LC_NUMERIC, "C");
//    setlocale(LC_ALL, "en_US.UTF-8");
//    setlocale(LC_NUMERIC, "de_DE");
//    setlocale(LC_TIME, "ja_JP");
//    setlocale(LC_ALL, "en_US");
//    wchar_t str[100];
//    time_t t = time(NULL);
//    wcsftime(str, 100, L"%A %c", localtime(&t));
//    wprintf(L"Number: %.2f\nDate: %Ls\n", 3.14, str);

  //  return 0;

    MatchViewer mv;
    cout << "OpenCV version " << CV_VERSION << endl;

//    mv.testWindowTool();

//    mv.compareWithGroundTruth();

//    mv.compareWithGorundTruth2();

    mv.createGroudTruth();

//    mv.standardExecution();

//    mv.matchTest();

//    mv.gaussianTest();

//    mv.generateMatlabMatrix("../../GroundTruth/Yacht_05_12_2014.txt");

//    if(argc > 1 && strcmp(argv[1], "CLD")==0)
//    {
//        if(argc > 3)
//            mv.closeLoopDetection(atoi(argv[2]), atoi(argv[3]));
//        else
//            mv.closeLoopDetection();
//    }

//    if(argc > 1 && strcmp(argv[1], "CLAR")==0)
//    {
//        mv.closeLoopAnaliseResult();
//    }

//    cout << "End of main" << endl;

    return 0;
}

//int main(int argc, char* argv[])
//{
//    Mat img = imread(argv[1],CV_LOAD_IMAGE_ANYDEPTH);
//    img.convertTo(img,CV_8UC1);
//    imshow("img", img);
//    waitKey();
//    return 0;
//}



//int main(int argc, char* argv[])
//{
//    Mat m1, m2;

//    for(unsigned i = 0 ; i < 50 ; i++)
//    {
//        char tmpTxt[200];
//        sprintf(tmpTxt, "frame_%0.3d.png",i);
//        m1 = imread(tmpTxt); // red
//        sprintf(tmpTxt, "frame_%0.3d_TR.png",i);
//        m2 = imread(tmpTxt); // green

//        for(unsigned j = 0 ; j < m1.rows ; j++)
//        {
//            for(unsigned k = 0 ; k < m1.cols; k++)
//            {
//                unsigned pixelCol = k*3;
//                if(m2.at<uchar>(j,pixelCol  ) == 0  &&
//                   m2.at<uchar>(j,pixelCol+1) == 255&&
//                   m2.at<uchar>(j,pixelCol+2) == 0 )
//                {
//                    if(m1.at<uchar>(j,pixelCol  ) == 0 &&
//                       m1.at<uchar>(j,pixelCol+1) == 0 &&
//                       m1.at<uchar>(j,pixelCol+2) == 255)
//                    {
//                        m1.at<uchar>(j,pixelCol  ) = 255;
//                        m1.at<uchar>(j,pixelCol+2) = 0;
//                    }else
//                    {
//                        m1.at<uchar>(j,pixelCol+1) = 255;
//                        m1.at<uchar>(j,pixelCol+2) = 0;
//                    }
//                }
//            }
//        }
//        sprintf(tmpTxt, "frame_%0.3d_merg.png",i);
//        imwrite(tmpTxt,m1);
//        imshow("result" , m1);
//        waitKey();


//    }

//    return 0;
//}


//int main(int argc, char* argv[])
//{
//    CSVMatrixGenerator csvGen;

////    csvGen.generate("../../GroundTruth/Yacht_05_12_2014_denso.txt");
//    csvGen.generate("../../GroundTruth/Yacht_05_12_2014.txt");

//    cout << "End of main" << endl;

//    return 0;
//}

