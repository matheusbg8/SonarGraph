#include "SonarDescritorTester.h"
#include "GraphMatcher/MatchInfo/MatchInfo.h"

SonarDescritorTest::SonarDescritorTest()
{

}

void SonarDescritorTest::matchDescriptor(vector<Point2f> &srcMatch, vector<Point2f> &destMatch,
                                         Mat &imgGray16_1, Mat &imgGray16_2)
{
    Mat result1, result2;
    SonarDescritor *sd1 = 0x0,
                   *sd2 = 0x0;

    sd1 = sn.newImageDebug(imgGray16_1,result1);
    sd2 = sn.newImageDebug(imgGray16_2,result2);


    vector<MatchInfo> matchs;

    gm.findMatch(sd1,sd2,matchs);

    vector<Gaussian> &gs1 = sd1->gaussians,
                     &gs2 = sd2->gaussians;

    for(unsigned i = 0 ; i < matchs.size() ; i++)
    {
        Gaussian &g1 = gs1[matchs[i].uID],
                 &g2 = gs2[matchs[i].vID];

        srcMatch.push_back(Point2f(g1.x, g1.y));
        destMatch.push_back(Point2f(g2.x, g2.y));
    }

    imshow("Descriptor1", result1);
    imshow("Descriptor2", result2);
}
