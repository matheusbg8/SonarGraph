#ifndef SONARCONFIG_H
#define SONARCONFIG_H

#include <string>
#include "Enuns.h"
#include <map>
#include<cstdio>
using namespace std;


class SonarConfig
{
private:
    map<string, pair<string,void*> > valueMaps;
    void createValueMaps();
    bool extractLine(FILE *f, char *str);

    void writeString(FILE*file, void *v);
    void writeBool(FILE*file, void *v);
    void writeFloat(FILE*file, void *v);
    void writeDouble(FILE*file, void *v);
    void writeInt(FILE*file, void *v);
    void writeUnsigned(FILE*file, void *v);
    void writeUnsignedChar(FILE*file, void *v);
    void writeUnsignedShort(FILE*file, void *v);
    void writeGraphCreatorMode(FILE*file, void *v);

    string readString(const char*str);
    bool readBool(const char*str);
    float readFloat(const char*str);
    double readDouble(const char*str);
    int readInt(const char*str);
    unsigned readUnsigned(const char*str);
    unsigned char readUnsignedChar(const char *str);
    unsigned short readUnsignedShort(const char *str);
    GraphCreatorMode readGraphCreatorMode(const char *str);

public:
    string name;

    // Sonar Config.
    bool sonar_computeHistogranThreshold;
    bool sonar_computeMaxPixel;
    bool sonar_maxPixel;

    float sonar_stdDevMultiply; // Number of times we multiply stdDev to work
    float sonar_graphLinkDistance; // Distance to link two vertex
    float sonar_graphNeigborDistanceRelativeLink; // Distance to link two vertex
    GraphCreatorMode sonar_graphCreatorMode;

    unsigned short sonar_pixelThreshold; // Threshold for pixel search (start floodfill) on 8bits normalized image
    unsigned short sonar_searchThreshold; // Threshold to floodfill search on 16bits image
    unsigned sonar_minSampleSize; // Minimun amount of pixel to create a vertex
    unsigned sonar_maxSampleSize;

    bool sonar_drawPixelFound;
    bool sonar_drawElipses;
    bool sonar_drawElipsesAngle;
    bool sonar_drawEdges;
    bool sonar_drawEdgesAngle;
    bool sonar_drawEdgesInvAngle;
    bool sonar_drawVertexID;
    bool sonar_drawStdDevValue;

    bool sonar_saveGraphImg;
    bool sonar_saveTruncateImg;
    bool sonar_saveMatchImgs;

    // GraphMatch Config.
    float graphMatcher_distThreshold;
    float graphMatcher_angThreshold;

    float graphMatcher_stdXWeightError;
    float graphMatcher_stdYWeightError;
    float graphMatcher_stdIWeightError;
    float graphMatcher_angWeightError;
    float graphMatcher_invAngWeightError;
    float graphMatcher_edgeDistanceWeightError;

    bool graphMatcher_drawVertexMatching;

    SonarConfig(const char *fileName=0x0);

    bool load(const char *fileName);
    bool save2(const char *fileName);
    bool save(const char *fileName);

};

/*
// Some tests
#include "SonarConfig.h"

int main()
{
    SonarConfig sc;

    FILE *f = fopen("teste.txt", "r");
    char line[1000];
    while(sc.extractLine(f, line))
    {
          printf("%s\n", line);
    }
    return 0;
}
*/

/*
// more tests
#include "SonarConfig.h"

int main(int argc, char* argv[])
{
    SonarConfig sc;
    sc.load("DummyConfig.txt");
    sc.save("TestConfig.txt");
    sc.save2("TestConfig2.txt");
    return 0;
}
*/

#endif // SONARCONFIG_H
