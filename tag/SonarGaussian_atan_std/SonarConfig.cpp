#include "SonarConfig.h"
#include "Sonar.h"

#include <cstdio>

typedef pair<string,void*> psv;

void SonarConfig::createValueMaps()
{
    valueMaps["name"] = psv("string",&name);

    // Sonar Config.
    valueMaps["sonar_computeHistogranThreshold"] = psv("bool",&sonar_computeHistogranThreshold);
    valueMaps["sonar_computeMaxPixel"] = psv("bool",&sonar_computeMaxPixel);
    valueMaps["sonar_maxPixel"] = psv("bool",&sonar_maxPixel);

    valueMaps["sonar_stdDevMultiply"] = psv("float",&sonar_stdDevMultiply);
    valueMaps["sonar_graphLinkDistance"] = psv("float",&sonar_graphLinkDistance);
    valueMaps["sonar_graphNeigborDistanceRelativeLink"] = psv("float",&sonar_graphNeigborDistanceRelativeLink);
    valueMaps["sonar_graphCreatorMode"] = psv("GraphCreatorMode",&sonar_graphCreatorMode);

    valueMaps["sonar_pixelThreshold"] = psv("unsigned short",&sonar_pixelThreshold);
    valueMaps["sonar_searchThreshold"] = psv("unsigned short",&sonar_searchThreshold);
    valueMaps["sonar_minSampleSize"] = psv("unsigned",&sonar_minSampleSize);
    valueMaps["sonar_maxSampleSize"] = psv("unsigned",&sonar_maxSampleSize);

    valueMaps["sonar_drawPixelFound"] = psv("bool",&sonar_drawPixelFound);
    valueMaps["sonar_drawElipses"] = psv("bool",&sonar_drawElipses);
    valueMaps["sonar_drawElipsesAngle"] = psv("bool",&sonar_drawElipsesAngle);
    valueMaps["sonar_drawEdges"] = psv("bool",&sonar_drawEdges);
    valueMaps["sonar_drawEdgesAngle"] = psv("bool",&sonar_drawEdgesAngle);
    valueMaps["sonar_drawEdgesInvAngle"] = psv("bool",&sonar_drawEdgesInvAngle);
    valueMaps["sonar_drawVertexID"] = psv("bool",&sonar_drawVertexID);
    valueMaps["sonar_drawStdDevValue"] = psv("bool",&sonar_drawStdDevValue);

    valueMaps["sonar_saveGraphImg"] = psv("bool",&sonar_saveGraphImg);
    valueMaps["sonar_saveTruncateImg"] = psv("bool",&sonar_saveTruncateImg);
    valueMaps["sonar_saveMatchImgs"] = psv("bool",&sonar_saveMatchImgs);

    valueMaps["graphMatcher_distThreshold"] = psv("float",&graphMatcher_distThreshold);
    valueMaps["graphMatcher_angThreshold"] = psv("float",&graphMatcher_angThreshold);

    valueMaps["graphMatcher_stdXWeightError"] = psv("float",&graphMatcher_stdXWeightError);
    valueMaps["graphMatcher_stdYWeightError"] = psv("float",&graphMatcher_stdYWeightError);
    valueMaps["graphMatcher_stdIWeightError"] = psv("float",&graphMatcher_stdIWeightError);
    valueMaps["graphMatcher_angWeightError"] = psv("float",&graphMatcher_angWeightError);
    valueMaps["graphMatcher_invAngWeightError"] = psv("float",&graphMatcher_invAngWeightError);
    valueMaps["graphMatcher_edgeDistanceWeightError"] = psv("float",&graphMatcher_edgeDistanceWeightError);

    valueMaps["graphMatcher_drawVertexMatching"] = psv("bool",&graphMatcher_drawVertexMatching);
}

bool SonarConfig::extractLine(FILE *f, char *str)
{
    bool comment=false;
    *str = getc(f);
    while(*str != EOF && *str != '\n')
    {
        if(*str == '#')
            comment = true;

        if(!comment)
            str++;

        *str = getc(f);
    }
    if(*str == EOF)
    {
        *str = '\0';
        return false;
    }
    *str = '\0';
    return true;
}

void SonarConfig::writeString(FILE *file, void *v)
{
    fprintf(file, "%s", (*((string*)v)).c_str());
}

void SonarConfig::writeBool(FILE *file, void *v)
{
    if(*((bool*)v))
        fprintf(file, "1");
    else
        fprintf(file, "0");
}

void SonarConfig::writeFloat(FILE *file, void *v)
{
    fprintf(file, "%f", *((float*)v));
}

void SonarConfig::writeDouble(FILE *file, void *v)
{
    fprintf(file, "%Lf", *((double*)v));
}

void SonarConfig::writeInt(FILE *file, void *v)
{
    fprintf(file, "%d", *((int*)v));
}

void SonarConfig::writeUnsigned(FILE *file, void *v)
{
    fprintf(file, "%u", *((unsigned*)v));
}

void SonarConfig::writeUnsignedChar(FILE *file, void *v)
{
    fprintf(file, "%hhu", *((unsigned char*)v));
}

void SonarConfig::writeUnsignedShort(FILE *file, void *v)
{
    fprintf(file, "%hu", *((unsigned short*)v));
}

void SonarConfig::writeGraphCreatorMode(FILE *file, void *v)
{
    GraphCreatorMode mode = *((GraphCreatorMode*)v);
    switch(mode)
    {
    case FIXED_DISTANCE:
        fprintf(file, "FIXED_DISTANCE");
    break;
    case CLOSEST_NEIGHBOR_RELATIVE_DISTANCE:
        fprintf(file, "CLOSEST_NEIGHBOR_RELATIVE_DISTANCE");
    break;
    }
}

string SonarConfig::readString(const char *str)
{
    return string(str);
}

bool SonarConfig::readBool(const char *str)
{
    int v;
    sscanf(str, "%d", &v);
    return v;
}

float SonarConfig::readFloat(const char *str)
{
    float v;
    sscanf(str, "%f", &v);
    return v;
}

double SonarConfig::readDouble(const char *str)
{
    double v;
    sscanf(str, "%lf", &v);
    return v;
}

int SonarConfig::readInt(const char *str)
{
    int v;
    sscanf(str, "%d", &v);
    return v;
}

unsigned SonarConfig::readUnsigned(const char*str)
{
    unsigned v;
    sscanf(str, "%u", &v);
    return v;
}

unsigned char SonarConfig::readUnsignedChar(const char *str)
{
    unsigned char v;
    sscanf(str, "%hhu", &v);
    return v;
}

unsigned short SonarConfig::readUnsignedShort(const char *str)
{
    unsigned short v;
    sscanf(str, "%hu", &v);
    return v;
}

GraphCreatorMode SonarConfig::readGraphCreatorMode(const char *str)
{
    string sstr(str);
    if(sstr == "CLOSEST_NEIGHBOR_RELATIVE_DISTANCE")
        return CLOSEST_NEIGHBOR_RELATIVE_DISTANCE;
    if(sstr == "FIXED_DISTANCE")
        return FIXED_DISTANCE;
}


SonarConfig::SonarConfig():
    name("Default"),
    // Sonar Config.
    sonar_computeHistogranThreshold(false),
    sonar_computeMaxPixel(false),
    sonar_maxPixel(1400),

    sonar_stdDevMultiply(5.f), // Number of times we multiply stdDev to work
    sonar_graphLinkDistance(800.f), // Distance to link two vertex
    sonar_graphNeigborDistanceRelativeLink(30.f), // Distance to link two vertex
    sonar_graphCreatorMode(FIXED_DISTANCE),

    sonar_pixelThreshold(30), // Threshold for pixel search (start floodfill) on 8bits normalized image
    sonar_searchThreshold(200), // Threshold to floodfill search on 16bits image
    sonar_minSampleSize(10), // Minimun amount of pixel to create a vertex
    sonar_maxSampleSize(150000),

    sonar_drawPixelFound(true),
    sonar_drawElipses(true),
    sonar_drawElipsesAngle(true),
    sonar_drawEdges(true),
    sonar_drawEdgesAngle(false),
    sonar_drawEdgesInvAngle(false),
    sonar_drawVertexID(true),
    sonar_drawStdDevValue(false),

    sonar_saveGraphImg(false),
    sonar_saveTruncateImg(false),
    sonar_saveMatchImgs(false),

    // GraphMatch Config.
    graphMatcher_distThreshold(25.f),
    graphMatcher_angThreshold(30.f),

    graphMatcher_stdXWeightError(0.2f),
    graphMatcher_stdYWeightError(0.2f),
    graphMatcher_stdIWeightError(0.2f),
    graphMatcher_angWeightError(0.1f),
    graphMatcher_invAngWeightError(0.2f),
    graphMatcher_edgeDistanceWeightError(0.1f),

    graphMatcher_drawVertexMatching(false)
{
    createValueMaps();
}

bool SonarConfig::load(const char *fileName)
{
    /// @todo Handle blank line
    /// @todo Handle blank space

    FILE *f = fopen(fileName, "r");
    map<string, psv>::iterator mi;

    char line[2000], varName[400];
    int p;
    while(extractLine(f,line))
    {
        if(strlen(line) <= 3) continue;
        sscanf(line, "%s", varName);
        p = strlen(varName)+1; // jumo space between value name and value

        mi = valueMaps.find(varName);
        if(mi == valueMaps.end())
        {
            printf("Error to load sonar config file, value %s not found\n",varName);
            continue;
        }

        if(mi->second.first == "string")
        {
            *((string*)mi->second.second) = readString(line+p);
        }
        else if(mi->second.first == "bool")
        {
            *((bool*)mi->second.second) = readBool(line+p);
        }
        else if(mi->second.first == "float")
        {
            *((float*)mi->second.second) = readFloat(line+p);
        }
        else if(mi->second.first == "double")
        {
            *((double*)mi->second.second) = readDouble(line+p);
        }
        else if(mi->second.first == "int")
        {
            *((int*)mi->second.second) = readInt(line+p);
        }
        else if(mi->second.first == "unsigned")
        {
            *((unsigned*)mi->second.second) = readUnsigned(line+p);
        }
        else if(mi->second.first == "unsigned char")
        {
            *((unsigned char*)mi->second.second) = readUnsignedChar(line+p);
        }
        else if(mi->second.first == "unsigned short")
        {
            *((unsigned short*)mi->second.second) = readUnsignedShort(line+p);
        }
        else if(mi->second.first == "GraphCreatorMode")
        {
            *((GraphCreatorMode*)mi->second.second) = readGraphCreatorMode(line+p);
        }
    }

    fclose(f);
}

bool SonarConfig::save2(const char *fileName)
{
    FILE *f = fopen(fileName, "w");
    if(f == 0x0) return false;

    fprintf(f,"name %s\n", name.c_str());

    fprintf(f,"sonar_computeHistogranThreshold %d\n", sonar_computeHistogranThreshold);
    fprintf(f,"sonar_computeMaxPixel %d\n",sonar_computeMaxPixel);
    fprintf(f,"sonar_maxPixel %d\n",sonar_maxPixel);

    fprintf(f,"sonar_stdDevMultiply %f\n",sonar_stdDevMultiply);
    fprintf(f,"sonar_graphLinkDistance %f\n",sonar_graphLinkDistance);
    fprintf(f,"sonar_graphNeigborDistanceRelativeLink %f\n",sonar_graphNeigborDistanceRelativeLink);
    fprintf(f,"sonar_graphCreatorMode ");
    switch(sonar_graphCreatorMode)
    {
    case FIXED_DISTANCE: fprintf(f,"FIXED_DISTANCE\n");
    break;
    case CLOSEST_NEIGHBOR_RELATIVE_DISTANCE:fprintf(f,"CLOSEST_NEIGHBOR_RELATIVE_DISTANCE\n");
    break;
    }

    fprintf(f,"sonar_pixelThreshold %u\n",sonar_pixelThreshold);
    fprintf(f,"sonar_searchThreshold %u\n",sonar_searchThreshold);
    fprintf(f,"sonar_minSampleSize %u\n",sonar_minSampleSize);
    fprintf(f,"sonar_maxSampleSize %u\n",sonar_maxSampleSize);

    fprintf(f,"sonar_drawPixelFound %d\n",sonar_drawPixelFound);
    fprintf(f,"sonar_drawElipses %d\n", sonar_drawElipses);
    fprintf(f,"sonar_drawElipsesAngle %d\n",sonar_drawElipsesAngle);


    fprintf(f,"sonar_drawEdges %d\n",sonar_drawEdges);
    fprintf(f,"sonar_drawEdgesAngle %d\n",sonar_drawEdgesAngle);
    fprintf(f,"sonar_drawEdgesInvAngle %d\n",sonar_drawEdgesInvAngle);
    fprintf(f,"sonar_drawVertexID %d\n",sonar_drawVertexID);
    fprintf(f,"sonar_drawStdDevValue %d\n",sonar_drawStdDevValue);


    fprintf(f,"graphMatcher_distThreshold %f\n",graphMatcher_distThreshold);
    fprintf(f,"graphMatcher_angThreshold %f\n",graphMatcher_angThreshold);

    fprintf(f,"graphMatcher_stdXWeightError %f\n",graphMatcher_stdXWeightError);
    fprintf(f,"graphMatcher_stdYWeightError %f\n",graphMatcher_stdYWeightError);
    fprintf(f,"graphMatcher_stdIWeightError %f\n",graphMatcher_stdIWeightError);
    fprintf(f,"graphMatcher_angWeightError %f\n",graphMatcher_angWeightError);
    fprintf(f,"graphMatcher_invAngWeightError %f\n",graphMatcher_invAngWeightError);
    fprintf(f,"graphMatcher_edgeDistanceWeightError %f\n",graphMatcher_edgeDistanceWeightError);

    fprintf(f,"graphMatcher_drawVertexMatching %d\n",graphMatcher_drawVertexMatching);

    fclose(f);
}

bool SonarConfig::save(const char *fileName)
{
    FILE *f = fopen(fileName, "w");
    if(f == 0x0) return false;

    map<string, psv>::iterator mi;
    for(mi = valueMaps.begin(); mi != valueMaps.end(); mi++)
    {
        fprintf(f,"%s ", mi->first.c_str());

        if(mi->second.first == "string")
        {
            writeString(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "bool")
        {
            writeBool(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "float")
        {
            writeFloat(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "double")
        {
            writeDouble(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "int")
        {
            writeInt(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "unsigned")
        {
            writeUnsigned(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "unsigned char")
        {
            writeUnsignedChar(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "unsigned short")
        {
            writeUnsignedShort(f,mi->second.second);
            putc('\n',f);
        }
        else if(mi->second.first == "GraphCreatorMode")
        {
            writeGraphCreatorMode(f,mi->second.second);
            putc('\n',f);
        }
    }
    fclose(f);
}
