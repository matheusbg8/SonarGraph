#include "CloseLoopTester.h"

CloseLoopTester::CloseLoopTester()
{

}

bool CloseLoopTester::loadFrames(const char *fileName)
{
    FILE *f_img_names = fopen(fileName,"r");
    char img_file_name[300];

    if(f_img_names == 0x0)
    {
        cout << "Image list file " << fileName << " not found!!" << endl;
        return false;
    }

    unsigned frame=0;
    while( fscanf(f_img_names,"%s", img_file_name) != -1)
    {
        frames.push_back(Frame(string(img_file_name),frame++));
    }
    fclose(f_img_names);

    return true;
}

void CloseLoopTester::describeFrames()
{
    // Loading sonar config
    SonarConfig sonarConfig;
    sonarConfig.load("../SonarGaussian_02_2016_r114/MachadosConfig");

    // Creating Sonar objects to describe frames
    Sonar sonar(sonarConfig,false);
    sonar.storeImgs = false;
    sonar.drawPixelFound = false;

    FILE *f = fopen("LoopDetections/ResultFramesInformations.csv", "w");

    sd.resize(frames.size());
    for(unsigned i = 0 ;i < frames.size() ; i++)
    {
        unsigned frNumber = frames[i].frameNumber;
        cout << "Describing frame " << frNumber << endl;
        Mat img = imread(frames[i].fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
        SonarDescritor *csd = sonar.newImageDirect(img);
        sd[i] = csd;

        fprintf(f, "%u,%lu,%u\n",i,csd->gaussians.size(),csd->numberOfEdges()/2);
    }
    fclose(f);
}

void CloseLoopTester::computeMatchs(unsigned windowJump,unsigned start, unsigned end)
{
    cout << "Computing matchs" << endl;
    GraphMatcher gm;
    char str[300];

    if(end == 0) end = sd.size();


    for(unsigned u =start ; u < end ; u++)
    {
        sprintf(str,"LoopDetections/MatchResults_fr%04u.csv",u);
        FILE *f = fopen(str, "w");
        cout << "Processing frame " << u << " until " << end << endl;
        for(unsigned v = u + windowJump+1 ; v < sd.size() ; v++)
        {
            vector<MatchInfo> vertexMatch;
            gm.matcheSIFTCut(sd[u],sd[v],vertexMatch);
            fprintf(f,"%u,%u,%lu\n",u,v,vertexMatch.size());
        }
        fclose(f);
    }
}
