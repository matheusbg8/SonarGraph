#include "CloseLoopTester.h"

CloseLoopTester::CloseLoopTester(const string &datasetPath, unsigned jump):
    datasetPath(datasetPath), jump(jump)
{

}

bool CloseLoopTester::loadFrames(const char *fileName)
{
    FILE *f_img_names = fopen((datasetPath + fileName).c_str(),"r");
    char img_file_name[301];

    if(f_img_names == 0x0)
    {
        cout << "Image list file " << fileName << " not found!!" << endl;
        return false;
    }

    unsigned frame=0,size;
    while(fgets(img_file_name, 300, f_img_names) !=NULL)
    {
        size = strlen(img_file_name);
        if(size > 0)
        {
            img_file_name[size-1] = '\0';
            frames.push_back(Frame(string(img_file_name),frame++));
        }
    }
    fclose(f_img_names);

    return true;
}

void CloseLoopTester::describeFrames()
{
    // Creating Sonar objects to describe frames
    ConfigLoader config("../SonarGaussian/Configs.ini");
    Sonar sonar(config,false);
    sonar.storeImgs = false;
    sonar.drawPixelFound = false;

    FILE *f = fopen((datasetPath + "Results/ResultFramesInformations.csv").c_str(), "w");

    if(f)
        cout << "Saving results in " << (datasetPath + "Results/LoopDetections/ResultFramesInformations.csv") << endl;
    else
        cout << "It was not possible to open write on file " << (datasetPath + "Results/LoopDetections/ResultFramesInformations.csv")
             << endl;

    fprintf(f,"# Frame ID, Amount of vertices, Amount of edges\n");

    sd.resize(frames.size());

    for(unsigned i = 0 ;i < frames.size() ; i+=jump)
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

void CloseLoopTester::saveDescriptions(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:saveGroundTruth:: use!" << endl;
    #endif

    setlocale (LC_ALL,"C");
    FILE *f = fopen(fileName, "w");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned frame=0, g;

    fprintf(f,"%u\n", frames.size());
    for(frame = 0 ; frame < frames.size() ; frame++)
    {
        Frame &fr = frames[frame];
        SonarDescritor *sdi = sd[frame];

        // Frame description
        // Image filename frameID NumberOfGaussians
        fprintf(f,"%s %u %u\n", fr.fileName.c_str() , frame, sdi->gaussians.size());

        // Gaussians Description
        vector<Gaussian> &gs = sdi->gaussians;
        if(gs.size() == 0) continue;

        for(g = 0 ; g < gs.size(); g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fprintf(f,"%d %f %f %f %f %f %f %f %u\n", g, ga.x, ga.y , ga.intensity, ga.dx , ga.dy, ga.di, ga.ang , ga.N );
        }

        // Graph description
        vector<vector<GraphLink*> > &graph = sdi->graph;
        for(unsigned i = 0; i < graph.size() ; i++)
        {
            for(unsigned j = 0 ; j < graph[i].size() ; j++)
            {
                GraphLink *link = graph[i][j];
                // float ang, p , rang, invang, int dest
                fprintf(f,"%f %f %f %f %d\n", link->ang, link->p, link->rAng, link->invAngle, link->dest);
            }
        }
    }
    fclose(f);
}

void CloseLoopTester::loadDescriptions(const char *fileName)
{
    #ifdef GROUND_TRUTH_TRACKING_DEBUG
        cout << "GroundTruth:saveGroundTruth:: use!" << endl;
    #endif

    setlocale (LC_ALL,"C");
    FILE *f = fopen(fileName, "r");
    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned nFrames, nGaussians;
    char str[400];

    fscanf(f,"%u\n", &nFrames);
    frames.resize(nFrames,Frame("",0));
    sd.resize(nFrames);

    for(unsigned frame = 0 ; frame < nFrames ; frame++)
    {
        Frame &fr = frames[frame];
        SonarDescritor *sdi = sd[frame];

        // Frame description
        // Image filename frameID NumberOfGaussians
        fscanf(f,"%s %*u %u\n", str , &nGaussians);
        fr.fileName = str;

        if(nGaussians == 0) continue;

        // Gaussians Description
        vector<Gaussian> &gs = sdi->gaussians;
        gs.resize(nGaussians);

        for(unsigned g = 0 ; g < nGaussians; g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Z , DX, DY, DZ, Ang, N
            fscanf(f,"%*d %f %f %f %f %f %f %f %u\n", &ga.x, &ga.y , &ga.intensity, &ga.dx,
                    &ga.dy, &ga.di, &ga.ang , &ga.N );
        }

        // Graph description
        vector<vector<GraphLink*> > &graph = sdi->graph;
        for(unsigned i = 0; i < graph.size() ; i++)
        {
            for(unsigned j = 0 ; j < graph[i].size() ; j++)
            {
                GraphLink *link = graph[i][j];
                // float ang, p , rang, invang, int dest
                fscanf(f,"%f %f %f %f %d\n", &link->ang, &link->p, &link->rAng, &link->invAngle, &link->dest);
            }
        }
    }
    fclose(f);
}

void CloseLoopTester::computeMatchs(unsigned windowJump, unsigned start, unsigned end)
{
    cout << "Computing matchs" << endl;
    ConfigLoader config("../SonarGaussian/Configs.ini");
    GraphMatcher gm(config);
    char str[300];

    if(end == 0) end = sd.size();
    windowJump =  windowJump - windowJump%jump;
    for(unsigned u =start - start%jump ; u < end ; u+=jump)
    {
        cout << "Processing frame " << u << " until " << end << " " << jump << " in " << jump << endl;
        sprintf(str,(datasetPath + "Results/LoopDetections/MatchResults_fr%04u.csv").c_str(),u);
        FILE *f = fopen(str, "r");
        if(f != 0x0)
        {
            cout << "Already computed!!" << endl;
            fclose(f);
            continue;
        }
        f = fopen(str, "w");

        fprintf(f,"#Src frame ID, Dst frame ID, Amout of similar vertex found between the frames\n");
        for(unsigned v = u + windowJump+jump ; v < sd.size() ; v+=jump)
        {
            vector<MatchInfo> vertexMatch;
            gm.findMatch(sd[u],sd[v],vertexMatch);
            fprintf(f,"%u,%u,%lu\n",u,v,vertexMatch.size());
        }
        fclose(f);
    }
}
