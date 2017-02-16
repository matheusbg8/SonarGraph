#ifndef CLOSELOOPANALISERESULT_H
#define CLOSELOOPANALISERESULT_H

#include <vector>
#include <string>
using namespace std;

#include "GraphMatcher/MatchInfo/MatchInfoWeighted.h"

class CloseLoopAnaliseResult
{
private:
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<unsigned,float> PUF;

    vector< vector< PUF > > resultGraph; // Matchs found, adjacence graph, first = dest frame ID , second = score (N vertex matche betwen uFr and vFr)
    vector< PUU > frResults;// Described frames statistics, Firs=N vertexs, Second= N edges

    vector< vector< PUF > > gtGraph; // adjacence graph, first = dest frame ID , second = score (nomalized distance until img center)

    string destPath;

    void loadResultFrameInformations(const char *fileName= "Results/ResultFramesInformations.csv");
    void loadResultOfMatch(const char *prefix="Results/LoopDetections/MatchResults_fr");
    void loadDirectResult(const char *csvFileName, unsigned nFrames);

    void loadGtMatch(const char *gtFileName="GTMatchs.csv");

    void normalizeResult();
    void normalizeResult2(unsigned maxValue);

    float findGtScore(unsigned uFr, unsigned vFr);

public:

    CloseLoopAnaliseResult(const char *destPath);

    void saveResults(const char *fileName = "Results/ResultFramesInformations.csv");
    void saveSplitedGTResults(unsigned uFr);
    void saveSplitedGTResults();

    void loadAnalisyAndSave();

    void saveAnalisyResults(const char *fileName= "CompareResult.csv");

    void splitGTResults();

    void generateGTImage();
    void generateResultImage();
    void generatePairFeaturesCount();

};

#endif // CLOSELOOPANALISERESULT_H
