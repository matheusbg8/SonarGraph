#ifndef CLOSELOOPANALISERESULT_H
#define CLOSELOOPANALISERESULT_H

#include <vector>
using namespace std;

class CloseLoopAnaliseResult
{
private:
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<unsigned,float> PUF;

    vector< vector< PUF > > rGraph; // adjacence graph, first = dest frame ID , second = score (N vertex matche betwen uFr and vFr)
    vector< PUU > FrResults;// Aumatic frame description, Firs=N vertexs, Second= N edges

    vector< vector< PUF > > gtGraph; // adjacence graph, first = dest frame ID , second = score (nomalized distance until img center)

    void loadResultFrameInformations();
    void loadResultFrameMatch();

    void loadGtLoop();

public:

    float PiLoop, PiGT;

    CloseLoopAnaliseResult();

    void saveResults();
    void saveGTResults(unsigned uFr);
    void saveGTResults();

    void generateGTResultCompare();

    void saveGTLoopAndResults();
    void saveGTLoopAndResults(unsigned uFr,
                              unsigned *hit, unsigned *wrong, unsigned *miss,
                              unsigned *GTLoopDetections);

    void SplitGTResults();

};

#endif // CLOSELOOPANALISERESULT_H
