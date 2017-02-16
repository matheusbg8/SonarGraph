#include "CloseLoopAnaliseResult.h"
#include <cstdio>
#include <iostream>

CloseLoopAnaliseResult::CloseLoopAnaliseResult():
    PiLoop(0.6),
    PiGT(0.1)
{
}

void CloseLoopAnaliseResult::loadResultFrameInformations()
{
    FrResults.clear();

    FILE *f =fopen("LoopDetections/ResultFramesInformations.csv","r");
    if(f == 0x0)
    {
        cout << "File FrameDescriptions.csv not found" << endl;
        return;
    }

    unsigned u, nV,nE;
    while(fscanf(f,"%u,",&u)!= -1)
    {
        fscanf(f,"%u,", &nV);
        fscanf(f,"%u", &nE);

        FrResults.push_back(PUU(nV,nE));
    }
}

void CloseLoopAnaliseResult::loadResultFrameMatch()
{
    rGraph.clear();

    unsigned nFrs= FrResults.size();
    rGraph.resize(nFrs);

    char str[300];
    unsigned u, v;
    float score;
    FILE *f=0x0;

    for(unsigned i =0 ; i < nFrs ; i++)
    {
        sprintf(str, "LoopDetections/MatchResults_fr%04u.csv",i);

        f = fopen(str,"r");
        if(f == 0x0)
        {
            cout << "File " << str << " not found" << endl;
            continue;
        }

        while(fscanf(f,"%u,",&u)!= -1)
        {
            fscanf(f,"%u,", &v);
            fscanf(f,"%g", &score);

            rGraph[u].push_back(PUF(v,score));
            rGraph[v].push_back(PUF(u,score));
        }
        fclose(f);
    }
}


void CloseLoopAnaliseResult::loadGtLoop()
{
    gtGraph.clear();
//    GTLoopDetection
    FILE *f = fopen("GTLoopDetection/GTSimilarMatchs.csv","r");
    if(f == 0x0)
    {
        cout << "Error to load GT results" << endl;
        return;

    }
    unsigned u, v;
    float score;

    gtGraph.reserve(1700);

    while(fscanf(f,"%u,",&u)!= -1)
    {
        fscanf(f,"%u,", &v);
        fscanf(f,"%g", &score);

        if(u >= gtGraph.size())
        {
            gtGraph.resize((u+1)*2);
        }

        gtGraph[u].push_back(PUF(v,score));
        gtGraph[v].push_back(PUF(u,score));
    }

    fclose(f);
}

void CloseLoopAnaliseResult::saveResults()
{
    FILE *f = fopen("ResultGraph.csv","w");
    if(f == 0x0)
    {
        cout << "Erro when saving results." << endl;
        return;
    }

    for(unsigned u =0 ; u < rGraph.size();u++)
    {
        for(unsigned i = u+1 ; i < rGraph[u].size(); i++)
        {
            PUF vertex = rGraph[u][i];
            fprintf(f,"%u,%u,%g\n",u,vertex.first,vertex.second);
        }
    }
    fclose(f);

    f = fopen("ResultFramesInformations.csv","w");
    if(f == 0x0)
    {
        cout << "Erro when saving results." << endl;
        return;
    }

    for(unsigned fr =0 ; fr < FrResults.size();fr++)
    {
        PUU frInfo = FrResults[fr];
        fprintf(f,"%u,%u,%u\n",fr,
                     frInfo.first,
                    frInfo.second);
    }
    fclose(f);

}

void CloseLoopAnaliseResult::saveGTResults(unsigned uFr)
{
    char str[300];
    sprintf(str,"GTLoopDetection/Splited/GtFrame_%04u.csv", uFr);
    FILE *f = fopen(str,"w");
    if(f == 0x0)
    {
        cout << "Error when saving file " << str << endl;
        return;
    }

    for(unsigned i = 0 ; i < gtGraph[uFr].size() ; i++)
    {
        unsigned vFr = gtGraph[uFr][i].first;
        float score = gtGraph[uFr][i].second;
        fprintf(f,"%u,%u,%g\n", uFr, vFr,score);
    }

    fclose(f);
}

void CloseLoopAnaliseResult::saveGTResults()
{
    for(unsigned i = 0 ;i < gtGraph.size(); i++)
    {
        saveGTResults(i);
    }
}

void CloseLoopAnaliseResult::generateGTResultCompare()
{
    // Load Results
    loadResultFrameInformations();
    loadResultFrameMatch();

    // Load Ground Truth
    loadGtLoop();

    // Save GT and Result together
    saveGTLoopAndResults();

}

void CloseLoopAnaliseResult::saveGTLoopAndResults()
{
    unsigned nFr = FrResults.size();

    FILE *correctLoopDetectionFile =
            fopen("GTResultCompare/CorrectLoopDetections.csv","w");

    fprintf(correctLoopDetectionFile,"FrameID,GtLoopDetections,Hit,Wrong,Miss,HitPercent,WrongPercent,MissPercent\n");
    for(unsigned i = 0 ; i < nFr ; i++ )
    {
        unsigned hit =0,wrong=0,miss=0,
                 GtLoopDetections = 0;

        saveGTLoopAndResults(i,
                      &hit,&wrong,&miss,
                      &GtLoopDetections);

        float HitPercent =0.f,
              MissPercent=0.f,
              WrongPercent=0.f;

        if(GtLoopDetections>0)
        {
            HitPercent = (float) hit / (float) (GtLoopDetections+miss);
            WrongPercent = (float) wrong / (float) (GtLoopDetections+miss);
            MissPercent = (float) miss / (float) (GtLoopDetections+miss);
        }

        fprintf(correctLoopDetectionFile,
                "%d,%u,%u,%u,%u,%g,%g,%g\n",i, GtLoopDetections,
                hit,wrong,miss, HitPercent,WrongPercent,MissPercent
               );
    }

    fclose(correctLoopDetectionFile);
}

/**
 * @brief
 *
 * @param uFr
 * @param correctLoopDetections
 * @param GTLoopDetections
 */
void CloseLoopAnaliseResult::saveGTLoopAndResults(unsigned uFr,
                                                  unsigned *hit,unsigned *wrong,unsigned *miss,
                                                  unsigned *GTLoopDetections)
{
    cout << "Save frame " << uFr << endl;

    unsigned nFr = FrResults.size();
    vector<float> gtResults(nFr,0.f);

    vector<pair<float,unsigned> > results(nFr);

    float bestResultScore=0.f;
    // Load results
    for(unsigned i = 0; i < rGraph[uFr].size(); i++)
    {
        unsigned vFr = rGraph[uFr][i].first;
        float score = rGraph[uFr][i].second;

        // Number of Matchs
        results[vFr].first = score;

        // If heape a loop detection
        results[vFr].second =
            score > ( (FrResults[uFr].first + FrResults[vFr].first) / 2.f) * PiLoop;

        bestResultScore = std::max( bestResultScore,
                                    score);
    }

    // Load Ground Truth results
    for(unsigned i = 0; i < gtGraph[uFr].size(); i++)
    {
        unsigned vFr = gtGraph[uFr][i].first;
        float score = gtGraph[uFr][i].second;
        gtResults[vFr] = score;
    }

    // Save
    char str[300];
    sprintf(str,"GTResultCompare/GTResultCompare_%04u.csv",uFr);
    FILE *f = fopen(str,"w");
    if(f == 0x0)
    {
        cout << "Error when saving " << str << endl;
        return ;
    }

    // Header
    fprintf(f,"DestFrId,NMatchFound,LoopDetected,GTScore\n");

    *hit = 0;
    *miss = 0;
    *wrong = 0;
    *GTLoopDetections=0;

    for(unsigned vFr = 0 ; vFr < nFr; vFr++)
    {
        fprintf(f,"%u,%g,%u,%g\n",vFr,
//                results[vFr].first/bestResultScore,
                results[vFr].first,
                results[vFr].second,
                gtResults[vFr]
                );

        if(gtResults[vFr] > PiGT)
        {
            (*GTLoopDetections)++;

            if(results[vFr].second)
            {
                (*hit)++;
            }else
            {
                (*miss)++;
            }
        }else
        {
            if(results[vFr].second)
            {
                (*wrong)++;
            }
        }

    }

    fclose(f);

}

void CloseLoopAnaliseResult::SplitGTResults()
{
    loadGtLoop();
    saveGTResults();

}
