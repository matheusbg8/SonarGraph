#include "CSVMatrixGenerator.h"
#include "GroundTruth/FrameGT.h"


#include <stdexcept>
#include <vector>
#include <set>
using namespace std;


CSVMatrixGenerator::CSVMatrixGenerator():
    distToLinkGraph(200.f), prefix("CSVs/"), VertexBalance(1.f)
{
    cout << "Initializing!!"
         << endl
         << GTFrameMatchs.size()
         << m_frames.size()
         << endl;
}

void CSVMatrixGenerator::saveCSV(const char *fileName, const CSVMat &m)
{
    FILE *f = fopen((prefix + fileName).c_str(),"w");
    if(f== 0x0)
    {
        std::cout << "File " << fileName << " can't be saved" << std::endl;
        return;
    }

    for(unsigned i = 0 ; i < m.size() ; i++)
    {
        fprintf(f,"%g",m[i][0]);
        for(unsigned j = 1; j < m[i].size(); j++)
        {
            fprintf(f,",%g",m[i][j]);
        }
        fputc('\n',f);
    }
    fclose(f);
}

void CSVMatrixGenerator::loadCSV(const char *fileName, const CSVMat &m)
{

}

void CSVMatrixGenerator::print(const CSVMatrixGenerator::CSVMat &m)
{
    for(unsigned i =0 ; i < m.size() ; i++)
    {
        for(unsigned j =0 ; j < m[i].size(); j++)
        {
            cout << m[i][j] << ' ';
        }
        cout << endl;
    }
}

void CSVMatrixGenerator::loadGroundTruth(const char *fileName)
{
    FILE *f = fopen(fileName, "r");

    if(f == 0x0)
        cout << "File " << fileName << " not found" << endl;

    unsigned g,frameSize,gaussiansSize, ug, vFr, vG, nMatch,i;
    char imgFileName[300];

    // Number of Frames on this GroundTrutuh
    fscanf(f,"%u", &frameSize);
    m_frames.resize(frameSize);

    cout << "Size of frames " << m_frames.size() << endl;
    for(unsigned iFr = 0 ; iFr < frameSize ; iFr++)
    {
        FrameGT &fr = m_frames[iFr];

        // Frame description
        // Image filename frameID NumberOfGaussians
        fscanf(f,"%s %u %u %u %u",imgFileName, &(fr.frameNumber), &gaussiansSize, &fr.threshold1, &fr.threshold2);
        fr.fileName = imgFileName;
        cout << "Loading frame " << fr.frameNumber << endl;

        if(gaussiansSize == 0) continue;

        // Gaussians Description
        vector<Gaussian> &gs = fr.gaussians;
        gs.resize(gaussiansSize);
        for(g = 0 ; g < gaussiansSize; g++)
        {
            Gaussian &ga = gs[g];
            // ID X, Y, Intensity , DX, DY, DI, Ang, N
            fscanf(f,"%*d %f %f %f %f %f %f %f %u",
                    &ga.x, &ga.y , &ga.intensity,
                    &ga.dx , &ga.dy, &ga.di,
                    &ga.ang , &ga.N );
        }

        // Match Description
        vector< vector<pair<unsigned,unsigned> > > &match = fr.match;
        match.resize(gaussiansSize);
        fscanf(f,"%u",&nMatch);

        for( i = 0; i < nMatch; i++)
        {
            fscanf(f,"%u %u %u",&ug, &vFr, &vG);
            match[ug].push_back(pair<unsigned, unsigned>(vFr,vG));
        }
    }
    fclose(f);

    cout << "Load ground truth completed" << endl;

    for(unsigned i = 0 ; i < m_frames.size() ; i++)
    {
        cout << m_frames[i] << endl;
    }

    computeGTFrameMatchs(m_frames);
    computeGTGaussianMatchs(GTFrameMatchs,m_frames);
}

bool CSVMatrixGenerator::loadEmptyFrames(const char *fileName)
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
        m_frames.push_back(FrameGT(string(img_file_name),frame++));
    }
    fclose(f_img_names);

    // Loading mask and isonification pattern
    return true;
}

void CSVMatrixGenerator::computeGTGaussianMatchs(SG &graph, FrameGT frame, unsigned dstFr)
{
    typedef pair<unsigned, unsigned> PUU;

    unsigned uG,iG;
    vector< vector<PUU> > &match = frame.match;
    graph.resize(frame.gaussians.size());

    for(uG = 0; uG < match.size() ; uG++)
    {
        for(iG = 0; iG < match[uG].size(); iG++)
        {
            PUU &edge = match[uG][iG];
            if(edge.first == dstFr)
            {
                graph[uG].push_back(edge.second);
            }
        }
    }
}

void CSVMatrixGenerator::computeGTGaussianMatchs(vector<vector<PUSG> > &graph, vector<FrameGT> &frames)
{
    for(unsigned uFr = 0; uFr < graph.size(); uFr++)
    {

        for(unsigned iFr = 0 ; iFr < graph[uFr].size(); iFr++)
        {
            PUSG &edge = graph[uFr][iFr];

            computeGTGaussianMatchs(edge.second,frames[uFr],edge.first);
        }
    }
}

void CSVMatrixGenerator::computeGTFrameMatchs(vector<FrameGT> &frames)
{
    GTFrameMatchs.resize(frames.size());

    // Computing results for each frame
    for(unsigned iFr = 0 ; iFr < frames.size(); iFr++)
    {
        FrameGT &currentFr = frames[iFr];
        vector< vector<pair<unsigned,unsigned> > > & currentMatchs = currentFr.match;
        set<unsigned> GTMatchDestIDs;

        // Search related match frames of iFr (currentFrame)
        for(unsigned i = 0; i < currentMatchs.size(); i++)
        {
            // Looking inside currentFrame gaussian i match j
            for(unsigned j = 0 ; j < currentMatchs[i].size() ; j++)
            {
                unsigned destFrameMatchID = currentMatchs[i][j].first;
                if(GTMatchDestIDs.find(destFrameMatchID) == GTMatchDestIDs.end())
                {
                    GTMatchDestIDs.insert(destFrameMatchID);
                }
            }
        }

        set<unsigned>::iterator destFrID;
        for(destFrID = GTMatchDestIDs.begin();
            destFrID != GTMatchDestIDs.end() ;
             destFrID++)
        {
            GTFrameMatchs[iFr].push_back(PUSG(*destFrID,SG()));
        }
    }
}

void CSVMatrixGenerator::describeFrames(vector<FrameGT *> &frames, Sonar &sonar,
                                        vector<SonarDescritor*> &descriptors)
{
    Mat img;
    descriptors.resize(frames.size());

    sonar.storeImgs = false;
    sonar.drawPixelFound = false;

    cout << "Describing sonar images..." << endl;
    for(unsigned iFr = 0 ; iFr < frames.size(); iFr++)
    {
        cout << "Describing frame " << iFr << endl;
        img = imread(frames[iFr]->fileName.c_str(),CV_LOAD_IMAGE_ANYDEPTH);
        descriptors[iFr] = sonar.newImageDirect(img);
    }
}

void CSVMatrixGenerator::generateGroundTruthCSV(FrameGT &uFr, FrameGT &vFr, SG &matchs)
{
    char str[200];
    sprintf(str, "gtX_%04u_%04u.csv", uFr.frameNumber, vFr.frameNumber);

    // Create graph assiment matrix between uFr and vFr
    vector< vector<float> > x(uFr.gaussians.size(),
                vector<float>(vFr.gaussians.size(),0.f)
                );

    for(unsigned uG = 0; uG < matchs.size() ; uG++)
    {
        for(unsigned iG = 0; iG < matchs[uG].size(); iG++)
        {
            unsigned vG = matchs[uG][iG];
            x[uG][vG] = 1.f;
        }
    }

    saveCSV(str,x);
}

void CSVMatrixGenerator::generateGroundTruthCSV(vector<vector<PUSG> > &GTFrameMatchs)
{
    vector<uchar> usedGraph(GTFrameMatchs.size(),0);

    for(unsigned uFr = 0 ; uFr < GTFrameMatchs.size() ; uFr++)
    {
        for(unsigned iFr = 0 ; iFr < GTFrameMatchs[uFr].size(); iFr++)
        {
            PUSG &edge = GTFrameMatchs[uFr][iFr];
            generateGroundTruthCSV(m_frames[uFr], m_frames[edge.first], edge.second);
        }
    }

    for(unsigned iFr = 0 ; iFr < usedGraph.size() ;iFr++)
    {
        if(usedGraph[iFr])
            generateGraphsPointsCSV(m_frames[iFr]);
    }
}

void CSVMatrixGenerator::generateGraphsPointsCSV(FrameGT &frame)
{
    char str[200];
    sprintf(str,"Pts_%04u.csv", frame.frameNumber);

    vector<Gaussian> &gs = frame.gaussians;
    vector<vector<float> >Pts(gs.size());

    for(unsigned i = 0 ; i < gs.size(); i++)
    {
        Pts[i].resize(2);
        Pts[i][0] = gs[i].x;
        Pts[i][1] = gs[i].y;
    }

    saveCSV(str,Pts);
}

void CSVMatrixGenerator::generateGraphsPointsCSV(vector<FrameGT> &frames)
{
    for(unsigned iFr = 0 ; iFr < frames.size() ; iFr++)
    {
        generateGraphsPointsCSV(frames[iFr]);
    }
}

void CSVMatrixGenerator::generateGaussianVertexCSV(FrameGT &frame)
{
    char str[200];
    sprintf(str,"Vertexes_%04u.csv", frame.frameNumber);

    vector<Gaussian> &gs = frame.gaussians;
    vector<vector<float> >V(gs.size());

    for(unsigned i = 0 ; i < gs.size(); i++)
    {
        V[i].resize(5);
        V[i][0] = gs[i].x;
        V[i][1] = gs[i].y;
        V[i][2] = gs[i].dx;
        V[i][3] = gs[i].dy;
        V[i][4] = gs[i].ang;
    }

    saveCSV(str,V);
}

void CSVMatrixGenerator::generateGaussianVertexCSV(vector<FrameGT> &frames)
{
    for(unsigned iFr = 0 ; iFr < frames.size() ; iFr++)
    {
        generateGaussianVertexCSV(frames[iFr]);
    }
}

void CSVMatrixGenerator::generateEgCSV(unsigned frId, vector<CSVMatrixGenerator::CSVEdge> &enumEdges)
{
    unsigned nE = enumEdges.size();
    CSVMat Eg(2, vector<float>(nE*2));

    for(unsigned i =0 ;i < nE ; i ++)
    {
        Eg[1][i+nE] = Eg[0][i] = enumEdges[i].first;
        Eg[0][i+nE] = Eg[1][i] = enumEdges[i].second->dest;
    }

    char str[200];

    sprintf(str, "Eg_%04u.csv",frId);
    saveCSV(str,Eg);
}

/**
 * @brief Generate undirected graph to FGM
 *
 * @param frId
 * @param vertices
 * @param edges
 */
void CSVMatrixGenerator::generateGH(unsigned frId, vector<Gaussian> &vertices, vector<CSVMatrixGenerator::CSVEdge> &edges)
{
    unsigned nV = vertices.size(),
             nE = edges.size();

    CSVMat G(nV, vector<float>(nE,0.f)),
           H(nV, vector<float>(nE+nV,0.f)); // H is completed with diagonal matrix in undirected graphs

    for(unsigned i = 0 ; i < nE ; i++)
    {
        unsigned u= edges[i].first,
                 v= edges[i].second->dest;
        G[u][i] = 1.f;
        H[v][i] = 1.f;
        G[v][i] = 1.f;
        H[u][i] = 1.f;
    }

    // Complete H with diagonal matrix
    for(unsigned i =0 ; i < nV; i++)
    {
        H[i][i+nE]= 1.f;
    }

    char str[200];
    sprintf(str, "G_%04u.csv", frId);
    saveCSV(str, G);

    sprintf(str, "H_%04u.csv", frId);
    saveCSV(str, H);
}

void CSVMatrixGenerator::generateVisCSV(unsigned frId, vector<Gaussian> &vertices, vector<CSVMatrixGenerator::CSVEdge> &edges)
{
    CSVMat vis(vertices.size() , vector<float>(vertices.size(),0.f));

    for(unsigned i = 0 ; i < edges.size() ; i++)
    {
        unsigned u = edges[i].first,
                 v = edges[i].second->dest;
        vis[v][u] = vis[u][v] = 1.f;
    }
    char str[200];
    sprintf(str,"Vis_%04u.csv",frId);
    saveCSV(str,vis);
}

/**
 * @brief Generate Eg of a graph.
 *
 * @param g - A graph.
 * @param Eg - A matrix Eg (2xnE) of edges Eg[0][e] = e->src Eg[1][e] = e->dst.
 */
void CSVMatrixGenerator::generateEg(vector<vector<GraphLink *> > &g, CSVMatrixGenerator::CSVMat &Eg)
{
    Eg.clear();
    Eg.resize(2);

    for(unsigned u = 0 ;u < g.size() ; u++)
    {
        for(unsigned i = 0 ; i < g.size();i++)
        {
            Eg[0].push_back(u);
            Eg[1].push_back(g[u][i]->dest);
        }
    }
}

void CSVMatrixGenerator::enumEdges(vector<vector<GraphLink *> > &g, vector<CSVMatrixGenerator::CSVEdge> &enumEdges)
{
    enumEdges.clear();

    for(unsigned u = 0 ;u < g.size() ; u++)
    {
        for(unsigned i = 0 ; i < g[u].size();i++)
        {
            enumEdges.push_back(CSVEdge(u,g[u][i]));
        }
    }
}

float CSVMatrixGenerator::computeSymmetricError(Gaussian &u, Gaussian &v)
{
    // Flatness diference!
    // Y aways bigger than X
    return std::fabs((u.x / u.y) - (v.x / v.y));
}

float CSVMatrixGenerator::computeSymmetricError(GraphLink *u, GraphLink *v)
{
    // Use dist diference
    return std::fabs(u->p - v->p);
}

void CSVMatrixGenerator::generateKp_Kq(SonarDescritor &sdU, vector<CSVEdge> &Eu,
                                       SonarDescritor &sdV, vector<CSVEdge> &Ev,
                                       CSVMat &Kp, CSVMat &Kq)
{
    vector<Gaussian> &gaU = sdU.gaussians,
                     &gaV = sdV.gaussians;

    vector<vector<GraphLink*> >  &graphU = sdU.graph,
                                 &graphV = sdV.graph;
    vector<MatchInfoWeighted> edgeMatchs;

    Kp.clear(); Kq.clear();
    Kp.resize(gaU.size(), vector<float>(gaV.size(),0.f));
    Kq.resize(Eu.size(), vector<float>(Ev.size(),0.f));

    // Computing Kp
    float KpMax=0.f, KqMax=0.f;
    for(unsigned i =0 ; i < gaU.size() ; i++)
    {
        for(unsigned j =0 ; j < gaV.size() ; j++)
        {
            float err = computeSymmetricError(gaU[i], gaV[j]);
//            float err = gm.computeEdgeMatchAxe(graphU[i], graphV[j],edgeMatchs);
            if(err > KpMax)
                KpMax = err;

            Kp[i][j] = err;
        }
    }
    cout << "KpMax " << KpMax << endl;

    // Normalize and inverse Kp
    for(unsigned i =0 ; i < gaU.size() ; i++)
    {
        for(unsigned j =0 ; j < gaV.size() ; j++)
        {
            Kp[i][j] = (1.f - Kp[i][j] / KpMax);
        }
    }

    // Computing Kq
    for(unsigned i =0 ; i < Eu.size(); i++)
    {
        for(unsigned j=0; j< Ev.size(); j++)
        {
            float err = computeSymmetricError(Eu[i].second, Ev[j].second);
            if(err > KqMax)
                KqMax = err;

            Kq[i][j] = err;
        }
    }
    cout << "KqMax " << KqMax << endl;

    // Normalizing and inverse Kq
    for(unsigned i =0 ; i < Eu.size(); i++)
    {
        for(unsigned j=0; j< Ev.size(); j++)
        {
            Kq[i][j] = (1.f - Kq[i][j] / KqMax);
        }
    }
}

void CSVMatrixGenerator::generateCSVs(vector<FrameGT> &frames, vector<vector<PUSG> > &matchs)
{
    vector<SonarDescritor> sd(m_frames.size());

    vector<CSVEdge> enumEdge[frames.size()];

    for(unsigned uFr = 0 ; uFr < matchs.size(); uFr++)
    {
        for(unsigned iFr = 0 ; iFr < matchs[uFr].size(); iFr++)
        {
            PUSG &edge = matchs[uFr][iFr];
            unsigned vFr = edge.first;

            SonarDescritor &du = sd[uFr],
                           &dv = sd[vFr];

            // Verify if sonar descriptor has gaussians
            if(du.gaussians.size() == 0)
            {
                du.gaussians = m_frames[uFr].gaussians;
            }
            if(dv.gaussians.size() == 0)
            {
                dv.gaussians = m_frames[vFr].gaussians;
            }

            // Verify if the frames was described
            if(du.graph.size() == 0)
            {
                du.createGraph(distToLinkGraph,false);
                enumEdges(du.graph,enumEdge[uFr]);
            }
            if(dv.graph.size() == 0 )
            {
                dv.createGraph(distToLinkGraph,false);
                enumEdges(dv.graph,enumEdge[vFr]);
            }

            // Generate Kp and Kq to FGM
            CSVMat Kp, Kq;
            generateKp_Kq(du,enumEdge[uFr],
                          dv,enumEdge[vFr],
                          Kp,Kq);

            char str[400];
            sprintf(str, "Kp_%04u_%04u.csv", uFr, vFr);
            saveCSV(str, Kp);

//            cout << "Kp " << Kp.size() << " x " << Kp[0].size() << endl;
//            cout << "Kq " << Kq.size() << " x " << Kq[0].size() << endl;

            sprintf(str, "Kq_%04u_%04u.csv", uFr, vFr);
            saveCSV(str, Kq);

            // Generate Ground Truth
            generateGroundTruthCSV(frames[uFr], frames[vFr], edge.second);
        }
    }

    // Generate Graphs Eg, Pt, G, H
    for(unsigned fr =0 ;fr < frames.size() ; fr++)
    {
        // If this frame is used in ground truth
        if(enumEdge[fr].size() > 0)
        {
            generateGraphsPointsCSV(frames[fr]);
//            generateGaussianVertexCSV(frames[fr]);
            generateEgCSV(fr, enumEdge[fr] );
            generateVisCSV(fr,frames[fr].gaussians,enumEdge[fr]);
            generateGH(fr,frames[fr].gaussians,enumEdge[fr]);
        }
    }
}


/**
 * @brief Generate undirected graph
 * from ground truth to csv matrix to
 * use in FGM frame work.
 *
 * @param groundTruthFileName
 */
void CSVMatrixGenerator::generate(const char *groundTruthFileName)
{
    loadGroundTruth(groundTruthFileName);
    generateCSVs(m_frames,GTFrameMatchs);


}

