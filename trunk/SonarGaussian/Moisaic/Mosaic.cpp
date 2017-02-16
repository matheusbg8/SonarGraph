#include "Mosaic.h"
#include <map>

Mosaic::Mosaic()
{
}

void Mosaic::createMosaic()
{

}

void Mosaic::createHomographyGraph(vector<FrameGT> &frames)
{
    typedef pair<unsigned,unsigned> PUU;

    for(unsigned srcFrID = 0 ; srcFrID < frames.size(); srcFrID++)
    {
        vector< vector<PUU> > &match = frames[srcFrID].match;

        map<unsigned, // Dest Frame ID
             pair< vector<Point2f> , vector<Point2f> > // point on src frame, point on dest frame
           > mapMatch; // Map of match points of srcFrame to all frames

        // Find match points from srcFrame to all frames
        for(unsigned guID = 0 ; match.size() ; guID++)
        {
            Gaussian &gu = frames[srcFrID].gaussians[guID];
            Point2f guPosition(gu.x, gu.y);

            for(unsigned m = 0 ; match[guID].size(); m++)
            {
                unsigned gvID = match[guID][m].second,
                         destFrID = match[guID][m].first;

                Gaussian &gv = frames[destFrID].gaussians[gvID];
                Point2f gvPosition(gv.x, gv.y);

                // src match
                mapMatch[destFrID].first
                      .push_back(guPosition);

                // dest match
                mapMatch[destFrID].second
                      .push_back(gvPosition);
            }
        }

        // Compute homography from srcFrame to all Frame
        // ang make a srcFrame links on HomographyGraph

    }
}
