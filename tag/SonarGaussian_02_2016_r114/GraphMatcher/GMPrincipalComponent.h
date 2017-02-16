#ifndef GMPRINCIPALCOMPONENT_H
#define GMPRINCIPALCOMPONENT_H

#include "GraphMatcher/GraphMatcher.h"

class GMPrincipalComponent : public GraphMatcher
{
public:
    GMPrincipalComponent();

    void matche(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void matcheDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                            vector<pair<unsigned, unsigned> > &vertexMatch,
                            vector<MatchInfoExtended> &matchInfo);
};

#endif // GMPRINCIPALCOMPONENT_H
