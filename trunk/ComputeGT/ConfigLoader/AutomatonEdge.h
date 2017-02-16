#ifndef AUTOMATONEDGE_H
#define AUTOMATONEDGE_H

#include <set>
using namespace std;

class Automaton;

class AutomatonEdge
{
    unsigned fromId, toId;
    set<char> aceptableSymbols;
    bool inverseAcept;
    bool recordSymbols;
public:
    AutomatonEdge(unsigned from, unsigned to,
                 const char *aceptableSymbols,
                  bool recordSymbols=false,
                  bool inverseAcept=false);

    bool doAcept(char symbol);
    bool doRecordSymbols();

    void setAceptableSymbols(const char *symbols);

    friend class Automaton;
};

#endif // AUTOMATONEDGE_H
