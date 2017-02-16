#ifndef AUTOMATON_H
#define AUTOMATON_H

#include <list>
#include <vector>
using namespace std;

#include "AutomatonEdge.h"
#include <string>
#include <map>
#include <set>

using namespace std;

// ==== DEBUG SECTION ====
//#define AUTOMATON_TRANSITIONS_DEBUG
// == END DEBIG SECTION ==

/**
 * @brief It's a Deterministic Finite Automaton (DFA)
 * developed to process text.
 *
 */
class Automaton
{
private:
    vector<vector<AutomatonEdge> > graph;
    unsigned initialState;
    set<unsigned> finalStates;

    mutable unsigned currentState;
    mutable vector<string> dataStates;

public:
    Automaton(unsigned numOfStates);

    void setNumberOfStates(unsigned numOfStates);

    void add(const AutomatonEdge& e);

    void addFinalState(unsigned state);

    void setInitialState(unsigned state);

    bool isFinish() const;

    void processSymbol(char c);

    const string &stateRecords(unsigned state);

    void reset();

};

#endif // AUTOMATON_H
