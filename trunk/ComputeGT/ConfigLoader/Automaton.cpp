#include "Automaton.h"
#include <iostream>


Automaton::Automaton(unsigned numOfStates)
{
    currentState=0;
    setNumberOfStates(numOfStates);
}

void Automaton::setNumberOfStates(unsigned numOfStates)
{
    graph.resize(numOfStates);
    dataStates.resize(numOfStates);
}

void Automaton::add(const AutomatonEdge &e)
{
    graph[e.fromId].push_back(e);
}

void Automaton::addFinalState(unsigned state)
{
    finalStates.insert(state);
}

void Automaton::setInitialState(unsigned state)
{
    currentState = initialState = state;
}

bool Automaton::isFinish() const
{
    return finalStates.find(currentState) != finalStates.end();
}

void Automaton::processSymbol(char c)
{
    vector<AutomatonEdge> &state = graph[currentState];

    for(unsigned i = 0 ; i < state.size() ; i++)
    {
        AutomatonEdge &e = state[i];
        if(e.doAcept(c))
        {
            currentState = e.toId;

            if(e.doRecordSymbols())
            {
                dataStates[e.toId]+=c;
            }

            #ifdef AUTOMATON_TRANSITIONS_DEBUG
                std::cout << "Transition from "
                          << e.fromId
                          << " by "
                          << c
                          << " to "
                          << e.toId
                          << std::endl;
            #endif
            break;
        }
    }
}

const string & Automaton::stateRecords(unsigned state)
{
    return dataStates[state];
}

void Automaton::reset()
{
    currentState = initialState;

    for(unsigned i = 0 ; i < dataStates.size() ; i++)
    {
        dataStates[i].clear();
    }
}
