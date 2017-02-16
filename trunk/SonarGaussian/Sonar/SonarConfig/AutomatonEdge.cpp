#include "AutomatonEdge.h"

AutomatonEdge::AutomatonEdge(unsigned from, unsigned to,
                           const char *aceptableSymbols, bool recordSymbols, bool inverseAcept):
    fromId(from), toId(to),recordSymbols(recordSymbols) ,inverseAcept(inverseAcept)
{
    setAceptableSymbols(aceptableSymbols);
}

bool AutomatonEdge::doAcept(char symbol)
{
    if(inverseAcept)
        return aceptableSymbols.find(symbol) == aceptableSymbols.end();

    return aceptableSymbols.find(symbol) != aceptableSymbols.end();
}

bool AutomatonEdge::doRecordSymbols()
{
    return recordSymbols;
}

void AutomatonEdge::setAceptableSymbols(const char *symbols)
{
    while(*symbols!= '\0')
    {
        aceptableSymbols.insert(*symbols);
        symbols++;
    }
}
