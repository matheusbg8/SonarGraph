#include "GraphMatchFinder.h"

GraphMatchFinder::GraphMatchFinder():
    m_vertexMatcher(0x0)
{
}

void GraphMatchFinder::setVertexMatcher(VertexMatcher *vertexMatcher)
{
    m_vertexMatcher = vertexMatcher;
}
