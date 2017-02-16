#include "IndexMapping.h"

IndexMapping::IndexMapping()
{
}

void IndexMapping::setup(int size)
{
    m_map.clear();
    m_invMap.clear();
    m_map.resize(size,-1);
    m_invMap.resize(size,-1);
    m_count = 0;
}

int IndexMapping::count() const
{
    return m_count;
}

int IndexMapping::map(int id)
{
    int invId=-1;
    switch(m_map[id])
    {
    case -1:
        invId = m_map[id] = m_count;
        m_invMap[m_count] = id;
        m_count++;
    break;
    default:
        invId = m_map[id];
    break;
    }
    return m_count-1;
}

int IndexMapping::inv(int id)
{
    return m_invMap[id];
}

