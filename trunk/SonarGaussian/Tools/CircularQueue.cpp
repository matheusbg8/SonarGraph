#include "CircularQueue.h"

CircularQueue::CircularQueue(unsigned maxSize):
    m_beg(0),m_end(maxSize-1), m_sz(0),m_msz(maxSize)
{
    m_el = new int[m_msz];
}

CircularQueue::~CircularQueue()
{
    delete [] m_el;
}

unsigned CircularQueue::size()
{
    return m_sz;
}

void CircularQueue::clear()
{
    m_beg = m_sz = 0;
    m_end = m_msz-1;
}

bool CircularQueue::empty()
{
    return m_sz == 0;
}

void CircularQueue::ResizeAndClear(unsigned newSize)
{
    delete [] m_el;
    m_el = new int[newSize];
    m_msz = newSize;
    m_beg = m_sz = 0;
    m_end = m_msz-1;
}

unsigned CircularQueue::maxSize()
{
    return m_msz;
}

bool CircularQueue::push(int v)
{
    if(m_sz < m_msz)
    {
        m_end = (m_end+1)%m_msz;
        m_el[m_end] = v;
        m_sz++;
        return true;
    }
    return false;
}

bool CircularQueue::pop()
{
    if(m_sz > 0)
    {
        m_beg = (m_beg+1)%m_msz;
        m_sz--;
        return true;
    }
    return false;
}

int CircularQueue::front()
{
    if(m_sz>0)
    {
        return m_el[m_beg];
    }
    return 0;
}

int CircularQueue::back()
{
    if(m_sz>0)
    {
        return m_el[m_end];
    }
    return 0;
}
