#ifndef INDEXMAPPING_H
#define INDEXMAPPING_H

#include <vector>
using namespace std;

class IndexMapping
{
private:
    vector<int> m_map, m_invMap;
    int m_count;
public:
    IndexMapping();

    void setup(int size);
    int count() const;
    int map(int id);
    int inv(int id);

};

#endif // INDEXMAPPING_H
