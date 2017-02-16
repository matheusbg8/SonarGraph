#ifndef FILES_H
#define FILES_H

#include <iostream>

#include <sys/stat.h>
#include <cstring>
#include <cstdlib>
#include <cerrno>

using namespace std;

typedef struct stat Stat;

class Files
{
    static int do_mkdir(const char *path, mode_t mode);
public:
    Files();


    static int mkpath(const char *path, mode_t mode);

};

#endif // FILES_H
