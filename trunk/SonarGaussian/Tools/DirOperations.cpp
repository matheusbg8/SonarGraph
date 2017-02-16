#include "DirOperations.h"
#include <ctime>

DirOperations::DirOperations()
{
}

bool DirOperations::isDirectory(const string &pathName)
{
    path p(pathName);
    boost::system::error_code ec;
    file_status s = status(p,ec);
    if(!ec)
        return is_directory(s);
    return false;
}

bool DirOperations::isLink(const string &pathName)
{
    path p(pathName);
    boost::system::error_code ec;
    file_status s = status(p,ec);
    if(!ec)
        return is_symlink(s);
    return false;
}

double DirOperations::lastWriteTimeInSec(const string &pathName)
{
    path p(pathName);
    boost::system::error_code ec;

    std::time_t t = last_write_time(p,ec);

    if(!ec)
    {
//        struct tm y2k = {0};

//        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
//        y2k.tm_year = 0; y2k.tm_mon = 0; y2k.tm_mday = 0;

//        return difftime(t,mktime(&y2k));
        std::time_t currentTime = time(0x0);

        return difftime(currentTime,t);
    }
    cout << "DirOperations::lastWriteTimeInSec(" << pathName << ") - ERROR!" << endl;
    return 0.0;
}

bool DirOperations::mkdir(const string &pathName)
{
    boost::system::error_code ec;
    create_directory(pathName,ec);
    if(!ec)
    {
        return true;
    }
    return false;
}

bool DirOperations::rm(const string &pathName)
{
    boost::system::error_code ec;
    boost::filesystem::remove_all(pathName,ec);
    if(!ec)
    {
        return true;
    }
    return false;
}

bool DirOperations::list(std::list<string> &list, const string &pathName)
{
    path p(pathName);
    directory_iterator end_itr;

    list.clear();
    if(is_directory(p))
    {
        for(directory_iterator itr( p );
            itr != end_itr;
            ++itr )
            list.push_back(itr->path().string());

        return true;
    }
    return false;
}

bool DirOperations::list(std::list<string> &list, const string &pathName, string estension)
{
    path p(pathName);

    directory_iterator end_itr;
    unsigned esz = estension.size();
    if(is_directory(p))
    {
        std::cout << p << " is a directory containing:\n";

        for(directory_iterator itr( p );
            itr != end_itr;
            ++itr )
        {
            const string &name = itr->path().string();
            unsigned sz = name.size();

            if(
                sz > esz
                    &&
                name.substr(sz-esz, esz) == estension)
            {
                list.push_back(name);
            }
        }

        return true;
    }
    return false;
}

unsigned long DirOperations::availableSpace(const string &pathName)
{
    path p(pathName);
    boost::system::error_code ec;
    space_info s = space(p,ec);
    if(!ec)
    {
        return s.available;
    }
    cout << "DirOperations::availableSpace(" << pathName << ") - ERROR!" << endl;
    return 0;
}
