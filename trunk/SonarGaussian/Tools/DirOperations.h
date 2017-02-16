#ifndef DIROPERATIONS_H
#define DIROPERATIONS_H

#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <list>

using namespace boost::filesystem;
using namespace std;

class DirOperations
{
public:
    DirOperations();
    static bool isDirectory(const string &pathName);
    static bool isLink(const string &pathName);
    static double lastWriteTimeInSec(const string &pathName);
    static bool mkdir(const string &pathName);
    static bool rm(const string &pathName);
    static bool list(std::list<string> &list, const string &pathName);
    static bool list(std::list<string> &list, const string &pathName, string estension);

    static long unsigned availableSpace(const string &pathName);
};

/*
int main(int argc, char* argv[])
{
    MatchViewer mv;
    cout << "OpenCV version " << CV_VERSION << endl;

    cout << DirOperations::isLink("Teste") << " "
         << DirOperations::isLink("link") << " "
         << DirOperations::isLink("/home/matheusbg/furgbol/trunk/IA") << endl;

    cout << current_path() << endl;
    cout.precision(12);
    cout << DirOperations::lastWriteTimeInSec("SonarGaussian") << endl;

    std::list<string> list;
    if(DirOperations::list(list, ".",".png"))
    {
        for(std::list<string>::iterator i = list.begin();
            i != list.end() ; i++)
        {
            cout << *i << endl;
        }
    }

    if(DirOperations::rm("Teste"))
        cout << "End of main" << endl;
    return 0;
}
*/

#endif // DIROPERATIONS_H
