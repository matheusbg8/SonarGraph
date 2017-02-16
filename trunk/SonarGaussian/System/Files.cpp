#include "Files.h"

int Files::do_mkdir(const char *path, mode_t mode)
{
    Stat st;
    int status = 0;

    if( stat(path, &st) != 0)
    {
        /* Directory does not exist. EEXIST for race condition */
        if (mkdir(path, mode) != 0 && errno != EEXIST)
            status = -1;
    }
    else if(!S_ISDIR(st.st_mode))
    {
        errno = ENOTDIR;
        status = -1;
    }

    return(status);
}

Files::Files()
{
}

/**
** mkpath - ensure all directories in path exist
** Algorithm takes the pessimistic view and works top-down to ensure
** each directory in path exists, rather than optimistically creating
** the last element and working backwards.
*/

int Files::mkpath(const char *path, mode_t mode)
{
    char *pp,*sp,*copypath = strdup(path);
    int  status;

    status = 0;
    pp = copypath;
    while (status == 0 && (sp = strchr(pp, '/')) != 0)
    {
        if (sp != pp)
        {
            /* Neither root nor double slash in path */
            *sp = '\0';
            status = do_mkdir(copypath, mode);
            *sp = '/';
        }
        pp = sp + 1;
    }

    if (status == 0)
        status = do_mkdir(path, mode);

    free(copypath);

    return status;
}
