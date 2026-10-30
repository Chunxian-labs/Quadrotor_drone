#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

/* Bare-metal replacements for the newlib-nano system-call hooks. */
int _close(int file)
{
    (void)file;
    return 0;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}

ssize_t _read(int file, void *buffer, size_t length)
{
    (void)file;
    (void)buffer;
    (void)length;
    return 0;
}

ssize_t _write(int file, const void *buffer, size_t length)
{
    (void)file;
    (void)buffer;
    return (ssize_t)length;
}

void _exit(int status)
{
    (void)status;
    for (;;) {
    }
}

int _getpid(void)
{
    return 1;
}

int _kill(int process, int signal)
{
    (void)process;
    (void)signal;
    errno = EINVAL;
    return -1;
}
