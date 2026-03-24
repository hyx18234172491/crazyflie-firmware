/**
 * Newlib syscalls stub implementation for ARM embedded systems.
 * Provides minimal implementations of required system calls for newlib libc.
 * 
 * These stubs are required when using standard C library functions that depend
 * on low-level I/O operations (_sbrk, _read, _write, _close, etc.)
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

/* External variables provided by the linker script */
extern char _end;       /* End of data section */

static char *heap_end = 0;

/**
 * Increase program data space.
 * Newlib calls this to get more memory for malloc.
 */
caddr_t _sbrk(int incr)
{
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;
    heap_end += incr;

    return (caddr_t) prev_heap_end;
}

/**
 * Close a file descriptor.
 */
int _close(int fd)
{
    (void)fd;  /* Not used */
    return -1;
}

/**
 * Check if a file descriptor refers to an interactive device.
 */
int _isatty(int fd)
{
    (void)fd;  /* Not used */
    return 0;
}

/**
 * Get file status.
 */
int _fstat(int fd, struct stat *st)
{
    (void)fd;  /* Not used */
    if (st) {
        st->st_mode = S_IFCHR;  /* Character device */
    }
    return 0;
}

/**
 * Seek to a position in a file.
 */
off_t _lseek(int fd, off_t offset, int whence)
{
    (void)fd;      /* Not used */
    (void)offset;  /* Not used */
    (void)whence;  /* Not used */
    return -1;
}

/**
 * Read from a file descriptor.
 * Stub: returns EOF for all reads.
 */
int _read(int fd, char *ptr, int len)
{
    (void)fd;   /* Not used */
    (void)ptr;  /* Not used */
    (void)len;  /* Not used */
    return 0;   /* EOF */
}

/**
 * Write to a file descriptor.
 * For stdout/stderr (fd 1,2), this would normally output somewhere.
 * This stub just returns success without actually writing.
 */
int _write(int fd, const char *ptr, int len)
{
    (void)fd;   /* Not used */
    (void)ptr;  /* Not used */
    return len; /* Pretend we wrote everything */
}

/**
 * Exit the program.
 */
void _exit(int code)
{
    (void)code;  /* Not used */
    /* Infinite loop - program will need to be reset */
    while(1);
}

/**
 * Kill a process or send a signal.
 */
int _kill(int pid, int sig)
{
    (void)pid;  /* Not used */
    (void)sig;  /* Not used */
    return -1;
}

/**
 * Get process ID.
 */
int _getpid(void)
{
    return 1;
}
