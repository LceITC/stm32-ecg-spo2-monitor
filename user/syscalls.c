#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>

// 堆内存（给 malloc 用）
static char *heap_end = 0;

// 提供给 sbrk 的堆起始地址（放在 RAM 末尾区域）
extern char _estack;  // 栈顶，来自链接脚本

// ---- 内存管理 ----
void *_sbrk(int incr)
{
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = (char *)&_estack - 0x2000;  // 预留 8KB 给栈
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > (char *)&_estack)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}

// ---- 文件操作（空实现）----
int _close(int file)     { return -1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _fstat(int file, struct stat *st)  { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file)    { return 1; }

// ---- 进程控制（空实现）----
void _exit(int status)   { while(1); }
int _kill(int pid, int sig) { return -1; }
int _getpid(void)        { return 1; }

// ---- I/O（可选择性实现）----
int _read(int file, char *ptr, int len) { return 0; }
int _write(int file, char *ptr, int len) { return len; }