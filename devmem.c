#include <stddef.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "common.h"
#include "devmem.h"

void *devmem_map(off_t address, size_t size)
{
    int mem_fd;
    void *res;

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (mem_fd == -1) {
        error("could not open `/dev/mem`");
        return NULL;
    }

    res = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
               mem_fd, address);

    close(mem_fd);

    if (res == MAP_FAILED) {
        error("could not perform the address mapping");
        return NULL;
    }

    return res;
}

int devmem_unmap(void* map, size_t size)
{
    if (munmap(map, size) < 0) {
        error("could not perform the unmapping");
        return TRUE;
    }

    return FALSE;
}
