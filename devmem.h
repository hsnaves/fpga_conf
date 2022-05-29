#ifndef __DEVMEM_H
#define __DEVMEM_H

#include <stddef.h>
#include <sys/types.h>

/* Exported functions. */

/* Map the I/O region starting at physical address `address`
 * and of size `size` to the address space of the current
 * process. Return NULL if failed.
 */
void *devmem_map(off_t address, size_t size);

/* Unmap the address space `map` of size `size`. This undoes
 * the mapping of `devmem_map()`.
 * Return TRUE on success.
 */
int devmem_unmap(void* map, size_t size);


#endif /* __DEVMEM_H */
