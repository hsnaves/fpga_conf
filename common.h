#ifndef __COMMON_H
#define __COMMON_H

#include <stddef.h>

/* Make sure TRUE and FALSE macros are defined. */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Useful macros such as MIN and MAX. */
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Exported functions. */
/* Report an error to stderr. */
void error(const char *fmt, ...);

/* Report a fatal error and exit the program. */
void fatal(const char *fmt, ...);

/* Read a file into a buffer.
 * The name of the file is given by `filename`, and the
 * buffer point is stored in `buffer`, and the size is stored
 * in `size`.
 * Return TRUE on success.
 */
int read_file(const char *filename,
              void **buffer, size_t *size);


#endif /* __COMMON_H */
