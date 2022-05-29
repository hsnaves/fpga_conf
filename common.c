#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "common.h"

void error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "error: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

void fatal(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "fatal: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);

    exit(1);
}

int read_file(const char *filename,
              void **buffer, size_t *size)
{
    FILE *fp;
    void *buf;
    size_t sz;

    fp = fopen(filename, "r");
    if (!fp) {
        error("could not open file `%s' for reading", filename);
        return FALSE;
    }

    if (fseek(fp, 0L, SEEK_END) < 0) {
        fclose(fp);
        error("error during seek of `%s`", filename);
        return FALSE;
    }

    sz = (size_t) ftell(fp);
    rewind(fp);

    buf = malloc(sz);
    if (!buf) {
        fclose(fp);
        error("could not allocate memory");
        return FALSE;
    }

    if (fread(buf, 1, sz, fp) != sz) {
        fclose(fp);
        free(buf);
        error("could not read `%s'", filename);
        return FALSE;
    }

    fclose(fp);

    buffer[0] = buf;
    size[0] = sz;

    return TRUE;
}
