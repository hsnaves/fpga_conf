#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "fpga_manager.h"

static
int do_program(struct fpga_manager *mgr,
               const char *filename)
{
    void *conf_data;
    size_t size;

    if (!read_file(filename, &conf_data, &size))
        return FALSE;

    if (!fpga_manager_configure(mgr, conf_data, size))
        goto fail_program;

    free(conf_data);
    return TRUE;

fail_program:
    free(conf_data);
    return FALSE;
}

static
void print_help(const char *prog_name)
{
    printf("Usage:\n");
    printf("  %s <rbf_name>\n\n", prog_name);
    printf("where <rbf_name> is the name of the file to program the FPGA\n");
}

int main(int argc, char **argv)
{
    const char *filename;
    struct fpga_manager mgr;
    int i, ret;

    filename = NULL;
    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if ((strcmp("-h", argv[i]) == 0)
                || (strcmp("--help", argv[i]) == 0)) {

                /* Print the help message. */
                print_help(argv[0]);
                return 0;

            }
        }
        filename = (const char *) argv[i];
    }

    if (!filename) {
        error("please specify the filename to program");
        return 1;
    }

    if (!fpga_manager_create(&mgr)) {
        return 1;
    }
    ret = do_program(&mgr, filename);
    fpga_manager_destroy(&mgr);

    return (ret) ? 0 : 1;
}
