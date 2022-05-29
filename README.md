# fpga_conf
Cyclone5 FPGA configuration program.

## Building (cross compilation)

To build fpga_conf, simply type the following command line on the root directory of the source code repository:

```sh
$ make
```

This assumes the environment variable `CROSS_COMPILE` is set and points to the correct ARM toolchain.

This will compile fpga_conf with no debugging information and with some compiler optimizations enabled. To enable debugging information and disable compiler optimizations, the user can specify `DEBUG=1 OPTIMIZE=0` in the command line above, such as:

```sh
$ DEBUG=1 OPTIMIZE=0 make
```
