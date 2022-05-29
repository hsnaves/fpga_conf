OBJS := $(OBJS) common.o devmem.o fpga_manager.o main.o

common.o: common.c common.h
devmem.o: devmem.c common.h devmem.h
fpga_manager.o: fpga_manager.c common.h devmem.h fpga_manager.h
main.o: main.c common.h fpga_manager.h
