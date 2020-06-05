# Suggested by GSL manual
CFLAGS = -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -Wno-unused-result -fshort-enums -fno-common -Dinline= -O2

LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk

default: all

mpc_get_lib:
	git clone https://github.com/ebni/MPC.git
	cd MPC; \
	make mpc.o; \
	make dyn.o; \
	make mpc_server; \
	mv *.o ..; \
	mv *.h ..; \
	mv mpc_server ..; \
	cd ..;

mpc_ctrl.o: mpc_ctrl.c mpc.h
	gcc -c mpc_ctrl.c $(CFLAGS) -o mpc_ctrl.o

mpc_ctrl: mpc_ctrl.o mpc.o mpc_ctrl.c
	gcc mpc_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_ctrl

all: mpc_ctrl mpc_server

clean:
	rm -rf mpc_ctrl mpc_server *.o dyn.h mpc.h MPC
