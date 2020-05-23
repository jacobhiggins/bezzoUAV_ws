# Suggested by GSL manual
CFLAGS = -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -Wno-unused-result -fshort-enums -fno-common -Dinline= -O2

LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk

default: mpc_ctrl

mpc_get_lib:
	git clone https://github.com/ebni/MPC.git
	cd MPC; \
	make mpc.o; \
	make dyn.o; \
	mv *.o ..; \
	mv *.h ..; \
	cd ..
	rm -rf MPC

#mpc.h: mpc_get_lib

#mpc.o: mpc_get_lib
#	gcc -c mpc.c -I$(INCLUDEDIR) $(CFLAGS) -o mpc.o

#dyn.o: dyn.c dyn.h
#	gcc -c dyn.c -I$(INCLUDEDIR) $(CFLAGS) -o dyn.o

#mpc_ctrl: mpc_ctrl.o mpc.o dyn.o
#	gcc mpc_ctrl.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_ctrl

mpc_ctrl.o: mpc_ctrl.c mpc.h
	gcc -c mpc_ctrl.c $(CFLAGS) -o mpc_ctrl.o

mpc_ctrl: mpc_ctrl.o mpc.o
	gcc mpc_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_ctrl

clean:
	rm -rf mpc_ctrl *.o dyn.h mpc.h MPC
