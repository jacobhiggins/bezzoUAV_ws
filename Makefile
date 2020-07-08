# Suggested by GSL manual
CFLAGS = -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -Wno-unused-result -Wno-unused-parameter -fshort-enums -fno-common -Dinline= -O2

LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -pthread

default: mpc_ctrl

mpc_get_lib:
	cd mpc; \
	git clone https://github.com/ebni/MPC.git; \
	cd MPC; \
	make mpc.o; \
	make dyn.o; \
	make mpc_server; \
	mv *.o ..; \
	mv *.h ..; \
	mv mpc_server ..; \
	cd ../..;

mpc_ctrl:
	cd mpc; \
	gcc -c mpc_ctrl.c $(CFLAGS) -o mpc_ctrl.o; \
	gcc mpc_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_ctrl; \
	cd ..

mpc_shm_ctrl:
	cd mpc; \
	gcc -c mpc_shm_ctrl.c $(CFLAGS) -o mpc_shm_ctrl.o; \
	gcc mpc_shm_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_shm_ctrl; \
	cd ..

trace_proc:
	cd mpc; \
#	gcc trace_proc.c $(CFLAGS) $(LDFLAGS) -o trace_proc; \
	gcc trace_proc.c  $(LDFLAGS) -o trace_proc; \
	cd ..

mpc_clean:
	cd mpc; \
	rm -rf mpc_ctrl mpc_server *.o dyn.h mpc.h MPC ; \
	cd ..
