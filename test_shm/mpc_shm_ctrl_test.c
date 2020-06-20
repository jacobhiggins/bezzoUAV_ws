#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <strings.h>
#include "common_test.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/* GLOBAL VARIABLES */
int shm_id;
struct shared_data * data;

/*
 * Signal handler (Ctrl-C). This process will terminate only on Ctrl-C
 */
void term_handler(int signum);

int main(void) {
	size_t i;
	struct sigaction sa;

	/* Shared memory is used to read state from and write input to UAV */
	shm_id = shmget(MPC_SHM_KEY, sizeof(struct shared_data),
			MPC_SHM_FLAGS | IPC_CREAT | IPC_EXCL);
	if (shm_id == -1) {
		PRINT_ERROR("issue in shmget");
		exit(EXIT_FAILURE);
	}
	data = (struct shared_data *)shmat(shm_id, NULL, 0);
	bzero(data, sizeof(*data));
	
	/* Resetting all semaphores */
	for (i=0; i<MPC_SEM_NUM; i++) {
		if (sem_init(data->sems+i,1,0) < 0) {
			PRINT_ERROR("issue in sem_init");
			exit(EXIT_FAILURE);
		}
	}

	/* Setting up the signal handler for termination */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	/* Cycling forever to get the state from UAV. Ctrl-C will terminate */
	while (1) {
	/* while ((num_bytes = read(fd_rd, &msg_recv, sizeof(msg_recv)))) { */
		sem_wait(data->sems+MPC_SEM_STATE_WRITTEN);
		/* Now UAV wrote the state in data->state */

		/* Just a fake control action */
		for (i=0; i<MPC_INPUT_NUM; i++) {
			data->input[i] = data->state[i % MPC_STATE_NUM];
		}

		/* Now the input in data->input is ready for the UAV */
		sem_post(data->sems+MPC_SEM_INPUT_WRITTEN);
	}
}

void term_handler(int signum)
{
	/* Removing shared memory object */
	shmctl(shm_id, IPC_RMID, NULL);
	shmdt(data);
	exit(0);          // this is a brute, working way
}

