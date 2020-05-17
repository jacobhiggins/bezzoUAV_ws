#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "common.h"


int main(int argc, char * argv[]) {
	int fd_rd, fd_wr, num_bytes;

	/* Messages sent/received */
	struct msg_to_mpc   msg_recv;
	struct msg_from_mpc msg_sent;

	printf("Starting mpc_ctrl\n");
	  

	fd_rd = atoi(argv[1]);
	fd_wr = atoi(argv[2]);

	while (num_bytes = read(fd_rd, &msg_recv, sizeof(msg_recv))) {
		/* Printing received message */
		printf("Got message from %i, job %i\n",
		       msg_recv.sender, msg_recv.job_id);
		printf("At %f\n", (double)msg_recv.timestamp.tv_sec
		       +(double)msg_recv.timestamp.tv_sec*1e-9);
		printf("State: %i\n\n", msg_recv.state);
		
		/* Preparing and sending control message */
		msg_sent.sender = getpid();
		clock_gettime(CLOCK_MONOTONIC, &(msg_sent.timestamp));
		/* Applying (fake) control law */
		msg_sent.ctrl = msg_recv.state+1000;
		write(fd_wr, &msg_sent, sizeof(msg_sent));
	}
	
	
}


	
