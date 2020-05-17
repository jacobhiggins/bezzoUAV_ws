#ifndef _COMMON_H_
#define _COMMON_H_

struct msg_to_mpc {
	pid_t sender;
	struct timespec timestamp;  /* man clock_gettime for more info*/
	unsigned int job_id;
	int state; /* to be replaced by the state info */
};


struct msg_from_mpc {
	pid_t sender;
	struct timespec timestamp;  /* man clock_gettime for more info*/
	int ctrl; /* to be replaced by the MPC control */
};

#endif
