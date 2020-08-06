#ifndef _COMMON_H_
#define _COMMON_H_

#define STATE_NUM 12
#define INPUT_NUM 4

//to be updated depending on the json file.
struct msg_to_mpc {
	pid_t sender;
	struct timespec timestamp;  /* man clock_gettime for more info*/
	unsigned int job_id;
	double state[STATE_NUM]; /* x,y,z .... to be replaced by the state info */
};

// also here, to be updated depending on the json file
struct msg_from_mpc {
	pid_t sender;
	struct timespec timestamp;  /* man clock_gettime for more info*/
	double input[INPUT_NUM]; /* to be replaced by the MPC control */
};

#endif
