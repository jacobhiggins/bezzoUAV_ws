#ifndef _COMMON_H_
#define _COMMON_H_

#include <semaphore.h>

#define MPC_SHM_KEY 0xC1A0  /* hard-coded key of shared mem SysV object */
#define MPC_SHM_FLAGS 0666  /* we go easy: everybody reads and writes */

#define MPC_STATE_NUM 12
#define MPC_INPUT_NUM 4

#define MPC_SEM_NUM            2
#define MPC_SEM_STATE_WRITTEN  0   /* +1: UAV; -1 MPC controller */
#define MPC_SEM_INPUT_WRITTEN  1   /* +1: MPC controller; -1 UAV */

struct shared_data {
	sem_t  sems[MPC_SEM_NUM];    /* semaphores to regulate communication */
	double state[MPC_STATE_NUM]; /* UAV -> MPC controller */
	double input[MPC_INPUT_NUM]; /* MPC controller -> UAV */
};

#endif /* _COMMON_H_ */
