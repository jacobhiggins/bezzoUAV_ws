/*
 * The interaction  between the MPC  controller and the  plant happens
 * via shared memory. It is  expected that the interaction follows the
 * steps:
 *
 * 1. the  plant writes  the state  in the state  field of  the shared
 * struct
 *
 * 2.  the  plant notifies  a  fresh  state  by sem_post(...)  on  the
 * semaphore MPC_SEM_STATE_WRITTEN
 *
 * 3.  the plant  then waits  for the  input by  sem_wait(...) on  the
 * semaphore  MPC_SEM_INPUT_WRITTEN. Also  sem_timedwait may  be used,
 * with care.
 *
 * 4. the input is available in the struct to be fed to the plant
 *
 * Based on  the characteristics of  the plant, the  following macros,
 * may need to be redefined:
 *
 * MPC_STATE_NUM, with the number of system states
 * MPC_INPUT_NUM, with the number of input to the system
 *
 * The shared memory is created and  removed by the MPC controller. In
 * case, the  default shared  memory key 0xC1A0  is already  used (for
 * example, because  another MPC controller  is up and  running), then
 * macro MPC_SHM_KEY must also be set  to another unused key agreed by
 * the MPC controller and the plant.
 */

#ifndef _MPC_INTERFACE_H_
#define _MPC_INTERFACE_H_

#include <semaphore.h>

#define MPC_SHM_KEY 0xC1A0  /* hard-coded key of shared mem SysV object */
#define MPC_SHM_FLAGS 0666  /* we go easy: everybody reads and writes */

#define MPC_STATE_NUM 12
#define MPC_INPUT_NUM 4

#define MPC_SEM_NUM            2
#define MPC_SEM_STATE_WRITTEN  0   /* +1: plant; -1 MPC controller */
#define MPC_SEM_INPUT_WRITTEN  1   /* +1: MPC controller; -1 plant */

struct shared_data {
	sem_t  sems[MPC_SEM_NUM];    /* semaphores to regulate communication */
	double state[MPC_STATE_NUM]; /* UAV -> MPC controller */
	double input[MPC_INPUT_NUM]; /* MPC controller -> UAV */
};

#endif /* _MPC_INTERFACE_H_ */
