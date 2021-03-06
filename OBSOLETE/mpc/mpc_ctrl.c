#define USE_SERVER
#define PRINT_LOG

#ifdef USE_SERVER
#define SOLVER_IP "127.0.0.1"  /* must be IP address of the MPC server */
#define SOLVER_PORT 6001     /* must be the same of the MPC server */
#define CLIENT_SOLVER
#endif /* USE_SERVER */
/*
 * Below are some #define which trigger something:
 *
 * CLIENT_SOLVER, expect UDP messages from a solver of the format as
 * in the struct mpc_status defined in mpc.h
 *
 * PRINT_LOG, print log information every time 
 *
 * PRINT_PROBLEM, print the problem formulation in txt files
 *
 * DEBUG_SIMPLEX, turn on all Simplex messages for debugging
 *
 * PRINT_MAT, print matrices (dont remember really how much stuff is
 * printed)
 *
 * USE_SERVER, offload to a server
 */
/*
#define CLIENT_SOLVER
#define PRINT_LOG
#define PRINT_PROBLEM
#define DEBUG_SIMPLEX
#define PRINT_MAT
#define USE_SERVER
*/

/*
 * argv[1], file descr of the pipe read end (where the states are recv)
 * argv[2], file descr of the pipe write end (where the inpute are sent)
 * argv[3], filename of the JSON file of the model
 */

#include <stdlib.h>
#include <limits.h> 
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#ifdef USE_SERVER
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif /* USE_SERVER */
#include "../src/quadrotor_sim/enrico_mpc2_pk/src/common.h"
#include "mpc.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/*
 * Initializing the model with JSON file
 */
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in);



int main(int argc, char * argv[]) {
	int model_fd;
	char * buffer;
	ssize_t size, num_bytes;
	size_t i;

	struct json_object *model_json;
	struct json_tokener * tok;

	/* Messages sent/received */
	struct msg_to_mpc   msg_recv;
	struct msg_from_mpc msg_sent;

	mpc_status * mpc_st;
	int fd_rd, fd_wr;
	mpc_glpk uav_mpc;
#ifdef USE_SERVER
	int sockfd;
	struct sockaddr_in servaddr;
	int steps_serv;
	double time_serv;
#endif
#ifdef PRINT_PROBLEM
	char s_sol[100] = SOL_FILENAME;
	char tmp[100];
#endif


	if (argc <= 3) {
		PRINT_ERROR("Too few arguments. 3 needed: <read fd> <write fd> <JSON model>");
		return -1;
	}

	/* Getting file descr of the pipe connecting to ROS */
	fd_rd = atoi(argv[1]);
	fd_wr = atoi(argv[2]);

	/* Reading the JSON file with the problem model */
	if ((model_fd = open(argv[3], O_RDONLY)) == -1) {
		PRINT_ERROR("Missing/wrong file");
		return -1;
	}
	/* Getting the size of the file */
	size = lseek(model_fd, 0, SEEK_END);
	lseek(model_fd, 0, SEEK_SET);

	/* Allocate the buffer and store data */
	buffer = malloc((size_t)size);
	size = read(model_fd, buffer, (size_t)size);
	close(model_fd);
	tok = json_tokener_new();
	model_json = json_tokener_parse_ex(tok, buffer, (int)size);
	free(buffer);

	/* Initializing the model */
	model_mpc_startup(&uav_mpc, model_json);
#ifdef PRINT_PROBLEM
	glp_print_sol(uav_mpc.op, "000glpk_sol.txt");
#endif

	/* 
	 * Checking consistency between the  problem size got from the
	 * JSON file and messages size defined in common.h
	 */
	if (STATE_NUM != uav_mpc.model->n) {
		PRINT_ERROR("Size of state mismatch");
		return -1;
	}
	if (INPUT_NUM != uav_mpc.model->m) {
		PRINT_ERROR("Size of input mismatch");
		return -1;
	}

	/* Allocating struct of solver status after problem defined */
	mpc_st = mpc_status_alloc(&uav_mpc);
	  
#ifdef PRINT_PROBLEM
	/* Save initial status */
	mpc_status_save(&uav_mpc, mpc_st);
	fprintf(stdout, "Initial status\n");
	mpc_status_fprintf(stdout, &uav_mpc, mpc_st);
 	glp_write_lp(uav_mpc.op, NULL, "initial_mpc.txt");
	glp_print_sol(uav_mpc.op, "initial_sol.txt");
#endif

#ifdef USE_SERVER
	/* Setting up the client: server to connect to */
	bzero(&servaddr, sizeof(servaddr)); 
	servaddr.sin_addr.s_addr = inet_addr(SOLVER_IP);
	servaddr.sin_port = htons(SOLVER_PORT);
	servaddr.sin_family = AF_INET;
      
	/* create and connect UPD socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
	if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
		PRINT_ERROR("client: error in connect");
#endif

	/* Cycling forever to get the state from ROS */
	while ((num_bytes = read(fd_rd, &msg_recv, sizeof(msg_recv)))) {
#ifdef PRINT_LOG
		/* Printing received message */
		printf("%s: [%f] Got message from %i, job %i\n",
		       __FILE__,
		       (double)msg_recv.timestamp.tv_sec
		       +(double)msg_recv.timestamp.tv_sec*1e-9,
		       msg_recv.sender, msg_recv.job_id);
#endif /* PRINT_LOG */
		
		/* Store the state received to the problem status */
		mpc_status_save(&uav_mpc, mpc_st);
		memcpy(mpc_st->state, msg_recv.state,
		       sizeof(*msg_recv.state)*STATE_NUM);
#ifdef USE_SERVER
		*mpc_st->steps_bdg = 1000;   /* number of max iterations */
		*mpc_st->time_bdg = 1000; /* max seconds (large value) */
		*mpc_st->prim_stat = GLP_INFEAS;  /* changing x0 makes primal undef */
		*mpc_st->dual_stat = GLP_FEAS;    /* should always be dual feasible */
		
		/* Sending/receiving status to/from server */
		send(sockfd, mpc_st->block, mpc_st->size, 0);
		recv(sockfd, mpc_st->block, mpc_st->size, 0);
		
		/* Storing server used budgets */
		steps_serv = *mpc_st->steps_bdg;
		time_serv  = *mpc_st->time_bdg; 
		
#ifdef PRINT_PROBLEM
		sprintf(tmp, "%02luA", k);
		strcat(tmp, s_sol);
		glp_print_sol(uav_mpc.op, tmp);
#endif
#ifdef PRINT_LOG
		/* Printing the status after the local computations */
		printf("%s: [%f] MPC server took: %d steps, %f secs\n",
		       __FILE__,
		       (double)msg_recv.timestamp.tv_sec
		       +(double)msg_recv.timestamp.tv_sec*1e-9,
		       steps_serv, time_serv);
#endif /* PRINT_LOG */
#endif /* USE SERVER */
		/* giving ourself "infinite" time/iterations */
		*mpc_st->steps_bdg = INT_MAX;
		*mpc_st->time_bdg  = INT_MAX;
		mpc_status_resume(&uav_mpc, mpc_st);
#ifdef PRINT_PROBLEM
		sprintf(tmp, "%02luB", k);
		strcat(tmp, s_sol);
		glp_print_sol(uav_mpc.op, tmp);
#endif
		glp_factorize(uav_mpc.op); /* INVESTIGATE: DONT KNOW IF USEFUL */
		glp_simplex(uav_mpc.op, uav_mpc.param);
		
#ifdef PRINT_PROBLEM
		sprintf(tmp, "%02luC", k);
		strcat(tmp, s_sol);
		glp_print_sol(uav_mpc.op, tmp);
		mpc_status_save(&uav_mpc, mpc_st);
		mpc_status_fprintf(stdout, &uav_mpc, mpc_st);
#endif	

		/*
		 * There is  a more efficient way  to make the steps  below by
		 * something using
		 *
		 *   mpc_status_save(&uav_mpc, mpc_st);
		 */
	
		/* Getting the solution */
		for (i = 0; i < INPUT_NUM; i++) {
			msg_sent.input[i] =
				glp_get_col_prim(uav_mpc.op,
						 uav_mpc.v_U+(int)i);
		}
		
		/* Adding also PID and timestamp */
		msg_sent.sender = getpid();
		clock_gettime(CLOCK_MONOTONIC, &(msg_sent.timestamp));

		/* Sending the input to ROS */
		write(fd_wr, &msg_sent, sizeof(msg_sent));
	}
}


	
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in)
{

	/* Cleanup the MPC struct */
	bzero(mpc,sizeof(*mpc));
	
	/* Init the solver control parameters */
	mpc->param = malloc(sizeof(*(mpc->param)));
	glp_init_smcp(mpc->param);
#ifdef DEBUG_SIMPLEX
	mpc->param->msg_lev = GLP_MSG_DBG; /* all messages */
#else
	mpc->param->msg_lev = GLP_MSG_OFF; /* no message */
#endif
	mpc->param->meth    = GLP_DUAL;    /* dual simplex */
#if 1
	mpc->param->it_lim  = INT_MAX;     /* max num of iterations */
#else
	mpc->param->it_lim  = 2;     /* max num of iterations */
#endif

	/* Initialize the plant */
	mpc->model = malloc(sizeof(*(mpc->model)));
	dyn_init_discrete(mpc->model, in);

	/* Setting up a GLPK problem instance */
	mpc->op = glp_create_prob();
	glp_set_prob_name(mpc->op, "Model Predictive Control");

	/* Setting up variables and bounds of control inputs */
	mpc_input_addvar(mpc, in);
	mpc_input_set_bnds(mpc, in);

	/* Setting up constraints: bounding input variation */
	/*	mpc_input_set_delta(mpc, in); */

	/* Add a variable for each norm of states X(1), ..., X(H)*/
	mpc_state_norm_addvar(mpc, in);
	
	/* Setting bounds to the states X(1), ..., X(H)*/
	mpc_state_set_bnds(mpc, in);
	
	/* Set a minimization cost for the MPC */
	mpc_goal_set(mpc, in);

	mpc_warmup(mpc);

	return 0;
}
