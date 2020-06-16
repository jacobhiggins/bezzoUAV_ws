#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <sched.h>
#include <unistd.h>
#include <stdint.h>


#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}


#define CMD_LEN 100
extern char ** environ;

/* GLOBAL VARIABLES */
pid_t child_pid;


void term_handler(int signum);
void stop_tracing(void);
void sched_set_prio_affinity(int prio, int cpu_id);

/*
 * Must be launched by superuser
 *
 * ./trace_proc <cpu-id> <prio> <command> <args> 
 *
 * <cpu-id>, CPU where to pin the process
 * <prio>, 0 is highest, 1 is second highest
 * <command>, command to be traced
 * <args-if-any>, command line arguments of <command> if any
 */
int main(int argc, char * argv[])
{	
	char cmd_str[CMD_LEN], tmp[CMD_LEN];
	struct sigaction sa;
	int cpu_id, null_fd, prio;

	/* Checking/Getting command-line arguments */
	if (argc <= 3) {
		PRINT_ERROR("Too few arguments. At least 3 needed. Usage\n\
./trace_proc <cpu-id> <prio> <command> <args-if-any>");
		return -1;
	}
	cpu_id = atoi(argv[1]);
	prio = atoi(argv[2]);
	
	/* Setting up the Ctrl-C signal handler for termination */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGINT, &sa, NULL);

	/**** TRACING: start-up ****/
	/* trace context switches */
	system("echo sched:sched_switch > /sys/kernel/debug/tracing/set_event");
	/* clean the PID list */
	system("echo > /sys/kernel/debug/tracing/set_event_pid");
	/* clean past trace */
	system("echo > /sys/kernel/debug/tracing/trace");

	if (!(child_pid = fork())) {
		
		/* Pinning the process and giving it a fixed prio */
		sched_set_prio_affinity(
			sched_get_priority_max(SCHED_FIFO)-prio, cpu_id);

		/* Redirect all output to /dev/null to reduce overhead */
		if ((null_fd = open("/dev/null", O_WRONLY)) == -1)
			PRINT_ERROR("issue in opening /dev/null");
		if (dup2(null_fd, STDOUT_FILENO) == -1 )
			PRINT_ERROR("issue in duplicating file descr");
		close(null_fd);
		
		/* Add the PID among the ones to be traced */
		strncpy(cmd_str, "echo", CMD_LEN);
		snprintf(tmp, CMD_LEN, " %d", getpid());
		strncat(cmd_str, tmp, CMD_LEN);
		strncat(cmd_str, " >> /sys/kernel/debug/tracing/set_event_pid",
			CMD_LEN);
		system(cmd_str);

		/* enable tracing */
		system("echo 1 > /sys/kernel/debug/tracing/tracing_on");
		execve(argv[3], argv+3, environ);
		PRINT_ERROR("error in execve");
		exit(EXIT_FAILURE);
	}
	
	wait(NULL);
	stop_tracing();

	return 0;
}

void stop_tracing(void)
{

	/**** TRACING: stop ****/
	/* disable tracing */
	system("echo 0 > /sys/kernel/debug/tracing/tracing_on");
	/* trace context switches */
	system("echo > /sys/kernel/debug/tracing/set_event");
	/* clean the PID list */
	system("echo > /sys/kernel/debug/tracing/set_event_pid");
	/* clean past trace */
	system("cp /sys/kernel/debug/tracing/trace .");
	system("echo > /sys/kernel/debug/tracing/trace");
	system("chmod ugo+rw trace");
	return;
}


void term_handler(int signum)
{
	kill(child_pid, SIGINT);
	wait(NULL);
	stop_tracing();
	exit(0);
}

void sched_set_prio_affinity(int prio, int cpu_id)
{
	cpu_set_t  mask;

	/* Set CPU affinity */
	CPU_ZERO(&mask);
	CPU_SET(cpu_id, &mask);
	if (sched_setaffinity(0, sizeof(mask), &mask) != 0) {
		PRINT_ERROR("sched_setaffinity");
		exit(-1);
	}

	/* Set priority */
#if SCHED_SETATTR_IN_SCHED_H
	/* EB: TO BE TESTED */
	struct sched_attr attr;
	
	bzero(&attr, sizeof(attr));
	attr.size = sizeof(attr);
	attr.sched_policy = SCHED_FIFO;
	attr.sched_priority = prio;
	if (sched_setattr(0, &attr, 0) != 0) {
		PRINT_ERROR("sched_setattr");
		exit(-1);
	}
#else
	struct sched_param param;
	
	bzero(&param, sizeof(param));
	param.sched_priority = prio;
	if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
		PRINT_ERROR("sched_setscheduler");
		exit(-1);
	}
#endif
}
