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


#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}


#define CMD_LEN 100
extern char ** environ;

/* GLOBAL VARIABLES */
pid_t child_pid;


void term_handler(int signum);
void remove_child_pid(void);

int main(int argc, char * argv[])
{	
	char cmd_str[CMD_LEN], tmp[CMD_LEN];
	struct sigaction sa;

	/* Setting up the Ctrl-C signal handler for termination */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGINT, &sa, NULL);

	if (!(child_pid = fork())) {
		/* CHILD PROCESS TO BE TRACED */
		strncpy(cmd_str, "echo", CMD_LEN);
		snprintf(tmp, CMD_LEN, " %d", getpid());
		strncat(cmd_str, tmp, CMD_LEN);
		strncat(cmd_str, " >> /sys/kernel/tracing/set_event_pid",
			CMD_LEN);
		system(cmd_str);
		execve(argv[1], argv+1, environ);
		PRINT_ERROR("error in execve");
		exit(EXIT_FAILURE);
	}
	
	wait(NULL);
	remove_child_pid();

	return 0;
}

void remove_child_pid(void)
{
	/* 
	 * EB: ideally here we should remove child_pid from the list
	 * of PIDs in /sys/kernel/tracing/set_event_pid. However, no
	 * easy and clean way comes to my mind
	 */
	return;
}


void term_handler(int signum)
{
	kill(child_pid, SIGINT);
	wait(NULL);
	remove_child_pid();
	exit(0);
}
