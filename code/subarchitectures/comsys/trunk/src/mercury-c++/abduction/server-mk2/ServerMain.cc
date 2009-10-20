#include <iostream>

#include <unistd.h>

using namespace std;

#define PIPE_READ   0
#define PIPE_WRITE  1

int
main(int argc, void ** argv)
{
	int pipe_to_child[2];
	int pipe_from_child[2];

	pipe(pipe_to_child);
	pipe(pipe_from_child);

	pid_t pchild;

	if ((pchild = fork()) == 0) {
		// child
		close(STDOUT_FILENO);
		dup(pipe_from_child[PIPE_WRITE]);

		close(STDIN_FILENO);
		dup(pipe_to_child[PIPE_READ]);

		close(pipe_to_child[0]);
		close(pipe_to_child[1]);
		close(pipe_from_child[0]);
		close(pipe_from_child[1]);

		execlp("sh", "sh", "-c", "echo here we go ; tr a-z A-Z", NULL);
		//execvp("cat", NULL);
		perror("Exec failed");
	}
	else {
		// parent
		close(STDOUT_FILENO);
		dup(pipe_to_child[PIPE_WRITE]);

		close(STDIN_FILENO);
		dup(pipe_from_child[PIPE_READ]);

		close(pipe_to_child[0]);
		close(pipe_to_child[1]);
		close(pipe_from_child[0]);
		close(pipe_from_child[1]);

		cout << "sending this to the child" << endl;

		close(STDOUT_FILENO);

		char buf[4096];
		while (cin) {
			cin.getline(buf, 4096);
			if (*buf) {
				cerr << buf << endl;
			}
		}

		wait(0);
	}

	return EXIT_SUCCESS;
}
