/** \file line-buffer.h buffer and split text line oriented messages
*/

#include <iostream>
#include <sstream>

using namespace std;

class LineBuffer;

typedef int (t_HANDLER)(class LineBuffer * stream, string & line);

class LineBuffer : public stringstream {
public:
	enum bufstate {EMPTY, COMPLETE, INCOMPLETE};
	enum flags {F_ECHO_DIN = 1, F_ECHO_OTHER=2, F_ECHO_INDX=4, F_DEF = 0};
	LineBuffer(int fd, t_HANDLER * handler = NULL);
	void virtual AddData(const char *data);
	int virtual ProcessData(t_HANDLER handler = NULL);
	enum bufstate GetData(string & line);

	t_HANDLER *m_handler;	
	int m_fd;
	int m_flags;
};

int printall(class LineBuffer * stream, string & line);