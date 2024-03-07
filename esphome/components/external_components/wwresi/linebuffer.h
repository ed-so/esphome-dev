/** \file LineBuffer.h buffer and split text line oriented messages
*/
#pragma once

#include <iostream>
#include <sstream>
#include <list>


//using namespace std;

typedef std::list<class linebuffer*> t_linebuffer_List;

typedef int t_HANDLER(class linebuffer * stream, string & line);

class linebuffer : public std::stringstream {
public:
	//Line_Buffer(int fd);
	linebuffer(int fd, t_HANDLER * handler = NULL);
	
	enum bufstate {EMPTY, COMPLETE, INCOMPLETE};
	enum flags {F_ECHO_DIN = 1, F_ECHO_OTHER=2, F_ECHO_INDX=4, F_DEF = 0};
	
	void virtual AddData(const char *data);
	//int virtual ProcessData(t_HANDLER handler = NULL);
	enum bufstate GetData(string & line);

	//t_HANDLER *m_handler;	
	int m_fd;
	int m_flags;
};





//int printall(class Line_Buffer * stream, string & line);


