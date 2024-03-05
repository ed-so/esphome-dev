/** \file line-buffer.cc buffer and split text line oriented messages

DESIGN:
each connection should have an buffer object associated with it. It will be based on stringstream
object but with added information about socket and event handler.
Data model is FIFO - on input data coming as strings, lines split at any place.
on output only reading complete lines possible.
When event handler is not null it will be called automatically by AddData().
New methods: AddData(const string & data) - appends data to FIFO, optionally calls ProcessData
ProcessData() - if event handler defined, call it with lines read from FIFO
GetData(string &line) - try to read single line.

*/

#include "line-buffer.h"








/// @brief 
/// @param fd 
/// @param handler 
LineBuffer::LineBuffer(int fd, t_HANDLER * handler) 
	: stringstream(stringstream::in | stringstream::out) {
	m_fd = fd;
	m_handler = handler;
	m_flags = F_DEF;
}

void LineBuffer::AddData(const char *data) {
	*this << data;
	ProcessData();
}

int LineBuffer::ProcessData(t_HANDLER *handler) {
	//bool ret;
	int cnt = 0;
	
	if(handler == NULL) {
		handler = m_handler;
	}
	
	string line;
	
	while(GetData(line) == COMPLETE) {
		cnt++;
		if(handler)
			handler(this, line);
	}

	return cnt;
}

/** get complete lines out of the buffer.
	every time this function returns EMPTY or INCOMPLETE internal buffer
	is guaranteed to be freed. This combined with while(GetData(line) == COMPLETE)
	above makes sure there are no memory leaks.
*/
LineBuffer::bufstate LineBuffer::GetData(string & line) {

	if(std::getline(*this,line)) {
		if(eof()) {
			// new line character not present, line was not complete, return it to the buffer
			clear(); // reset error (eof) status
			str(""); // free internal buffer
			// put back first characters of incoming not complete line
			//*this << line;
			write(line.data(), line.size());
			return INCOMPLETE;
		}
		return COMPLETE;
	}
	clear(); // reset error (eof) status
	str(""); // free internal buffer
	return EMPTY;
}

int printall(class LineBuffer * stream, string & line) {

	cerr << ":" << stream->m_fd << ":" << line << ";\n";
	return 0;
}

#ifdef _TEST_LIBRARY

int main(int argc, char* argv[]) {
	string re;
	{
		int i;
		LineBuffer buf1(2,printall);

		for(i=0; i<100000; i++) {
			cerr << "+";
			buf1.AddData("<11111");
			cerr << "+";
			buf1.AddData("222>\n<");
			cerr << "+";
			buf1.AddData("333>\n<4444");
			cerr << "+";
			buf1.AddData("5555>\n<5a5a5a5a>\n<5b5b5b5b>\n<5c5c5c5c5c5c5c5c>\n<5d5d");
			cerr << "+";
			buf1.AddData("666>\n");
			cerr << "=";
		}
		cerr << "DONE\n";
 		cin >> re;
	}
	cerr << "free?\n";
 	cin >> re;

	return 0;
}
#endif

