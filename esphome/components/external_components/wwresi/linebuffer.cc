/* * \file line-buffer.cc buffer and split text line oriented messages

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

#include "linebuffer.h"

#ifdef _TEST_LIBRARY

int printall(class Linebuffer *stream, string &line) {
  cerr << ":" << stream->m_fd << ":" << line << ";\n";
  return 0;
}


int main(int argc, char *argv[]) {
  string re;
  {
    int i;
    Linebuffer buf1(2, printall);

    for (i = 0; i < 100000; i++) {
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
