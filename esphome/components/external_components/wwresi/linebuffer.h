/** \file Linebuffer.h buffer and split text line oriented messages
 */
#pragma once

#include <iostream>
#include <sstream>
#include <list>

#include "esphome/core/defines.h"

#include "wwresi.h"

namespace esphome {
namespace wwresi {

typedef std::list<class Linebuffer *> t_Linebuffer_List;

typedef int t_HANDLER(class Linebuffer *stream, std::string &line);

class Linebuffer : public std::stringstream {
 public:
  Linebuffer(int fd) {
    m_fd = fd;
    m_handler = NULL;
    m_flags = F_DEF;
  }

  Linebuffer(int fd, t_HANDLER *handler) {
    m_fd = fd;
    m_handler = handler;
    m_flags = F_DEF;
  }

  ~Linebuffer() {}

  enum bufstate { EMPTY, COMPLETE, INCOMPLETE };
  enum flags { F_ECHO_DIN = 1, F_ECHO_OTHER = 2, F_ECHO_INDX = 4, F_DEF = 0 };

  void AddData(const std::string data) {
    *this << (data.c_str());

    // add = '-';

    // ESP_LOGD("wwresi", "data: add1 %s", add.c_str());

    // *this  >> add;
    
    // ESP_LOGD("wwresi", "data: add2 %s", add.c_str());

    // add = '-';

    // *this  >> add;
    
    // ESP_LOGD("wwresi", "data: add3 %s", add.c_str());

    ProcessData(NULL);
  }

  int ProcessData(t_HANDLER *handler) {
    int cnt = 0;
    ESP_LOGD("wwresi", "cnt_b  PD  %d", cnt);
    if (handler == NULL) {
      handler = m_handler;
    }
    std::string line = "---";

    bufstate bf = EMPTY;

    while (GetData(line) == COMPLETE) {
      cnt++;
      ESP_LOGD("wwresi", "data: PD  %s", line.c_str());
      if (handler)
        handler(this, line);
    }
    ESP_LOGD("wwresi", "cnt_e  PD  %d - line %s", cnt, line.c_str());
    return cnt;
  }

  // void virtual AddData(const char *data);
  // int virtual ProcessData(t_HANDLER handler = NULL);

  // enum bufstate GetData(std::string &line);
  /** get complete lines out of the buffer.
    every time this function returns EMPTY or INCOMPLETE internal buffer
    is guaranteed to be freed. This combined with while(GetData(line) == COMPLETE)
    above makes sure there are no memory leaks.
  */
  bufstate GetData(std::string &line) {
    // std::string tst;
    // *this >> tst;

    // ESP_LOGD("wwresi", "data: GD0  %s", tst.c_str());
    // ESP_LOGD("wwresi", "data: GD1  %s", line.c_str());

    if (std::getline(*this, line)) {
      ESP_LOGD("wwresi", "data: GD11  %s", line.c_str());
      if (eof()) {
        // new line character not present, line was not complete, return it to the buffer
        clear();  // reset error (eof) status
        str("");  // free internal buffer
        // put back first characters of incoming not complete line
        //*this << line;
        write(line.data(), line.size());
        ESP_LOGD("wwresi", "data: GD12  %s", "INCOMPLETE");
        return INCOMPLETE;
      }
      ESP_LOGD("wwresi", "data: GD13  %s", "COMPLETE");
      return COMPLETE;
    }
    clear();  // reset error (eof) status
    str("");  // free internal buffer
    ESP_LOGD("wwresi", "data: GD2  %s", "EMPTY");
    return EMPTY;
  }

  t_HANDLER *m_handler;
  int m_fd;
  int m_flags;
};

}  // namespace wwresi
}  // namespace esphome

// int printall(class Line_Buffer * stream, string & line);
