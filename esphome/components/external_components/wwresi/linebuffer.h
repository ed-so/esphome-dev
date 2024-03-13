/** \file Linebuffer.h buffer and split text line oriented messages
 */
#pragma once

#include <iostream>
#include <sstream>
#include <list>

#include "esphome/core/defines.h"

//#include "wwresi.h"

namespace esphome {
namespace wwresi {

typedef std::list<class Linebuffer *> t_Linebuffer_List;

typedef int t_HANDLER(class Linebuffer *stream, std::string &line);

class Linebuffer : public std::stringstream {
 private: 
  int debug = 1; 
 public:
  //---------------------------------------------------------------------------
  Linebuffer(int fd) {
    m_fd = fd;
    m_handler = NULL;
    m_flags = F_ECHO_DEF;
  }

  //---------------------------------------------------------------------------
  Linebuffer(int fd, t_HANDLER *handler) {
    m_fd = fd;
    m_handler = handler;
    m_flags = F_ECHO_DEF;
  }

  //---------------------------------------------------------------------------
  /// @brief destructor
  ~Linebuffer() {}


  std::string Line = "";
  enum bufstate { EMPTY, COMPLETE, INCOMPLETE };
  enum flags { F_ECHO_DIN = 1, F_ECHO_OTHER = 2, F_ECHO_INDX = 4, F_ECHO_DEF = 0 };

  //---------------------------------------------------------------------------
  /// @brief add string to stream and call Process....
  /// @param data 
  void AddData(const std::string data) {
    *this << (data.c_str());
    ProcessData(NULL);
  }


  //---------------------------------------------------------------------------
  /// @brief if data complete call handler
  /// @param handler 
  /// @return 
  int ProcessData(t_HANDLER *handler) {
    int cnt = 0;
    if (debug > 1) {
      ESP_LOGD("wwresi", "cnt_b  PD  %d", cnt);
    }
    if (handler == NULL) {
      handler = m_handler;
    }

    std::string line = Line = "";

    bufstate bf = EMPTY;

    while (GetData(line) == COMPLETE) {
      cnt++;
      if (debug > 1) {
        ESP_LOGD("wwresi", "data: PD0  %s", line.c_str());
      }
      Line = line;
      if (handler)
        handler(this, line);
    }
    if (debug > 1) {
      ESP_LOGD("wwresi", "cnt_e  PDx  %d - line %s", cnt, Line.c_str());
    }
    return cnt;
  }

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
      if (debug > 1) {
        ESP_LOGD("wwresi", "data: GD11  %s", line.c_str());
      }
      if (eof()) {
        // new line character not present, line was not complete, return it to the buffer
        clear();  // reset error (eof) status
        str("");  // free internal buffer
        // put back first characters of incoming not complete line
        //*this << line;
        write(line.data(), line.size());
        if (debug > 1) {
          ESP_LOGD("wwresi", "data: GD12  %s", "INCOMPLETE");
        }
        line = "";
        return INCOMPLETE;
      }
      if (debug > 1) {
        ESP_LOGD("wwresi", "data: GD13  %s", "COMPLETE");
      }
      return COMPLETE;
    }
    clear();  // reset error (eof) status
    str("");  // free internal buffer
    if (debug > 1) {
      ESP_LOGD("wwresi", "data: GD2  %s", "EMPTY");
    }
    line = "";
    return EMPTY;
  }

  t_HANDLER *m_handler;
  int m_fd;
  int m_flags;
};

}  // namespace wwresi
}  // namespace esphome

// int printall(class Line_Buffer * stream, string & line);
