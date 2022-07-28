//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_MESSAGE_LOGGER_HPP
#define RAISIM_MESSAGE_LOGGER_HPP

#ifdef RAISIM_STATIC_API
#undef RAISIM_STATIC_API
#endif

#ifdef WIN32
  #ifdef RAISIM_STATIC_MEMBER_EXPORT
    #define RAISIM_STATIC_API __declspec(dllexport)
  #else
    #define RAISIM_STATIC_API __declspec(dllimport)
  #endif
#else
  #define RAISIM_STATIC_API
#endif

#include <chrono>
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <functional>
#include "math.h"


namespace raisim {

constexpr int RSEVERITY_INFO = 0;
constexpr int RSEVERITY_WARN = 1;
constexpr int RSEVERITY_FATAL = 2;


class RaiSimMsg {
 public:
  void stream(const char *file, const int line, std::stringstream &msg, int severity) {

    const char *filename_start = file;
    const char *filename = filename_start;
    while (*filename != '\0')
      filename++;
    while ((filename != filename_start) && (*(filename - 1) != '/'))
      filename--;

    std::stringstream printout;

    std::string color;

    switch (severity) {
      case 0:
        color = "";
        break;
      case 1:
        color = "\033[33m";
        break;
      case 2:
        color = "\033[1;31m";
        break;
    }

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

#ifdef WIN32
    struct tm timeinfo;
    localtime_s(&timeinfo, &in_time_t);
    std::tm* timePtr = &timeinfo;
#else
    std::tm* timePtr = std::localtime(&in_time_t);
#endif

    printout << "[" << std::setfill('0')
             << std::put_time(timePtr, "%Y:%m:%d:%X")<< ' '
             << std::setfill(' ')
             << filename
             << ':' << line << "] " << color << msg.str() << "\033[0m" << std::endl;

    std::cout << printout.str();

    if (severity == RSEVERITY_FATAL)
      fatalCallback_();
  }

  static void setFatalCallback(std::function<void()> fatalCallback) {
    fatalCallback_ = fatalCallback;
  }

 private:
  std::stringstream log;
  RAISIM_STATIC_API static std::function<void()> fatalCallback_;
};


}

#endif //RAISIM_MESSAGE_LOGGER_HPP
