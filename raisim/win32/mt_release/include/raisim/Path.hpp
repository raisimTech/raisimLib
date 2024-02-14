//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#pragma once

#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <vector>

#include "raisim/raisim_message.hpp"

#if WIN32
#else
#include <unistd.h>
#include <limits.h>
#endif

namespace raisim {

class Path {
 public:

  Path(const std::string& path, bool skipPathCheck = false) {
    path_ = path;    
    raisim::Path::replaceAntiSeparatorWithSeparator(path_);
    if(skipPathCheck) return;

    while(true) {
      size_t found = path_.find(upperDir());      
      if(found == std::string::npos)
        break;
      else {
        RSFATAL_IF(found < 3, "\'"<< path <<"\'" << " is an invalid path")        
        size_t dirStarts = path_.substr(0, found-2).rfind(separator());
        path_.erase(dirStarts, found - dirStarts + 2);
      }
    }
    while(true) {
      size_t found = path_.find(separator() + separator());
      if(found == std::string::npos)
        break;
      else
        path_.erase(found, 1);      
    }
  }

  Path operator +(const std::string& additionalPath) {
    if(additionalPath.substr(1) == separator())
      return Path(path_ + additionalPath.substr(1), true);
    else
      return Path(path_ + additionalPath, true);
  }

  Path operator +(const Path& additionalPath) {
    return Path(path_ + separator() + additionalPath.getPath(), true);
  }

  inline static std::string separator() {
  #ifdef WIN32
    return "\\";
  #else
    return "/";
  #endif
  }

  inline static std::string antiSeparator() {
#ifdef WIN32
    return "/";
#else
    return "\\";
#endif
  }

  inline static void replaceAntiSeparatorWithSeparator(
    std::string& string) {
    if (string.empty()) return;

    std::size_t pos;
    while ((pos = string.find(antiSeparator())) != std::string::npos)
      string.replace(pos, antiSeparator().size(), separator());
  }

  inline static std::string upperDir() {
    return "..";
  }

  const std::string& getPath() const { return path_; }
  const std::string& getString() const {return path_; }

  /* returns the file name without extension */
  inline std::string getBaseFileName() {
    std::string extension = path_.substr(path_.find_last_of('.') + 1);
    return path_.substr(path_.find_last_of(separator()) + 1,
                      path_.size() - path_.find_last_of(separator()) - extension.size() - 2);
  }

  /* returns only the name of the directory one directory higher than the directory of the file */
  inline Path getTopDirectory() {
    size_t lastSep = path_.find_last_of(separator());
    size_t secondLastSep = path_.substr(0, lastSep).find_last_of(separator());
    return Path(path_.substr(secondLastSep+1, lastSep-secondLastSep-1));
  }

  /* returns the file name including the extension */
  inline std::string getFileName() {
    return path_.substr(path_.find_last_of(separator()) + 1);
  }

  /* returns only the file extension */
  inline std::string getFileExtension() {
    return path_.substr(path_.find_last_of('.') + 1);
  }

  /* returns the full path to the directory that contains the file (remove the file name in the full path) */
  inline Path getDirectory() {
    size_t i = path_.rfind(separator(), path_.length());
    if (i != std::string::npos)
      return Path(path_.substr(0, i));

    return Path("");
  }

  /* returns true if the file exists, returns false if not */
  inline bool fileExists () {
    std::ifstream f(path_.c_str());
    return f.good();
  }

  inline bool directoryExists () {
    struct stat buffer;
    return (stat (path_.c_str(), &buffer) == 0);
  }

  operator std::string() const { return path_; }

  static Path setFromArgv(char* argv0) {
    /// get binary path
#if WIN32
    std::string basePath(argv0);
    return raisim::Path(basePath);
#else
    char cwd[PATH_MAX];
    RSFATAL_IF(getcwd(cwd, sizeof(cwd))==0, "Could not read directory path.");
    std::string basePath(argv0);
    std::string cwdStr(cwd);

    if(basePath[0]=='.')
      basePath.erase(0, 1);

    auto absolutePath = basePath.find(cwd);
    if(absolutePath == size_t(0))
      return raisim::Path(cwdStr + separator() + basePath.substr(cwdStr.size()));

    return raisim::Path(cwdStr + separator() + basePath);
#endif
  }

 private:
  std::string path_;
};

inline static Path searchForFile(const std::string& filename, const Path& searchRoot, const std::vector<std::string>& hints) {
  Path basePath = searchRoot.getString() + raisim::Path::separator() + filename;
  if(basePath.fileExists()) {
    return basePath;
  } else {
    for (const auto& h : hints) {
      Path hintPath = searchRoot.getString() + raisim::Path::separator() + h + raisim::Path::separator() + filename;
      if(hintPath.fileExists()) return hintPath;
    }
  }
  return Path("");
}
}