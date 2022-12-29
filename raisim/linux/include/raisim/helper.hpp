//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_HELPER_HPP
#define RAISIM_HELPER_HPP

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max
#else
#include <unistd.h>
#include <mm_malloc.h>
#endif

#include <vector>
#include <stdexcept>
#include <cstddef>
#include <chrono>
#include "raisim_message.hpp"
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "raisim/Path.hpp"


namespace raisim {

static inline std::string separator() {
#ifdef _WIN32
  return "\\";
#else
  return "/";
#endif
}

#ifdef _WIN32

static inline uint64_t GetPerfFrequency() {
  ::LARGE_INTEGER freq;
  ::QueryPerformanceFrequency(&freq);
  return freq.QuadPart;
}

static inline uint64_t PerfFrequency() {
  static uint64_t xFreq = GetPerfFrequency();
  return xFreq;
}

static inline uint64_t PerfCounter() {
  ::LARGE_INTEGER counter;
  ::QueryPerformanceCounter(&counter);
  return counter.QuadPart;
}

static uint64_t NowInUs() {
  return static_cast<uint64_t>(
      static_cast<double>(PerfCounter()) * 1000000 / PerfFrequency());
}

#endif

static inline void MSLEEP(int sleepMs)
{
#ifdef _WIN32
  auto start = NowInUs();
  while ((NowInUs() - start) < sleepMs * 1000) {
  }
#else
  usleep(sleepMs * 1000);   // usleep takes sleep time in us (1 millionth of a second)
#endif
}

static inline void USLEEP(int sleepMs)
{
#ifdef _WIN32
  auto start = NowInUs();
  while ((NowInUs() - start) < sleepMs) {
  }
#else
  usleep(sleepMs);   // usleep takes sleep time in us (1 millionth of a second)
#endif
}



/* returns the file name without extension */
inline std::string getBaseFileName(const std::string& fullPath) {
  std::string extension = fullPath.substr(fullPath.find_last_of('.') + 1);
  return fullPath.substr(fullPath.find_last_of(Path::separator()) + 1,
                     fullPath.size() - fullPath.find_last_of(Path::separator()) - extension.size() - 2);
}

/* returns only the name of the directory one directory higher than the directory of the file */
inline std::string getTopDirectory(const std::string& fullPath) {
  size_t lastSep = fullPath.find_last_of(Path::separator());
  size_t secondLastSep = fullPath.substr(0, lastSep).find_last_of(Path::separator());
  return fullPath.substr(secondLastSep+1, lastSep-secondLastSep-1);
}

/* returns the file name including the extension */
inline std::string getFileName(const std::string& fullPath) {
  return fullPath.substr(fullPath.find_last_of(Path::separator()) + 1);
}

/* returns only the file extension */
inline std::string getFileExtension(const std::string& fullPath) {
  return fullPath.substr(fullPath.find_last_of('.') + 1);
}

/* returns the full path containing the file (remove the file name in the full path) */
inline std::string getPathName(const std::string &fullPath) {
  size_t i = fullPath.rfind(Path::separator(), fullPath.length());
  if (i != std::string::npos)
    return (fullPath.substr(0, i));

  return ("");
}

/* returns true if the file exists, returns false if not */
inline bool fileExists (const std::string& name) {
  std::ifstream f(name.c_str());
  return f.good();
}

inline bool directoryExists (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

class TimedLoop {
 public:
  TimedLoop(uint64_t loop_time_in_us) {
    loopTimeInUs = loop_time_in_us;
    begin = std::chrono::steady_clock::now();
  }

  ~TimedLoop() {
    auto end = std::chrono::steady_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    if (int(loopTimeInUs) > microseconds)
      USLEEP(int(loopTimeInUs - microseconds));
  }

  std::chrono::steady_clock::time_point begin;
  uint64_t loopTimeInUs;
};

#define RS_TIMED_LOOP(US) auto raisim_arb_variable_name123 = raisim::TimedLoop(US);

void read_png_file(const char *file_name, int &width, int &height, std::vector<double> &values, double scale, double zOffset);

template<typename T, std::size_t RSALOCATOR_ALIGNMENT>
class AlignedAllocator {
 public:

  typedef T *pointer;
  typedef const T *const_pointer;
  typedef T &reference;
  typedef const T &const_reference;
  typedef T value_type;
  typedef std::size_t size_type;
  typedef ptrdiff_t difference_type;

  T *address(T &r) const {
    return &r;
  }

  const T *address(const T &s) const {
    return &s;
  }

  [[nodiscard]] std::size_t max_size() const {
    return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T);
  }

  template<typename U>
  struct rebind {
    typedef AlignedAllocator<U, RSALOCATOR_ALIGNMENT> other;
  };

  bool operator!=(const AlignedAllocator &other) const {
    return !(*this == other);
  }

  void construct(T *const p, const T &t) const {
    void *const pv = static_cast<void *>(p);

    new(pv) T(t);
  }

  void destroy(T *const p) const {
    p->~T();
  }

  bool operator==(const AlignedAllocator &other) const {
    return true;
  }

  AlignedAllocator() = default;

  AlignedAllocator(const AlignedAllocator &) = default;

  template<typename U>
  AlignedAllocator(const AlignedAllocator<U, RSALOCATOR_ALIGNMENT> &) {}

  ~AlignedAllocator() = default;

  T *allocate(const std::size_t n) const {
    if (n == 0)
      return nullptr;

    void *const pv = _mm_malloc(n * sizeof(T), RSALOCATOR_ALIGNMENT);

    if (pv == nullptr) {
      throw std::bad_alloc();
    }

    return static_cast<T *>(pv);
  }

  void deallocate(T *const p, const std::size_t n) const {
    _mm_free(p);
  }

  template<typename U>
  T *allocate(const std::size_t n, const U * /* const hint */) const {
    return allocate(n);
  }

 private:
  AlignedAllocator &operator=(const AlignedAllocator &);
};

struct ColorRGB {
  uint8_t r,g,b;
};

struct ColorRGBA {
  uint8_t r,g,b,a;
};

}

#endif //RAISIM_HELPER_HPP