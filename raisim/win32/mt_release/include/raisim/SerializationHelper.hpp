//----------------------------//
// This file is part of RaiSim//
// Copyright 2022, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_
#define RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_

#include <vector>
#include <string>
#include <cstring>

#include "math.hpp"

namespace raisim {
namespace server {

template<typename T>
static inline char *set(char *data, T *val) {
  memcpy(data, val, sizeof(T));
  return data + sizeof(T);
}

template<typename T>
static inline char *setN(char *data, T *val, int64_t N) {
  memcpy(data, val, sizeof(T) * N);
  return data + sizeof(T) * N;
}

template<int N>
static inline char *set(char *data, const Vec<N> &vec) {
  memcpy(data, vec.v, sizeof(double) * N);
  return data + sizeof(double) * N;
}

template<int N>
static inline char *setInFloat(char *data, const Vec<N> &vec) {
  float val;
  for (int i=0; i<N; i++) {
    val = float(vec[i]);
    memcpy(data, &val, sizeof(float));
    data += sizeof(float);
  }
  return data;
}

template<int N>
static inline char *set(char *data, const Vec<N> &&vec) {
  Vec<N> temp = vec;
  memcpy(data, temp.v, sizeof(double) * N);
  return data + sizeof(double) * N;
}

template<typename T>
static inline char *set(char *data, T val) {
  T temp = val;
  memcpy(data, &temp, sizeof(T));
  return data + sizeof(T);
}

static inline char *setInFloat(char *data, double val) {
  float temp = val;
  memcpy(data, &temp, sizeof(float));
  return data + sizeof(float);
}

static inline char *set(char *data, const std::string &val) {
  data = set(data, (uint64_t) (val.size()));
  memcpy(data, val.data(), sizeof(char) * val.size());
  return data + sizeof(char) * val.size();
}

static inline char *setInFloat(char *data, const std::vector<double>& val) {
  data = set(data, (uint64_t) (val.size()));
  for (int i = 0; i < val.size(); i++)
    data = set(data, float(val[i]));

  return data;
}

template<typename T>
static inline char *setInFloat(char *data, T val) {
  for (int i = 0; i < val.size(); i++)
    data = set(data, float(val[i]));

  return data;
}

template<typename T>
static inline char *get(char *data, T *val) {
  memcpy(val, data, sizeof(T));
  return data + sizeof(T);
}

template<typename T>
static inline char *getN(char *data, T *val, uint64_t N) {
  memcpy(val, data, sizeof(T) * N);
  data += sizeof(T) * N;
  return data;
}

static inline char *getString(char *data, std::string &str) {
  uint64_t size;
  data = get(data, &size);
  str.resize(size);
  data = getN(data, const_cast<char *>(str.c_str()), size);
  return data;
}

template<typename T>
static inline char *set(char *data, const std::vector<T> &str) {
  data = set(data, int64_t(str.size()));
  data = setN(data, str.data(), uint64_t(str.size()));
  return data;
}

static inline char *set(char *data, const std::vector<std::string> &str) {
  data = set(data, int64_t(str.size()));
  for (auto& s: str)
    data = set(data, s);
  return data;
}

static inline char *set(char *data, const VecDyn &vec) {
  data = set(data, int64_t(vec.size()));
  data = setN(data, vec.ptr(), vec.size());
  return data;
}

static inline char *set(char *data, const std::vector<VecDyn> &vecs) {
  data = set(data, int64_t(vecs.size()));
  for (auto& v : vecs) data = set(data, v);
  return data;
}

template<typename T>
static inline char *setInFloat(char *data, const std::vector<double> &str) {
  data = set(data, int64_t(str.size()));
  for (auto v : str)
    data = set(data, float(v));
  return data;
}

template<typename T>
static inline char *getStdVector(char *data, std::vector<T> &str) {
  uint64_t size;
  data = get(data, &size);
  str.resize(size);
  data = getN(data, str.data(), (uint64_t) (str.size()));
  return data;
}

template<typename T>
static inline char *set(char *data, const std::vector<std::string> &val) {
  data = set(data, uint64_t(val.size()));
  for (auto &v : val)
    data = set(data, v);
  return data;
}

template<typename T, typename... Types>
static inline char *set(char *data, const T &arg, Types... args) {
  return set(set(data, arg), args...);
}

template<typename T, typename... Types>
static inline char *setInFloat(char *data, const T &arg, Types... args) {
  return setInFloat(setInFloat(data, arg), args...);
}
}
}

#endif // RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_
