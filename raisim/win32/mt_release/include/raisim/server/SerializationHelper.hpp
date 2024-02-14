//----------------------------//
// This file is part of RaiSim//
// Copyright 2022, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_
#define RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_

#include <vector>
#include <string>
#include <cstring>

#include "raisim/math.hpp"

namespace raisim {
namespace server {

template<typename T>
static inline char *set(char *data, T val) {
  T temp = val;
  memcpy(data, &temp, sizeof(T));
  return data + sizeof(T);
}

template<int N>
static inline char *set(char *data, const Vec<N> &vec) {
  memcpy(data, vec.v, sizeof(double) * N);
  return data + sizeof(double) * N;
}
template<typename T>
static inline char *setN(char *data, T* val, int64_t N) {
  memcpy(data, val, sizeof(T) * N);
  return data + sizeof(T) * N;
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


static inline char *setInFloat(char *data, double val) {
  auto temp = float(val);
  memcpy(data, &temp, sizeof(float));
  return data + sizeof(float);
}

static inline char *set(char *data, const std::string &val) {
  data = set(data, (int32_t) (val.size()));
  memcpy(data, val.data(), sizeof(char) * val.size());
  return data + sizeof(char) * val.size();
}

static inline char *setInFloat(char *data, const std::vector<double>& val) {
  data = set(data, (int32_t) (val.size()));
  for (int i = 0; i < val.size(); i++)
    data = set(data, float(val[i]));

  return data;
}

template<typename T>
static inline char *setInFloat(char *data, const T& val) {
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
static inline char *getInFloat(char *data, T *val) {
  float temp;
  for (int i = 0; i < T::size(); i++) {
    data = get(data, &temp);
    (*val)[i] = double(temp);
  }
  return data;
}

static inline char *getInFloat(char *data, double *val) {
  float temp;
  memcpy(&temp, data, sizeof(float));
  *val = temp;
  return data + sizeof(float);
}

template<typename T>
static inline char *getN(char *data, T *val, int32_t N) {
  memcpy(val, data, sizeof(T) * N);
  data += sizeof(T) * N;
  return data;
}

static inline char *get(char *data, std::string *str) {
  uint64_t size;
  data = get(data, &size);
  str->resize(size);
  data = getN(data, const_cast<char *>(str->c_str()), size);
  return data;
}

template<typename T>
static inline char *set(char *data, const std::vector<T> &str) {
  data = set(data, int32_t(str.size()));
  data = setN(data, str.data(), int32_t(str.size()));
  return data;
}

static inline char *set(char *data, const std::vector<std::string> &str) {
  data = set(data, int32_t(str.size()));
  for (auto& s: str)
    data = set(data, s);
  return data;
}

static inline char *set(char *data, const VecDyn &vec) {
  data = set(data, int32_t(vec.size()));
  data = setN(data, vec.ptr(), int32_t(vec.size()));
  return data;
}

static inline char *set(char *data, const std::vector<VecDyn> &vecs) {
  data = set(data, int32_t(vecs.size()));
  for (auto& v : vecs) data = set(data, v);
  return data;
}

template<typename T>
static inline char *setInFloat(char *data, const std::vector<double> &str) {
  data = set(data, int32_t(str.size()));
  for (auto v : str)
    data = set(data, float(v));
  return data;
}

template<typename T>
static inline char *get(char *data, std::vector<T> *vec) {
  int32_t size;
  data = get(data, &size);
  vec->resize(size);
  data = getN(data, &vec[0], (int32_t) (vec->size()));
  return data;
}

template<typename T>
static inline char *set(char *data, const std::vector<std::string> &val) {
  data = set(data, int32_t(val.size()));
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

template<typename T, typename... Types>
static inline char *get(char *data, T *arg, Types... args) {
  return get(get(data, arg), args...);
}

template<typename T, typename... Types>
static inline char *getInFloat(char *data, T *arg, Types... args) {
  return getInFloat(getInFloat(data, arg), args...);
}

}
}

#endif // RAISIM_INCLUDE_RAISIM_SERIALIZATIONHELPER_HPP_
