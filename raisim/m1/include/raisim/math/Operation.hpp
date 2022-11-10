//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAIMATH__OPERATION_HPP_
#define RAIMATH__OPERATION_HPP_

#include "Expression.hpp"
#include "Matrix.hpp"

namespace raisim {

template<class T1, class T2>
inline double Prod_func(const T1& exp1, const T2&  exp2, size_t i, size_t j) {
  double value = 0.;
  for (size_t idx = 0; idx < exp1.cols(); ++idx)
    value += exp1(i, idx) * exp2(idx, j);
  return value;
}

template<class T1, class T2>
inline double Sum_func(const T1& exp1, const T2&  exp2, size_t i, size_t j) { return exp1(i, j) + exp2(i, j); }

template<class T1, class T2>
inline double Sub_func(const T1& exp1, const T2&  exp2, size_t i, size_t j) { return exp1(i, j) - exp2(i, j); }

template<class T1, class T2>
inline double Div_func(const T1& exp1, const T2&  exp2, size_t i, size_t j) { return exp1(i, j) / exp2(i, j); }

#define RaiSim_define_binary_operator(Operator,OpFunc)                                              \
template<typename T1, typename T2>                                                                  \
class OpFunc##_op : public MatExpr<OpFunc##_op<T1, T2>> {                                           \
 public:                                                                                            \
  using SelfType = OpFunc##_op<T1, T2>;                                                             \
  const T1 &_u;                                                                                     \
  const T2 &_v;                                                                                     \
  inline OpFunc##_op(T1 const &u, T2 const &v) : _u(u), _v(v) {}                                    \
  inline double operator()(size_t i, size_t j) const { return OpFunc##_func(_u, _v, i, j); }        \
  inline double operator()(size_t i) const { return OpFunc##_func(_u, _v, i, 0); }                  \
  inline double operator[](size_t i) const { return OpFunc##_func(_u, _v, i, 0); }                  \
  inline constexpr static size_t size() { return T2::size(); }                                      \
  inline constexpr static size_t cols() { return T2::cols(); }                                      \
  inline constexpr static size_t rows() { return T2::rows(); }                                      \
  RAIMATH_MATEXPR_OPERATORS                                                                         \
};                                                                                                  \

#define RaiSim_define_binary_function(Operator,OpFunc)                                              \
template<typename T1, typename T2>                                                                  \
inline OpFunc##_op<T1, T2> operator Operator (MatExpr<T1> const &u, MatExpr<T2> const &v)  {        \
  return OpFunc##_op<T1, T2>(*static_cast<const T1 *>(&u), *static_cast<const T2 *>(&v));           \
}                                                                                                   \

RaiSim_define_binary_operator(+,Sum)
RaiSim_define_binary_function(+,Sum)

RaiSim_define_binary_operator(-,Sub)
RaiSim_define_binary_function(-,Sub)

RaiSim_define_binary_operator(/,Div)
RaiSim_define_binary_function(/,Div)

RaiSim_define_binary_operator(*, Prod)
RaiSim_define_binary_function(*, Prod)

template<>
class Prod_op<Mat<4, 4>, Mat<4, 4>> : public MatExpr<Prod_op<Mat<4, 4>, Mat<4, 4>>> {
 public:
  Mat<4, 4> result;
  Prod_op(Mat<4, 4> const &u, Mat<4, 4> const &v) {
    for (size_t k = 0; k < 1; k++) // row
      for (size_t j = 0; j < 1; j++) // col
        result(j, k) = u[k] * v[4*j] + u[k+4] * v[1 + j*4] + u[k+8] * v[2 + 4 * j] + u[k + 12] * v[3 + 4* j];
  }

  inline double operator()(size_t i, size_t j) const { return result(i, j); }
  inline static size_t size() { return 16; }
  inline static size_t cols() { return 4; }
  inline static size_t rows() { return 4; }
};

template<class T>
inline double Div_S_func(const T& exp1, double scalar, size_t i, size_t j) { return exp1(i, j) / scalar; }
template<class T>
inline double Prod_S_func(const T& exp1, double scalar, size_t i, size_t j) { return exp1(i, j) * scalar; }
template<class T>
inline double Sum_S_func(const T& exp1, double scalar, size_t i, size_t j) { return exp1(i, j) + scalar; }
template<class T>
inline double Sub_S_func(const T& exp1, double scalar, size_t i, size_t j) { return exp1(i, j) - scalar; }

//////////////////// scalar operators
#define RaiSim_define_scalar_operator(Operator,OpFunc)                                              \
template<typename T>                                                                                \
class OpFunc##_S_op : public MatExpr<OpFunc##_S_op<T>> {                                            \
 public:                                                                                            \
  using SelfType = MatExpr<OpFunc##_S_op<T>>;                                                       \
  const T &_u;                                                                                      \
  const double _s;                                                                                  \
  inline OpFunc##_S_op(T const &u, double s) : _u(u), _s(s) {}                                      \
  inline double operator()(size_t i, size_t j) const { return OpFunc##_S_func(_u, _s, i, j); }      \
  inline double operator()(size_t i) const { return OpFunc##_S_func(_u, _s, i, 0); }                \
  inline double operator[](size_t i) const { return OpFunc##_S_func(_u, _s, i, 0); }                \
  inline constexpr static size_t size() { return T::size(); }                                                  \
  inline constexpr static size_t cols() { return T::cols(); }                                                  \
  inline constexpr static size_t rows() { return T::rows(); }                                                  \
};                                                                                                  \

#define RaiSim_define_scalar_function(Operator,OpFunc)                                              \
template<typename T>                                                                                \
inline OpFunc##_S_op<T> operator Operator (MatExpr<T> const &u, double scalar)  {                   \
  return OpFunc##_S_op<T>(*static_cast<const T *>(&u), scalar);                                     \
}                                                                                                   \
template<typename T>                                                                                \
inline OpFunc##_S_op<T> operator Operator (double scalar, MatExpr<T> const &u)  {                   \
  return OpFunc##_S_op<T>(*static_cast<const T *>(&u), scalar);                                     \
}                                                                                                   \

RaiSim_define_scalar_operator(+,Sum)
RaiSim_define_scalar_function(+,Sum)

RaiSim_define_scalar_operator(-,Sub)
RaiSim_define_scalar_function(-,Sub)

RaiSim_define_scalar_operator(/,Div)
RaiSim_define_scalar_function(/,Div)

RaiSim_define_scalar_operator(*, Prod)
RaiSim_define_scalar_function(*, Prod)

}

#endif //RAIMATH__OPERATION_HPP_
