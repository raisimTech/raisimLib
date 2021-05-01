//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAIMATH__EXPRESSION_HPP_
#define RAIMATH__EXPRESSION_HPP_

#define RAIMATH_MATEXPR_OPERATORS                                 \
inline double norm() const {                                      \
  double sum=0.0;                                                 \
  for (size_t i = 0; i < size(); i++)                             \
    sum += operator()(i) * operator()(i);                         \
  return std::sqrt(sum);                                          \
}                                                                 \
                                                                  \
inline double sum() const {                                       \
  double sum = 0;                                                 \
  for(size_t i=0; i< size(); i++)                                 \
    sum += operator()(i);                                         \
  return sum;                                                     \
}                                                                 \
                                                                  \
inline double squaredNorm() const {                               \
  double sum=0.0;                                                 \
  for (size_t i = 0; i < size(); i++)                             \
    sum += operator()(i)*operator()(i);                           \
  return sum;                                                     \
}                                                                 \
                                                                  \
template<class OtherType> double dot(const OtherType& other) const { \
  double sum = 0;                                                 \
  for(size_t i=0; i<size(); i++)                                  \
    sum += operator()(i) * other(i);                              \
  return sum;                                                     \
}                                                                 \
                                                                  \
void setZero() {                                                  \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) = 0;                                            \
}                                                                 \
                                                                  \
void setConstant(double constant) {                               \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) = constant;                                     \
}                                                                 \
                                                                  \
inline void operator*=(const double val) {                        \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) *= val;                                         \
}                                                                 \
                                                                  \
inline void operator/=(const double val) {                        \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) /= val;                                         \
}                                                                 \
                                                                  \
inline void operator+=(const double val) {                        \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) += val;                                         \
}                                                                 \
                                                                  \
inline void operator-=(const double val) {                        \
  for(size_t i=0; i< size(); i++)                                 \
    operator()(i) -= val;                                         \
}                                                                 \
                                                                  \
template<class OtherType>                                         \
CrossProduct<SelfType, OtherType> cross(const OtherType& other) const { \
  return CrossProduct<SelfType, OtherType>(*this, other);         \
}

namespace raisim {

template<typename E>
class MatExpr {
 public:
  using SelfType = MatExpr<E>;
//  inline double operator[](size_t i) const { return static_cast<E const &>(*this)[i]; }
//  inline double operator()(size_t i, size_t j) const { return static_cast<E const &>(*this)(i, j); }
//  inline double operator()(size_t i) const { return static_cast<E const &>(*this)(i); }
//
//  inline double& operator[](size_t i) { return static_cast<E&>(*this)[i]; }
//  inline double& operator()(size_t i, size_t j) { return static_cast<E&>(*this)(i, j); }
//  inline double& operator()(size_t i) { return static_cast<E&>(*this)(i); }

  inline constexpr size_t rows() const { return static_cast<E const &>(*this).rows(); }
  inline constexpr size_t size() const { return static_cast<E const &>(*this).size(); }
  inline constexpr size_t cols() const { return static_cast<E const &>(*this).cols(); }
};

template<class T1, class T2>
class CrossProduct : public MatExpr<CrossProduct<T1, T2>> {
 public:
  explicit CrossProduct(T1 const &u, T2 const &v) : _u(u), _v(v) {
    static_assert(T1::cols()==1 && T1::rows()==3, "the vector should be 3X1");
    static_assert(T2::cols()==1 && T2::rows()==3, "the vector should be 3X1");
  }
  const T1 &_u;
  const T2 &_v;
  inline double operator()(size_t i, size_t j) const { return _u((i+1)%3)*_v((i+2)%3) - _u((i+2)%3)*_v((i+1)%3); }
  inline double operator()(size_t i) const { return _u((i+1)%3)*_v((i+2)%3) - _u((i+2)%3)*_v((i+1)%3); }
  static inline constexpr size_t size() { return 3; }
  static inline constexpr size_t cols() { return 1; }
  static inline constexpr size_t rows() { return 3; }
};

}
#endif //RAIMATH__EXPRESSION_HPP_
