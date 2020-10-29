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


namespace raisim {

template<typename E>
class MatExpr {
 public:
  
//  inline double operator[](size_t i) const { return static_cast<E const &>(*this)[i]; }
//  inline double operator()(size_t i, size_t j) const { return static_cast<E const &>(*this)(i, j); }
//  inline double operator()(size_t i) const { return static_cast<E const &>(*this)(i); }
//
//  inline double& operator[](size_t i) { return static_cast<E&>(*this)[i]; }
//  inline double& operator()(size_t i, size_t j) { return static_cast<E&>(*this)(i, j); }
//  inline double& operator()(size_t i) { return static_cast<E&>(*this)(i); }

  inline size_t rows() const { return static_cast<E const &>(*this).rows(); }
  inline size_t size() const { return static_cast<E const &>(*this).size(); }
  inline size_t cols() const { return static_cast<E const &>(*this).cols(); }
};

}
#endif //RAIMATH__EXPRESSION_HPP_
