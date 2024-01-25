//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAIMATH__MATRIX_HPP_
#define RAIMATH__MATRIX_HPP_

#include "Expression.hpp"
#include "Blocks.hpp"

namespace raisim {

template<class T>
class Transpose : public MatExpr<Transpose<T>> {
 public:
  typedef Transpose<T> SelfType;
  explicit Transpose(T const &u) : _u(u) {}
  const T &_u;
  inline double operator()(size_t i, size_t j) const { return _u(j,i); }
  inline double operator[] (size_t i) const { return _u(i/T::n, i%T::n); }
  static inline constexpr size_t size() { return T::size(); }
  static inline constexpr size_t cols() { return T::rows(); }
  static inline constexpr size_t rows() { return T::cols(); }
};

using ScalarType = double;

template<size_t n, size_t m>
class Mat : public MatExpr<Mat<n, m>> {
 public:
  typedef Eigen::Map<Eigen::Matrix<ScalarType, n, m> > EigenMat;
  typedef Eigen::Map<const Eigen::Matrix<ScalarType, n, m> > ConstEigenMat;
  typedef Mat<n,m> SelfType;

  union {
    std::aligned_storage<n*m*sizeof(ScalarType), 32> alignment_only;
    double v[n * m];
  };

  Mat() = default;
  static constexpr inline size_t size() { return n * m; }
  static constexpr inline size_t rows() { return n; }
  static constexpr inline size_t cols() { return m; }

  /// Constructors
  Mat(std::initializer_list<ScalarType> init) { ScalarType* ele = &v[0]; for (auto it=init.begin(); it!=init.end(); ++it) *ele++ = *it; }

  /// copy eigen matrix
  template<class scalarType>
  Mat(const Eigen::Matrix<scalarType, n, m>& eigen) { for (size_t j = 0; j < cols(); ++j) for (size_t i = 0; i < rows(); ++i) v[i + j * n] = eigen(i,j); }

  template<class T2>
  Mat(const T2 &rhs) { for (size_t i = 0; i < rows(); ++i) for (size_t j = 0; j < cols(); ++j) operator()(i,j) = rhs(i,j); }

  template<class T2>
  inline Mat<n,m>& operator = (const MatExpr<T2>& expr) {
    for (size_t j = 0; j < cols(); ++j) for (size_t i = 0; i < rows(); ++i) v[i + j * n] = expr(i,j);
    return *this;
  }

  template<typename T2, size_t n2, size_t m2>
  inline Mat<n,m>& operator = (const Mat<n2, m2>& expr) { for (size_t j = 0; j < size(); ++j) v[j] = expr[j];
    return *this;
  }

  inline ScalarType *ptr() { return &(v[0]); }
  inline const ScalarType *ptr() const { return &(v[0]); }

  /// bracket methods
  inline double operator[](size_t i) const { return v[i]; }
  inline double &operator[](size_t i) { return v[i]; }

  inline double operator()(size_t i) const { return v[i]; }
  inline double &operator()(size_t i) { return v[i]; }

  inline double operator()(size_t i, size_t j) const { return v[i + j * n]; }
  inline double &operator()(size_t i, size_t j) { return v[i + j * n]; }

  inline Mat<n,m> operator-() {
    Mat<n,m> result = *this;
    result *=-1;
    return result;
  }

  RAIMATH_MATEXPR_OPERATORS

  EigenMat e() { return EigenMat(v);}
  ConstEigenMat e() const{ return ConstEigenMat(v); }

  inline void operator-=(const Mat<n,m> val) {
    for(size_t i=0; i< size(); i++)
      v[i] -= val[i];
  }

  inline void operator+=(const Mat<n,m> val) {
    for(size_t i=0; i< size(); i++)
      v[i] += val[i];
  }

  inline void operator/=(const Mat<n,m> val) {
    for(size_t i=0; i< size(); i++)
      v[i] /= val[i];
  }

  inline void setIdentity() {
    static_assert(n==m, "setIdentity only works with a square matrix");
    setZero();
    for (size_t i = 0; i < n; i++)
      v[i * n + i] = 1.0;
  }

  static inline Mat<n,n> getIdentity() {
    static_assert(n==m, "getIdentity only works for square matrices");
    Mat<n,n> identity; identity.setIdentity();
    return identity;
  }

  static inline Mat<n,m> getZeros() {
    Mat<n,m> zero; zero.setZero();
    return zero;
  }

  inline Transpose<Mat<n,m>> transpose() { return Transpose<Mat<n,m>>(*this); }
  inline const Transpose<Mat<n,m>> transpose() const { return Transpose<Mat<n,m>>(*this); }

  inline RowRef<Mat<n, m>, m> row(size_t r) { return RowRef<Mat<n,m>, m>(*this, r); }
  inline const RowRef<Mat<n, m>, m> row(size_t r) const { return RowRef<Mat<n,m>, m>(*this, r); }

  inline ColRef<Mat<n, m>, n> col(size_t r) { return ColRef<Mat<n,m>, n>(*this, r); }
  inline const ColRef<Mat<n, m>, n> col(size_t r) const { return ColRef<Mat<n,m>, n>(*this, r); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> segment(size_t startRow, size_t startCol) {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, startRow, startCol); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> segment(size_t startRow, size_t startCol) const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, startRow, startCol); }

  template<size_t sizeRow>
  inline BlockRef<Mat<n, m>, sizeRow, 1> head() {
    static_assert(m==1, "head method is only for vectors");
    return BlockRef<Mat<n, m>, sizeRow, 1>(*this, 0, 0); }

  template<size_t sizeRow>
  inline BlockRef<Mat<n, m>, sizeRow, 1> head() const {
    static_assert(m==1, "head method is only for vectors");
    return BlockRef<Mat<n, m>, sizeRow, 1>(*this, 0, 0); }

  template<size_t sizeRow>
  inline BlockRef<Mat<n, m>, sizeRow, 1> tail() {
    static_assert(m==1, "tail method is only for vectors");
    return BlockRef<Mat<n, m>, sizeRow, 1>(*this, n-sizeRow, 0); }

  template<size_t sizeRow>
  inline BlockRef<Mat<n, m>, sizeRow, 1> tail() const {
    static_assert(m==1, "tail method is only for vectors");
    return BlockRef<Mat<n, m>, sizeRow, 1>(*this, n-sizeRow, 0); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> topLeftCorner() const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, 0, 0); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> topLeftCorner() {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, 0, 0); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> topRightCorner() const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(0), m-sizeCol); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> topRightCorner() {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(0), m-sizeCol); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> bottomRightCorner() const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(n-sizeRow), size_t(m-sizeCol)); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> bottomRightCorner() {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(n-sizeRow), size_t(m-sizeCol)); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> bottomLeftCorner() const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(n-sizeRow), size_t(0)); }

  template<size_t sizeRow, size_t sizeCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> bottomLeftCorner() {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(n-sizeRow), size_t(0)); }

  template<size_t sizeRow, size_t sizeCol, size_t startRow, size_t startCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> block() const {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(startRow), size_t(startCol)); }

  template<size_t sizeRow, size_t sizeCol, size_t startRow, size_t startCol>
  inline BlockRef<Mat<n, m>, sizeRow, sizeCol> block() {
    return BlockRef<Mat<n, m>, sizeRow, sizeCol>(*this, size_t(startRow), size_t(startCol)); }
};

}

#endif //RAIMATH__MATRIX_HPP_
