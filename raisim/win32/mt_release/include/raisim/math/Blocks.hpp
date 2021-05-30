//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_MATH_BLOCKS_H_
#define RAISIM_INCLUDE_RAISIM_MATH_BLOCKS_H_

#include "Expression.hpp"

namespace raisim {

template<class T>
class ElementRef {
 public:
  ElementRef(T& array, size_t id) : ref(array), idx(id) { }
  T& ref;
  size_t idx;
};

template<class T, size_t n>
class DiagonalRef : public MatExpr<DiagonalRef<T, n>> {
 public:
  typedef DiagonalRef<T, n> SelfType;
  explicit DiagonalRef(T& array) :
      ref(array) { }

  T& ref;
};

template<class T>
class SkewRef : public MatExpr<SkewRef<T>> {
 public:
  using SelfType = SkewRef<T>();
  explicit SkewRef(T& vec) :
      ref(vec) {}
  inline double operator()(size_t i) const { return ref(i); }

  T& ref;
};

template<class T, size_t n, size_t m>
class BlockRef : public MatExpr<BlockRef<T, n, m>> {
 public:
  typedef BlockRef<T, n, m> SelfType;
  BlockRef(T& array, size_t r_s, size_t c_s) :
      ref(array), rowStart(r_s), colStart(c_s) { }

  static constexpr inline size_t size() { return n*m; }
  static constexpr inline size_t rows() { return n; }
  static constexpr inline size_t cols() { return m; }

  inline double operator()(size_t i) const { return ref(i+rowStart); }
  inline double &operator()(size_t i) { return ref(i+rowStart); }

  inline double operator[](size_t i) const { return ref(i+rowStart); }
  inline double &operator[](size_t i) { return ref(i+rowStart); }

  inline double operator()(size_t i, size_t j) const { return ref(i+rowStart, j+colStart); }
  inline double &operator()(size_t i, size_t j) { return ref(i+rowStart, j+colStart); }
  RAIMATH_MATEXPR_OPERATORS
  T& ref;
  size_t rowStart, colStart;
};


template<class T, size_t m>
class RowRef : public MatExpr<RowRef<T, m>> {
 public:
  typedef RowRef<T, m> SelfType;

  RowRef(T& array, size_t r) :
      ref(array), row(r) { }

  inline double operator()(size_t i) const { return ref(row, i); }
  inline double &operator()(size_t i) { return ref(row, i); }
  inline double operator[](size_t i) const { return ref(row, i); }
  inline double &operator[](size_t i) { return ref(row, i); }
  inline double operator()(size_t i, size_t j) const { return ref(row, j); }
  inline double &operator()(size_t i, size_t j) { return ref(row, j); }

  static constexpr inline size_t size() { return m; }
  static constexpr inline size_t rows() { return size_t(1); }
  static constexpr inline size_t cols() { return m; }

  template<class T2>
  inline RowRef<T, m>& operator = (const MatExpr<T2>& expr) {
    for (size_t j = 0; j < ref.rows(); ++j) ref(row, j) = expr(j);
    return *this;
  }

  RAIMATH_MATEXPR_OPERATORS
  T& ref;
  size_t row;
};

template<class T, size_t n>
class ColRef : public MatExpr<ColRef<T, n>> {
 public:
  typedef ColRef<T, n> SelfType;
  ColRef(T& array, size_t c) :
      ref(array), col(c) { }

  inline double operator()(size_t i) const { return ref(i, col); }
  inline double &operator()(size_t i) { return ref(i, col); }
  inline double operator[](size_t i) const { return ref(i, col); }
  inline double &operator[](size_t i) { return ref(i, col); }

  inline double operator()(size_t i, size_t j) const { return ref(i, col); }
  inline double &operator()(size_t i, size_t j) { return ref(i, col); }

  static constexpr inline size_t size() { return n; }
  static constexpr inline size_t rows() { return n; }
  static constexpr inline size_t cols() { return 1; }

  RAIMATH_MATEXPR_OPERATORS
  T& ref;
  size_t col;
};

}

#endif //RAISIM_INCLUDE_RAISIM_MATH_BLOCKS_H_
