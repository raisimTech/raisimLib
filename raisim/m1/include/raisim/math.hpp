//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_MATH_HPP
#define RAISIM_MATH_HPP
#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <numeric>
#include "math.h"
#include "math/Core.hpp"
#include <iostream>
#include <stdlib.h>
#include "float.h"
#include "raisim/raisim_message.hpp"
#include <cstdlib>
#include <cstdio>

#ifdef __APPLE__
inline void sincosf(float x, float *s, float *c) { __sincosf(x, s, c); }
inline void sincos(double x, double *s, double *c) { __sincos(x, s, c); }

// implementation from https://embeddedartistry.com/blog/2017/2/20/implementing-aligned-malloc

//Number of bytes we're using for storing the aligned pointer offset
typedef uint16_t offset_t;
#define PTR_OFFSET_SZ sizeof(offset_t)

//Number of bytes we're using for storing the aligned pointer offset
typedef uint16_t offset_t;
#define PTR_OFFSET_SZ sizeof(offset_t)

#ifndef align_up
#define align_up(num, align) \
    (((num) + ((align) - 1)) & ~((align) - 1))
#endif

inline void * aligned_alloc(size_t align, size_t size) {
    void * ptr = NULL;

    //We want it to be a power of two since align_up operates on powers of two
    assert((align & (align - 1)) == 0);

    if(align && size)
    {
        /*
         * We know we have to fit an offset value
         * We also allocate extra bytes to ensure we can meet the alignment
         */
        uint32_t hdr_size = PTR_OFFSET_SZ + (align - 1);
        void * p = malloc(size + hdr_size);

        if(p)
        {
            /*
             * Add the offset size to malloc's pointer (we will always store that)
             * Then align the resulting value to the arget alignment
             */
            ptr = (void *) align_up(((uintptr_t)p + PTR_OFFSET_SZ), align);

            //Calculate the offset and store it behind our aligned pointer
            *((offset_t *)ptr - 1) = (offset_t)((uintptr_t)ptr - (uintptr_t)p);

        } // else NULL, could not malloc
    } //else NULL, invalid arguments

    return ptr;
}

inline void aligned_free(void * ptr) {
    assert(ptr);

    /*
    * Walk backwards from the passed-in pointer to get the pointer offset
    * We convert to an offset_t pointer and rely on pointer math to get the data
    */
    offset_t offset = *((offset_t *)ptr - 1);

    /*
    * Once we have the offset, we can get our original pointer and call free
    */
    void * p = (void *)((uint8_t *)ptr - offset);
    free(p);
}
#endif

namespace raisim {

template <size_t Size>
using Vec = Mat<Size, 1>;

class VecDyn;
class MatDyn;

class DynamicArray{
 public:

  friend class raisim::VecDyn;
  friend class raisim::MatDyn;

  DynamicArray() = default;

  ~DynamicArray() { dealloc(); }

  double *v = nullptr;

  double* ptr() {return v;}
  const double* ptr() const {return v;}

  double* data() {return v;}
  const double* data() const {return v;}

  inline double operator [](size_t i) const    {return v[i];}
  inline double & operator [](size_t i) {return v[i];}

  /// comma initializer
  inline ElementRef<DynamicArray> operator = (double a) {
    ElementRef<DynamicArray> ref(*this, 1);
    v[0] = a;
    return ref;
  }

 private:
  void allocate(size_t size) {
    dealloc();
#ifdef _WIN32
    v = static_cast<double *>(
        Eigen::internal::handmade_aligned_malloc(size * sizeof(double)));
#elif __linux__
    v = static_cast<double*>(aligned_alloc(32, size * sizeof(double)));
#elif __APPLE__
    v = static_cast<double*>(aligned_alloc(32, size * sizeof(double)));
#endif
  }

  void dealloc() {
#ifdef __APPLE__
    if (v) aligned_free(v);
#elif __linux__
    free(v);
#elif WIN32
    if (v) Eigen::internal::handmade_aligned_free(v);
#endif
  }
};



template<int n>
inline ElementRef<DynamicArray> operator , (ElementRef<DynamicArray> ref, double a) {
  ref.ref[ref.idx] = a;
  ref.idx++;
  return ref;
}

template<int n>
inline std::ostream &operator<<(std::ostream &os, const Vec<n> &m) {
  for(size_t i=0; i<n ; i++)
    os << m[i] << " ";

  return os;
}

template<int n>
inline ElementRef<Vec<n>> operator , (ElementRef<Vec<n>> ref, double a) {
  ref.ref[ref.idx] = a;
  ref.idx++;
  return ref;
}

template<int n, int m>
inline ElementRef<Mat<n,m>> operator , (ElementRef<Mat<n,m>> ref, double a) {
  ref.ref[ref.idx] = a;
  ref.idx++;
  return ref;
}

template<size_t n, size_t m>
inline std::ostream &operator<<(std::ostream &os, const Mat<n,m> &mat) {
  for(size_t i=0; i < n ; i++) {
    for (size_t j = 0; j < m; j++)
      os << mat[j*n+i] << " ";
    os << "\n";
  }
  return os;
}

class VecDyn: public DynamicArray {
 public:

  size_t n;
  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVecn;
  typedef Eigen::Map<const Eigen::Matrix<double, -1, 1> > ConstEigenVecn;

  VecDyn() = default;

  VecDyn(size_t size){
    resize(size);
  }

  inline double operator ()(size_t i, size_t j) const { return v[i+j*0]; }
  inline double & operator ()(size_t i, size_t j) {return v[i+j*0];}

  size_t size() const {
    return n;
  }

  size_t rows() const {
    return n;
  }

  size_t cols() const {
    return 1;
  }

  VecDyn(const VecDyn& vec){
    resize(vec.n);
    memcpy(ptr(), vec.ptr(), n* sizeof(double));
  }

  inline double sum() const {
    double sum = 0;
    for(size_t i=0; i< n; i++)
      sum += v[i];
    return sum;
  }

  void resize(size_t size) {
    n = size;
    allocate(size);
  }

  inline double squaredNorm() const {
    double sum=0.0;
    for (size_t i = 0; i < n; i++)
      sum += v[i]*v[i];
    return sum;
  }

  inline double norm() const {
    double sum=0.0;
    for (size_t i = 0; i < n; i++)
      sum += v[i]*v[i];
    return sqrt(sum);
  }

  EigenVecn e() {
    return EigenVecn(v, n, 1);
  }

  const ConstEigenVecn e() const {
    return ConstEigenVecn(v, n, 1);
  }

  inline void operator/=(double val) {
    for (size_t i = 0; i < n; i++)
      v[i] /= val;
  }

  inline void operator*=(double val) {
    for (size_t i = 0; i < n; i++)
      v[i] *= val;
  }

  inline VecDyn& operator=(const VecDyn &rhs) {
    for(size_t i=0; i<n; ++i)
      v[i] = rhs[i];
    return *this;
  }

  inline VecDyn& operator-=(const VecDyn &rhs) {
    for (size_t i = 0; i < n; i++)
      v[i] -= rhs[i];
    return *this;
  }

  inline VecDyn& operator+=(const VecDyn &rhs) {
    for (size_t i = 0; i < n; i++)
      v[i] += rhs[i];
    return *this;
  }

  template<int rows, int cols>
  inline VecDyn& operator=(const Eigen::Matrix<double, rows, cols> &rhs) {
    for(size_t i=0; i<n; ++i)
      v[i] = rhs[i];
    return *this;
  }

  inline VecDyn& operator=(const Eigen::Matrix<double, -1, 1> &rhs) {
    DRSFATAL_IF(rhs.rows()!=n, "dimension mismatch");
    for(size_t i=0; i<n; ++i)
      v[i] = rhs[i];
    return *this;
  }

  template<int size>
  inline VecDyn& operator=(const Vec<size> &rhs) {
    for(size_t i=0; i<n; ++i)
      v[i] = rhs[i];
    return *this;
  }

  inline void fillSegment(const VecDyn &rhs, size_t startingIdx) {
    memcpy(ptr() + startingIdx, rhs.ptr(), rhs.n * sizeof(double));
  }
  
  template<size_t size>
  inline void fillSegment(const Vec<size> &rhs, size_t startingIdx) {
    memcpy(ptr() + startingIdx, rhs.ptr(), size * sizeof(double));
  }
  
  inline void setZero() {
    for (size_t i = 0; i < n; i++) v[i] = 0;
  }

  inline void setZero(size_t cols) {
    resize(cols);
    for (size_t i = 0; i < cols; i++) v[i] = 0;
  }

  inline void setConstant(double c) {
    for (size_t i = 0; i < n; i++)
      v[i] = c;
  }
};

inline std::ostream &operator<<(std::ostream &os, const VecDyn &m) {
  for(size_t i=0; i<m.n ; i++)
    os << m[i] << " ";

  return os;
}

class MatDyn: public DynamicArray {
 public:

  size_t n, m;

  MatDyn(){}

  MatDyn(size_t rows, size_t cols){
    resize(rows, cols);
  }

  MatDyn(const MatDyn& mat){
    resize(mat.n, mat.m);
    memcpy(ptr(), mat.ptr(), n*m* sizeof(double));
  }

  size_t size() const {
    return n * m;
  }

  size_t rows() const {
    return n;
  }

  size_t cols() const {
    return m;
  }

  Eigen::Map<Eigen::Matrix<double, -1, -1> > e() {
    return Eigen::Map<Eigen::Matrix<double, -1, -1> >(v, n, m);
  }

  const Eigen::Map<const Eigen::Matrix<double, -1, -1> > e() const {
    return Eigen::Map<const Eigen::Matrix<double, -1, -1> >(v, n, m);
  }

  inline double operator ()(size_t i, size_t j) const { return v[i+j*n]; }
  inline double & operator ()(size_t i, size_t j) { return v[i+j*n]; }

  void resize(size_t rows, size_t cols) {
    n = rows;
    m = cols;
    allocate(rows * cols);
  }

  template<size_t j, size_t k>
  void fillSub(size_t startRow, size_t startColumn, const Mat<j, k> &in) {
    for (size_t i = 0; i < j; i++) /// column index
      for (size_t o = 0; o < k; o++) /// row index
        v[n * (startColumn + i) + (o + startRow)] = in[i * j + o];
  }

  void fillSubSkewSym(size_t startRow, size_t startColumn, const Vec<3> &in) {
    v[n * (startColumn) + (1 + startRow)] = in[2];
    v[n * (startColumn) + (2 + startRow)] = -in[1];
    v[n * (startColumn + 1) + (startRow)] = -in[2];
    v[n * (startColumn + 1) + (2 + startRow)] = in[0];
    v[n * (startColumn + 2) + (0 + startRow)] = in[1];
    v[n * (startColumn + 2) + (1 + startRow)] = -in[0];
  }

  void fillSubSkewSymTransposed(size_t startRow, size_t startColumn, Vec<3> &in) {
    v[n * (startColumn) + (1 + startRow)] = -in[2];
    v[n * (startColumn) + (2 + startRow)] = in[1];
    v[n * (startColumn + 1) + (startRow)] = in[2];
    v[n * (startColumn + 1) + (2 + startRow)] = -in[0];
    v[n * (startColumn + 2) + (0 + startRow)] = -in[1];
    v[n * (startColumn + 2) + (1 + startRow)] = in[0];
  }


  template<size_t j, size_t k>
  void fillSubTransposed(size_t startRow, size_t startColumn, const Mat<j, k> &in) {
    for (size_t i = 0; i < j; i++) /// column index
      for (size_t o = 0; o < k; o++) /// row index
        v[n * (startColumn + i) + (o + startRow)] = in[i + o * j];
  }

  template<size_t j>
  void fillSub(size_t startRow, size_t startColumn, const Vec<j> &in) {
    for (size_t o = 0; o < j; o++) /// row index
      operator()(startRow+o, startColumn) = in[o];
  }

  template<size_t j>
  void fillSubTransposed(size_t startRow, size_t startColumn, const Vec<j> &in) {
    for (size_t o = 0; o < j; o++) /// row index
      operator()(startRow, startColumn+o) = in[o];
  }

  void setIdentity() {
    memset(ptr(), 0, n * n * sizeof(double));
    for (size_t i = 0; i < n; i++)
      v[i * n + i] = 1.0;
  }

  inline void setZero() {
    for (size_t i = 0; i < n * m; i++) v[i] = 0;
  }

  inline double sum() const {
    double summ = 0;
    for (size_t j = 0; j < m; j++)
      for (size_t i = 0; i < n; i++)
        summ += v[j * n + i];
    return summ;
  }

  inline void setZero(size_t rows, size_t cols) {
    resize(rows,cols);
    for (size_t i = 0; i < n * m; i++) v[i] = 0;
  }

  template<size_t rows, size_t cols>
  inline MatDyn& operator=(const Mat<rows,cols> &rhs) {
    for(size_t i=0; i<n*m; ++i)
      v[i] = rhs[i];
    return *this;
  }

  template<size_t rows, size_t cols>
  inline MatDyn& operator=(const Eigen::Matrix<double, rows, cols> &rhs) {
    for(size_t i=0; i<n*m; ++i)
      v[i] = rhs[i];
    return *this;
  }

  inline MatDyn& operator=(const MatDyn &rhs) {
    for(size_t i=0; i<n*m; ++i)
      v[i] = rhs[i];
    return *this;
  }

  inline void operator*=(double val) {
    for (size_t j = 0; j < m; j++)
      for (size_t i = 0; i < n; i++)
        v[i+n*j] *= val;
  }

};

inline std::ostream &operator<<(std::ostream &os, const MatDyn &mat) {
  for(size_t i=0; i < mat.n ; i++) {
    for (size_t j = 0; j < mat.m; j++)
      os << mat[j*mat.n+i] << " ";
    os << "\n";
  }
  return os;
}

class SparseJacobian {
 public:
  SparseJacobian() {}
  ~SparseJacobian() {}

  inline void resize(size_t cols) {
    size = cols;
    if(capacity < cols){
      v.resize(3, cols);
      v.setZero();
      idx.resize(cols);
      capacity = cols;
    }
  }

  double operator [](size_t i) const    {return v[i];}
  double & operator [](size_t i) {return v[i];}

  inline double operator ()(size_t i, size_t j) const { return v[i+j*3]; }
  inline double & operator ()(size_t i, size_t j) { return v[i+j*3]; }

  inline void operator*=(double val) {
    v *= val;
  }

  double* ptr() {return &(v.v[0]);}
  const double* ptr() const {return &(v.v[0]);}

  double* data() {return &(v.v[0]);}
  const double* data() const {return &(v.v[0]);}

  Eigen::Map<Eigen::Matrix<double, -1, -1> > e() {
    return Eigen::Map<Eigen::Matrix<double, -1, -1> >(v.ptr(), 3, size);
  }

  const Eigen::Map<const Eigen::Matrix<double, -1, -1> > e() const {
    return Eigen::Map<const Eigen::Matrix<double, -1, -1> >(v.ptr(), 3, size);
  }

  template<size_t j, size_t k>
  void fillSub(size_t startRow, size_t startColumn, const Mat<j, k> &in) {
    v.fillSub(startRow, startColumn, in);
  }

  inline SparseJacobian& operator=(const SparseJacobian &rhs) {
    size = rhs.size;
    DRSFATAL_IF(size<1, "assigning zero volume")
    v.resize(3, size);
    memcpy(v.ptr(), rhs.v.ptr(), sizeof(double) * 3 * size);
    capacity = rhs.size;
    idx.resize(size);
    for(size_t i=0; i<size; i++)
      idx[i] = rhs.idx[i];
    return *this;
  }

  size_t size, capacity=0;
  MatDyn v;
  std::vector<size_t> idx;
};

class SparseJacobian1D {
 public:
  SparseJacobian1D() = default;
  ~SparseJacobian1D() = default;

  void resize(size_t cols) {
    size = cols;
    if(capacity < cols){
      v.resize(1, cols);
      idx.resize(cols);
      capacity = cols;
    }
    memset(v.ptr(), 0, cols * sizeof(double));
  }

  double operator [](size_t i) const    {return v[i];}
  double & operator [](size_t i) {return v[i];}

  inline void operator*=(double val) {
    v *= val;
  }

  double* ptr() {return &(v.v[0]);}
  const double* ptr() const {return &(v.v[0]);}
  double* data() {return &(v.v[0]);}
  const double* data() const {return &(v.v[0]);}

  Eigen::Map<Eigen::Matrix<double, -1, -1> > e() {
    return Eigen::Map<Eigen::Matrix<double, -1, -1> >(v.ptr(), 1, size);
  }

  const Eigen::Map<const Eigen::Matrix<double, -1, -1> > e() const {
    return Eigen::Map<const Eigen::Matrix<double, -1, -1> >(v.ptr(), 1, size);
  }

  inline SparseJacobian1D& operator=(const SparseJacobian1D &rhs) {
    size = rhs.size;
    v.resize(1, rhs.capacity);
    v = rhs.v;
    capacity = rhs.capacity;
    idx = rhs.idx;
    return *this;
  }

  size_t size, capacity=0;
  MatDyn v;
  std::vector<size_t> idx;
};

struct Transformation {
  Transformation () {
    rot.setIdentity();
    pos.setZero();
  }

  Mat<3,3> rot;
  Vec<3> pos;
};


template<size_t n>
static inline void cholInvFast(const double * A, double * AInv) {
  size_t i, j, k;
  double sum;
  double p[n], Mtemp_[n*n];
  memcpy(AInv, A, n * n * sizeof(double));

  p[0] = 1. / std::sqrt(AInv[0]);

  for (j = 1; j < n; j++)
    AInv[j] = AInv[n * j] * p[0];

  for (i = 1; i < n; i++) {
    sum = AInv[i * n + i];
    for (k = i - 1; k >= 1; k--)
      sum -= AInv[i + n * k] * AInv[i + n * k];
    sum -= AInv[i] * AInv[i];
    p[i] = 1. / std::sqrt(sum);
    for (j = i + 1; j < n; j++) {
      sum = AInv[i + n * j];
      for (k = i - 1; k >= 1; k--)
        sum -= AInv[i + n * k] * AInv[j + n * k];
      sum -= AInv[i] * AInv[j];
      AInv[j + n * i] = sum * p[i];
    }
  }

//  for (i = 0; i < n; i++) {
//    AInv[i + n * i] = 1./p[i];
//  }

//  memcpy(Mtemp_, AInv, n * n * sizeof(double));

  /// Matrix inversion using forward substitution
  AInv[0] = 0.;
  for (j = size_t(0); j < n; ++j) {
    size_t jdof = j * n;
    size_t jjdof = j * n + j;

    ///diagonal terms
    AInv[jjdof] = p[j] - AInv[jdof] * AInv[jdof];
    for (k = 1; k < j; k++)
      AInv[jjdof] -= AInv[k + jdof] * AInv[k + jdof];

    AInv[jjdof] *= p[j];
    ///off - diagonal terms
    for (i = j + size_t(1); i < n; ++i) {
      size_t idof = i * n;
      AInv[i + jdof] = -AInv[idof] * AInv[jdof];
      for (k = size_t(1); k < i; ++k)
        AInv[i + jdof] -= AInv[k + idof] * AInv[k + jdof];

      AInv[i + jdof] *= p[i];
      AInv[j + idof] = AInv[i + jdof];
    }
  }
}


template<size_t n>
static inline void cholInv(const double * A, double * AInv) {
  size_t i, j, k;
  double sum;
  double p[n];
  memcpy(AInv, A, n * n * sizeof(double));

  p[0] = 1. / std::sqrt(AInv[0]);

  for (j = 1; j < n; j++)
    AInv[j] = AInv[n * j] * p[0];

  for (i = 1; i < n; i++) {
    sum = AInv[i * n + i];
    for (k = i - 1; k >= 1; k--)
      sum -= AInv[i + n * k] * AInv[i + n * k];
    sum -= AInv[i] * AInv[i];
    p[i] = 1. / std::sqrt(sum);
    for (j = i + 1; j < n; j++) {
      sum = AInv[i + n * j];
      for (k = i - 1; k >= 1; k--)
        sum -= AInv[i + n * k] * AInv[j + n * k];
      sum -= AInv[i] * AInv[j];
      AInv[j + n * i] = sum * p[i];
    }
  }

  for (i = 0; i < n; i++) {
    AInv[i + n * i] = p[i];
    for (j = i + 1; j < n; j++) {
      sum = 0.0;
      for (k = i; k < j; k++) {
        sum -= AInv[j + n * k] * AInv[k + n * i];
      }
      AInv[j + n * i] = sum * p[j];
    }
  }

  for (i = 0; i < n; i++) {
    AInv[i + n * i] *= AInv[i + n * i];
    for (k = i + 1; k < n; k++)
      AInv[i + n * i] += AInv[k + n * i] * AInv[k + n * i];

    for (j = i + 1; j < n; j++) {
      AInv[i + n * j] = AInv[j + n * i] * AInv[j + n * j];
      for (k = j + 1; k < n; k++)
        AInv[i + n * j] += AInv[k + n * i] * AInv[k + n * j];
    }
  }

  for (i = 1; i < n; i++)
    for (j = 0; j < i; j++)
      AInv[i + n * j] = AInv[j + n * i];
}


template<size_t n>
static inline void cholInv(const raisim::Vec<(n*n+n)/2>& A,  raisim::Vec<(n*n+n)/2>& result) {
  size_t i, j, k;
  double sum;
  Vec<n> p;
  Mat<n,n> AInv;

  k=0;
  for(i=0; i<n; i++) {
    AInv[i + n * i] = A[k++];
    for(j=i+1; j<n; j++) {
      AInv[i + n * j] = A[k];
      AInv[j + n * i] = A[k++];
    }
  }

  p[0] = 1. / std::sqrt(AInv[0]);

  for (j = 1; j < n; j++)
    AInv[j] *= p[0];

  for (i = 1; i < n; i++) {
    sum = AInv[i * n + i];
    for (k = i - 1; k >= 1; k--)
      sum -= AInv[i + n * k] * AInv[i + n * k]; // lower
    sum -= AInv[i] * AInv[i];
    p[i] = 1. / std::sqrt(sum);
    for (j = i + 1; j < n; j++) {
      sum = AInv[i + n * j]; // upper
      for (k = i - 1; k >= 1; k--)
        sum -= AInv[i + n * k] * AInv[j + n * k]; //lower
      sum -= AInv[i] * AInv[j]; // lower
      AInv[j + n * i] = sum * p[i];
    }
  }

  for (i = 0; i < n; i++) {
    AInv[i + n * i] = p[i];
    for (j = i + 1; j < n; j++) {
      sum = 0.0;
      for (k = i; k < j; k++) {
        sum -= AInv[j + n * k] * AInv[k + n * i];
      }
      AInv[j + n * i] = sum * p[j];
    }
  }

  size_t c=0;
  for (i = 0; i < n; i++) {
    result[c] = AInv[i + n * i] * AInv[i + n * i];
    for (k = i + 1; k < n; k++)
      result[c] += AInv[k + n * i] * AInv[k + n * i];

    ++c;
    for (j = i + 1; j < n; j++) {
      result[c] = AInv[j + n * i] * AInv[j + n * j];
      for (k = j + 1; k < n; k++)
        result[c] += AInv[k + n * i] * AInv[k + n * j];
      ++c;
    }
  }
}


inline void quatToRotMat(const Vec<4> &q, Mat<3, 3> &R) {
  R[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R[1] = 2 * q[0] * q[3] + 2 * q[1] * q[2];
  R[2] = 2 * q[1] * q[3] - 2 * q[0] * q[2];

  R[3] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R[5] = 2 * q[0] * q[1] + 2 * q[2] * q[3];

  R[6] = 2 * q[0] * q[2] + 2 * q[1] * q[3];
  R[7] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
  R[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

inline void quatToRotMat(const double* q, Mat<3, 3> &R) {
  R[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R[1] = 2 * q[0] * q[3] + 2 * q[1] * q[2];
  R[2] = 2 * q[1] * q[3] - 2 * q[0] * q[2];

  R[3] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R[5] = 2 * q[0] * q[1] + 2 * q[2] * q[3];

  R[6] = 2 * q[0] * q[2] + 2 * q[1] * q[3];
  R[7] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
  R[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

inline void rotMatToQuat(const Mat<3, 3> &R, Vec<4> &q) {
  double tr = R[0] + R[4] + R[8];
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    q[0] = 0.25 * S;
    q[1] = (R[5] - R[7]) / S;
    q[2] = (R[6] - R[2]) / S;
    q[3] = (R[1] - R[3]) / S;
  } else if ((R[0] > R[4]) & (R[0] > R[8])) {
    double S = sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0; // S=4*qx
    q[0] = (R[5] - R[7]) / S;
    q[1] = 0.25 * S;
    q[2] = (R[3] + R[1]) / S;
    q[3] = (R[6] + R[2]) / S;
  } else if (R[4] > R[8]) {
    double S = sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0; // S=4*qy
    q[0] = (R[6] - R[2]) / S;
    q[1] = (R[3] + R[1]) / S;
    q[2] = 0.25 * S;
    q[3] = (R[7] + R[5]) / S;
  } else {
    double S = sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0; // S=4*qz
    q[0] = (R[1] - R[3]) / S;
    q[1] = (R[6] + R[2]) / S;
    q[2] = (R[7] + R[5]) / S;
    q[3] = 0.25 * S;
  }
}

inline void rpyToRotMat_intrinsic(const Vec<3> &rpy, Mat<3, 3> &R) {
  const double su = std::sin(rpy[0]), cu = std::cos(rpy[0]);
  const double sv = std::sin(rpy[1]), cv = std::cos(rpy[1]);
  const double sw = std::sin(rpy[2]), cw = std::cos(rpy[2]);

  R[0] = cv * cw;
  R[1] = cv * sw;
  R[2] = -sv;

  R[3] = su * sv * cw - cu * sw;
  R[4] = cu * cw + su * sv * sw;
  R[5] = su * cv;

  R[6] = su * sw + cu * sv * cw;
  R[7] = cu * sv * sw - su * cw;
  R[8] = cu * cv;
}

inline void rpyToRotMat_extrinsic(const Vec<3> &rpy, Mat<3, 3> &R) {
  const double su = std::sin(rpy[0]), cu = std::cos(rpy[0]);
  const double sv = std::sin(rpy[1]), cv = std::cos(rpy[1]);
  const double sw = std::sin(rpy[2]), cw = std::cos(rpy[2]);

  R[0] = cv * cw;
  R[1] = cu * sw + su * cw * sv;
  R[2] = su * sw - cu * cw * sv;

  R[3] = -cv * sw;
  R[4] = cu * cw - su * sv * sw;
  R[5] = su * cw + cu * sv * sw;

  R[6] = sv;
  R[7] = -su * cv;
  R[8] = cu * cv;
}

inline void angleAxisToRotMat(const Vec<3> &a1, const double theta, Mat<3, 3> &rotMat) {
  const double s = sin(theta);
  const double c = cos(theta);
  const double t = 1.0 - c;
  const double tmp1 = a1[0] * a1[1] * t;
  const double tmp2 = a1[2] * s;
  const double tmp3 = a1[0] * a1[2] * t;
  const double tmp4 = a1[1] * s;
  const double tmp5 = a1[1] * a1[2] * t;
  const double tmp6 = a1[0] * s;
  const double tmp7 = a1[0] * a1[0] * t + c;
  const double tmp8 = a1[1] * a1[1] * t + c;
  const double tmp9 = a1[2] * a1[2] * t + c;

  rotMat[0] = tmp7;
  rotMat[1] = tmp1 + tmp2;
  rotMat[2] = tmp3 - tmp4;
  rotMat[3] = tmp1 - tmp2;
  rotMat[4] = tmp8;
  rotMat[5] = tmp5 + tmp6;
  rotMat[6] = tmp3 + tmp4;
  rotMat[7] = tmp5 - tmp6;
  rotMat[8] = tmp9;
}

inline void angleAxisToQuaternion(const Vec<3> &a1, const double theta, Vec<4>& quaternion) {
  quaternion[0] = cos(theta * .5);
  quaternion[1] = a1[0] * sin(theta * .5);
  quaternion[2] = a1[1] * sin(theta * .5);
  quaternion[3] = a1[2] * sin(theta * .5);
}

inline void quatMul(const Vec<4>& q, const Vec<4>& p, Vec<4>& pq) {
  pq[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
  pq[1] = p[0] * q[1] + p[1] * q[0] - p[2] * q[3] + p[3] * q[2];
  pq[2] = p[0] * q[2] + p[1] * q[3] + p[2] * q[0] - p[3] * q[1];
  pq[3] = p[0] * q[3] - p[1] * q[2] + p[2] * q[1] + p[3] * q[0];
}

inline void quatMul(const double* q, const double* p, double* pq) {
  pq[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
  pq[1] = p[0] * q[1] + p[1] * q[0] - p[2] * q[3] + p[3] * q[2];
  pq[2] = p[0] * q[2] + p[1] * q[3] + p[2] * q[0] - p[3] * q[1];
  pq[3] = p[0] * q[3] - p[1] * q[2] + p[2] * q[1] + p[3] * q[0];
}

inline void quatInvQuatMul(const double* q, const double* p, double* pq) {
  pq[0] = p[0] * q[0] - p[1] * -q[1] - p[2] * -q[2] - p[3] * -q[3];
  pq[1] = p[0] * -q[1] + p[1] * q[0] - p[2] * -q[3] + p[3] * -q[2];
  pq[2] = p[0] * -q[2] + p[1] * -q[3] + p[2] * q[0] - p[3] * -q[1];
  pq[3] = p[0] * -q[3] - p[1] * -q[2] + p[2] * -q[1] + p[3] * q[0];
}

inline void matvecAddDiagonally(MatDyn& mat, const VecDyn& diag) {
  DRSFATAL_IF(mat.n!=mat.m && mat.n!=diag.n, "wrong dimension")

  for(size_t i=0; i<mat.n; i++)
    mat[i*mat.n+i] = diag[i];
}

inline void eulerVecToQuat(const Vec<3>& eulerVec, Vec<4>& quat) {
  const double angle = eulerVec.norm();
  if(angle < 1e-12) {
    quat = {1,0,0,0};
    return;
  }
  Vec<3> axis = eulerVec;
  const double angleInv = 1.0/angle;
  axis *= angleInv;
  const double sin = std::sin(0.5 * angle);
  const double cos = std::cos(0.5 * angle);
  quat = {cos, sin*axis[0], sin*axis[1], sin*axis[2]};
}

inline void quatToEulerVec(const double* quat, double* eulerVec) {
  const double norm = (std::sqrt(quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]));
  if(fabs(norm) < 1e-12) {
    eulerVec[0] = 0;
    eulerVec[1] = 0;
    eulerVec[2] = 0;
    return;
  }

  const double normInv = 1.0/norm;
  const double angleNormInv = std::acos(std::min(quat[0],1.0)) * 2.0 * normInv;
  eulerVec[0] = quat[1] * angleNormInv;
  eulerVec[1] = quat[2] * angleNormInv;
  eulerVec[2] = quat[3] * angleNormInv;
}

inline void matmul(const Mat<3, 3> &mat1, const Mat<3, 3> &mat2, Mat<3, 3> &mat) {
//  for (size_t j = 0; j < 3; j++) // col
//    for (size_t k = 0; k < 3; k++) // row
//      mat[k + j*3] = mat1[k] * mat2[3*j] + mat1[k+3] * mat2[1 + j*3] + mat1[k+6] * mat2[2 + 3* j];

  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat(k ,j) = mat1(k, 0) * mat2(0, j) + mat1(k,1) * mat2(1, j) + mat1(k, 2) * mat2(2, j);

}

inline void matmul(const Mat<4, 4> &mat1, const Mat<4, 4> &mat2, Mat<4, 4> &mat) {
  for (size_t j = 0; j < 4; j++) // col
    for (size_t k = 0; k < 4; k++) // row
      mat[k + j*4] = mat1[k] * mat2[4*j] + mat1[k+4] * mat2[1 + j*4] + mat1[k+8] * mat2[2 + 4 * j] + mat1[k + 12] * mat2[3 + 4* j];
}

template<size_t i, size_t j, size_t k>
inline void matmul(const Mat<i, j> &mat1, const Mat<j, k> &mat2, Mat<i, k> &mat) {
  for (size_t a = 0; a < k; a++) // col
    for (size_t b = 0; b < i; b++)  {
      double temp = 0;
      for (size_t c = 0; c < j; c++)
        temp += mat1(b,c) * mat2(c, a);

      mat(b, a) = temp;
    }
}

inline void matmul(const MatDyn &mat1, const MatDyn &mat2, MatDyn &mat) {
  mat.setZero();
  for (size_t a = 0; a < mat1.n; a++) // col
    for (size_t b = 0; b < mat2.m; b++)
      for (size_t c = 0; c < mat1.m; c++)
        mat(b, a) += mat1(b,c) * mat2(c, a);

}

inline void matmattransposemul(const Mat<3, 3> &mat1, const Mat<3, 3> &mat2, Mat<3, 3> &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat(k, j) = mat1(k, 0) * mat2(j, 0) + mat1(k, 1) * mat2(j, 1) + mat1(k, 2) * mat2(j, 2);
}

inline void mattransposematmul(const Mat<3, 3> &mat1, const Mat<3, 3> &mat2, Mat<3, 3> &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat(k, j) =  mat1(0, k) * mat2(0, j) + mat1(1, k) * mat2(1, j) + mat1(2, k) * mat2(2, j);
}

inline void rotmatmul(const Mat<3, 3> &mat1, const Mat<3, 3> &mat2, Mat<3, 3> &mat) {
  for (size_t j = 0; j < 2; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat[j * 3 + k] =
          mat1[k] * mat2[j * 3] + mat1[k + 3] * mat2[j * 3 + 1] + mat1[k + 6] * mat2[j * 3 + 2];

  mat[6] = mat[1] * mat[5] - mat[2] * mat[4];
  mat[7] = mat[2] * mat[3] - mat[0] * mat[5];
  mat[8] = mat[0] * mat[4] - mat[1] * mat[3];
}


inline void matmul(const MatDyn &mat1, const Mat<3,3> &mat2, MatDyn &mat) {
  for (size_t j = 0; j < mat1.n; j++) // row
    for (size_t k = 0; k < 3; k++) // col
      mat[j + mat1.n * k] = mat1[j] * mat2[3*k] + mat1[j+mat1.n] * mat2[3*k+1] + mat1[j+2*mat1.n] * mat2[3*k+2];
}

inline void matmul(const Mat<3,3> &mat1, const MatDyn &mat2, MatDyn &mat) {
  for (size_t j = 0; j < mat2.n; j++) // row
    for (size_t k = 0; k < 3; k++) // col
      mat[j*3 + k] = mat1[j] * mat2[3*j] + mat1[j+3] * mat2[3*j+1] + mat1[j+6] * mat2[3*j+2];
}

inline void matmul_fillBot3Rows(const Mat<3, 3> &mat1, const Mat<3, 3> &mat2, Mat<6, 3> &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat[j * 6 + k + 3] =
          mat1[k] * mat2[j * 3] + mat1[k + 3] * mat2[j * 3 + 1] + mat1[k + 6] * mat2[j * 3 + 2];
}

/// first matrix is transposed
template<size_t n, size_t m, size_t l>
inline void transposedMatMul(const Mat<m, n> &mat1, const Mat<m, l> &mat2, Mat<n, l> &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat[j * 3 + k] =
          mat1[k * 3] * mat2[j * 3] + mat1[k * 3 + 1] * mat2[j * 3 + 1] + mat1[k * 3 + 2] * mat2[j * 3 + 2];
}

/// second matrix is transposed
template<size_t n, size_t m, size_t l>
inline void transposed2MatMul(const Mat<n, m> &mat1, const Mat<l, m> &mat2, Mat<n, l> &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < 3; k++) // row
      mat[j * 3 + k] =
          mat1[k] * mat2[j] + mat1[k + 3 * 1] * mat2[j + 3 * 1] + mat1[k + 3 * 2] * mat2[j + 3 * 2];
}

/// second matrix is transposed
inline void transposed2MatMul(const MatDyn &mat1, const Mat<3, 3> &mat2, MatDyn &mat) {
  for (size_t j = 0; j < 3; j++) // col
    for (size_t k = 0; k < mat1.n; k++) // row
      mat[j * mat1.n + k] =
          mat1[k] * mat2[j] + mat1[k + mat1.n * 1] * mat2[j + 3 * 1] + mat1[k + mat1.n * 2] * mat2[j + 3 * 2];
}

template<size_t n, size_t l>
inline void matScalarMul(double scalar, Mat<n, l> &mat) {
  for (size_t j = 0; j < n*l; j++)
    mat[j] *= scalar;
};

template<size_t n, size_t l>
inline void matScalarMul(double scalar, const Mat<n, l> &mat1, Mat<n, l> &mat) {
  for (size_t j = 0; j < n*l; j++)
    mat[j] *= scalar * mat1[j];
};

/// computes RIR^T
inline void similarityTransform(const Mat<3, 3> &R, const Mat<3, 3> &I, Mat<3, 3> &mat) {
//  Mat<3, 3> temp;
//  transposed2MatMul(I, R, temp);
//  matmul(R, temp, mat);

  const double t00 = R[0]*I[0]+R[3]*I[3]+R[6]*I[6];
  const double t10 = R[0]*I[1]+R[3]*I[4]+R[6]*I[7];
  const double t20 = R[0]*I[2]+R[3]*I[5]+R[6]*I[8];

  const double t01 = R[1]*I[0]+R[4]*I[1]+R[7]*I[2];
  const double t11 = R[1]*I[1]+R[4]*I[4]+R[7]*I[5];
  const double t21 = R[1]*I[2]+R[4]*I[5]+R[7]*I[8];

  const double t02 = R[2]*I[0]+R[5]*I[1]+R[8]*I[2];
  const double t12 = R[2]*I[1]+R[5]*I[4]+R[8]*I[5];
  const double t22 = R[2]*I[2]+R[5]*I[5]+R[8]*I[8];

  mat[0] = R[0]*t00 + R[3]*t10 + R[6]*t20;
  mat[1] = R[1]*t00 + R[4]*t10 + R[7]*t20;
  mat[2] = R[2]*t00 + R[5]*t10 + R[8]*t20;
  mat[3] = mat[1];
  mat[4] = R[1]*t01 + R[4]*t11 + R[7]*t21;
  mat[5] = R[2]*t01 + R[5]*t11 + R[8]*t21;
  mat[6] = mat[2];
  mat[7] = mat[5];
  mat[8] = R[2]*t02 + R[5]*t12 + R[8]*t22;
//  std::cout<<test.e()<<"\n";
//  std::cout<<mat.e()<<"\n\n";
}


/// computes R^TIR
inline void similarityTransform2(const Mat<3, 3> &R, const Mat<3, 3> &I, Mat<3, 3> &mat) {
//  Mat<3, 3> temp;
//  transposed2MatMul(I, R, temp);
//  matmul(R, temp, mat);

  const double t00 = R[0]*I[0]+R[1]*I[3]+R[2]*I[6];
  const double t10 = R[0]*I[1]+R[1]*I[4]+R[2]*I[7];
  const double t20 = R[0]*I[2]+R[1]*I[5]+R[2]*I[8];

  const double t01 = R[3]*I[0]+R[4]*I[1]+R[5]*I[2];
  const double t11 = R[3]*I[1]+R[4]*I[4]+R[5]*I[5];
  const double t21 = R[3]*I[2]+R[4]*I[5]+R[5]*I[8];

  const double t02 = R[6]*I[0]+R[7]*I[1]+R[8]*I[2];
  const double t12 = R[6]*I[1]+R[7]*I[4]+R[8]*I[5];
  const double t22 = R[6]*I[2]+R[7]*I[5]+R[8]*I[8];

  mat[0] = R[0]*t00 + R[1]*t10 + R[2]*t20;
  mat[1] = R[3]*t00 + R[4]*t10 + R[5]*t20;
  mat[2] = R[6]*t00 + R[7]*t10 + R[8]*t20;
  mat[3] = mat[1];
  mat[4] = R[3]*t01 + R[4]*t11 + R[5]*t21;
  mat[5] = R[6]*t01 + R[7]*t11 + R[8]*t21;
  mat[6] = mat[2];
  mat[7] = mat[5];
  mat[8] = R[6]*t02 + R[7]*t12 + R[8]*t22;
//  std::cout<<test.e()<<"\n";
//  std::cout<<mat.e()<<"\n\n";
}

template<size_t n, size_t m>
inline void transpose(const Mat<n, m> &mat1, Mat<m, n> &mat) {
  for (size_t i = 0; i < n; i++)
    for (size_t j = 0; j < m; j++)
      mat[j + i * m] = mat1[i + j * n];
}

template<size_t n, size_t m>
inline void matvecmul(const Mat<n, m> &mat1, const Vec<m> &vec1, Vec<n> &vec) {
  for (size_t j = 0; j < n; j++) {
    vec[j] = 0;
    for (size_t i = 0; i < m; i++) // col
      vec[j] += mat1[i * n + j] * vec1[i];
  }
}

template<size_t n, size_t m>
inline void matTransposevecmul(const Mat<n, m> &mat1, const Vec<m> &vec1, Vec<n> &vec) {
  for (size_t j = 0; j < n; j++) {
    vec[j] = 0;
    for (size_t i = 0; i < m; i++) // col
      vec[j] += mat1[i + j* n] * vec1[i];
  }
}

inline void matTransposevecmul(const MatDyn &mat1, const VecDyn &vec1, VecDyn &vec) {
  for (size_t j = 0; j < mat1.n; j++) {
    vec[j] = 0;
    for (size_t i = 0; i < mat1.m; i++) // col
      vec[j] += mat1[i + j* mat1.n] * vec1[i];
  }
}

inline void matvecmulInvertSgn(const Mat<3,3> &mat1, const Vec<3> &vec1, Vec<3> &vec) {
  for (size_t j = 0; j < 3; j++) // col
    vec[j] = -mat1[j] * vec1[0] - mat1[j + 3] * vec1[1] - mat1[j + 6] * vec1[2];
}

template<size_t n, size_t m>
inline void matvecmulThenAdd(const Mat<n, m> &mat1, const Vec<m> &vec1, Vec<n> &vec) {
  for (size_t j = 0; j < n; j++)
    for (size_t i = 0; i < m; i++) // col
      vec[j] += mat1[i * n + j] * vec1[i];
}

inline void matvecmulThenAdd(const Mat<3,3> &mat1, const Vec<3> &vec1, Vec<3> &vec) {
  for (size_t j = 0; j < 3; j++) // col
    vec[j] += mat1[j] * vec1[0] + mat1[j + 3] * vec1[1] + mat1[j + 6] * vec1[2];
}

inline void matvecmulThenAdd(const Mat<3,3> &mat1, const Vec<3>* vec1, Vec<3> &vec) {
  for (size_t j = 0; j < 3; j++) // col
    vec[j] += mat1[j] * vec1->v[0] + mat1[j + 3] * vec1->v[1] + mat1[j + 6] * vec1->v[2];
}

inline void matvecmulThenAdd(const MatDyn &mat1, const Vec<3> &vec1, VecDyn &vec) {
  for (size_t j = 0; j < mat1.n; j++)
    vec[j] += mat1[j] * vec1[0] + mat1[j+mat1.n] * vec1[1] + mat1[j+mat1.n*2] * vec1[2];
}

inline void matvecmulThenSub(const MatDyn &mat1, const Vec<3> &vec1, VecDyn &vec) {
  for (size_t j = 0; j < mat1.n; j++)
    vec[j] -= mat1[j] * vec1[0] + mat1[j+mat1.n] * vec1[1] + mat1[j+mat1.n*2] * vec1[2];
}

template<size_t n, size_t m>
inline void matvecmulThenSub(const Mat<n, m> &mat1, const Vec<m> &vec1, Vec<n> &vec) {
  for (size_t j = 0; j < n; j++)
    for (size_t i = 0; i < m; i++) // col
      vec[j] -= mat1[i * n + j] * vec1[i];
}

template<size_t m>
inline void matTransposeVecmulThenAdd(const MatDyn &mat1, const Vec<m> &vec1, VecDyn &vec) {
  for (size_t j = 0; j < mat1.m; j++)
    for (size_t i = 0; i < m; i++) // col
      vec[j] += mat1[i + j * m] * vec1[i];
}

template<size_t n, size_t m>
inline void matTransposeVecmulThenAdd(const Mat<n, m> &mat1, const Vec<m> &vec1, Vec<n> &vec) {
  for (size_t j = 0; j < n; j++)
    for (size_t i = 0; i < m; i++) // col
      vec[j] += mat1[i + j * n] * vec1[i];
}

inline void matvecmul(const MatDyn &mat1, const VecDyn &vec1, VecDyn &vec) {
  for (size_t j = 0; j < mat1.n; j++) {
    vec[j] = 0;
    for (size_t i = 0; i < mat1.m; i++) // col
      vec[j] += mat1[i * mat1.n + j] * vec1[i];
  }
}

inline void matvecmul(const Mat<3, 3> &mat1, const Vec<3> &vec1, double *vec) {
  for (size_t j = 0; j < 3; j++) // col
    vec[j] = mat1[j] * vec1[0] + mat1[j + 3] * vec1[1] + mat1[j + 6] * vec1[2];
}

inline void matvecmul(const Mat<3, 3>& mat1, const Vec<3>& vec1, Vec<3>& vec) {
    for (size_t j = 0; j < 3; j++) // col
        vec[j] = mat1[j] * vec1[0] + mat1[j + 3] * vec1[1] + mat1[j + 6] * vec1[2];
}

template<size_t size>
inline void vecsub(const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] = vec1[j] - vec2[j];
}

inline void productOfSkewMatrices(const Vec<3> &vec1, const Vec<3> &vec2, Mat<3,3>& mat) {
  mat[0] = -vec1[2] * vec2[2] - vec1[1] *vec2[1];
  mat[1] = vec1[0] * vec2[1];
  mat[2] = vec1[0] * vec2[2];

  mat[3] = vec1[1] * vec2[0];
  mat[4] = -vec1[2] * vec2[2] - vec1[0] * vec2[0];
  mat[5] = vec1[1] * vec2[2];

  mat[6] = vec1[2] * vec2[0];
  mat[7] = vec1[2] * vec2[1];
  mat[8] = -vec1[1] * vec2[1] - vec1[0] * vec2[0];
}

inline void vecsub(const VecDyn &vec1, const VecDyn &vec2, VecDyn &vec) {
  for (size_t j = 0; j < vec1.n; j++) // col
    vec[j] = vec1[j] - vec2[j];
}

inline void vecadd(const VecDyn &vec1, const VecDyn &vec2, VecDyn &vec) {
  for (size_t j = 0; j < vec1.n; j++) // col
    vec[j] = vec1[j] + vec2[j];
}

template<size_t size>
inline void vecsub(const Vec<size> &vec1, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] -= vec1[j];
}

template<size_t n, size_t m>
inline void matadd(const Mat<n, m> &mat1, Mat<n, m> &mat) {
  for (size_t i = 0; i < n; i++) // col
    for (size_t j = 0; j < m; j++) // col
      mat[i + n * j] += mat1[i + n * j];
}

inline void matadd(const MatDyn &mat1, MatDyn &mat) {
  for (size_t i = 0; i < mat1.n; i++) // col
    for (size_t j = 0; j < mat1.m; j++) // col
      mat[i + mat1.n * j] += mat1[i + mat1.n * j];
}

template<size_t n, size_t m>
inline void matadd(const Mat<n, m> &mat1, const Mat<n, m> &mat2, Mat<n, m> &mat) {
  for (size_t i = 0; i < n; i++) // col
    for (size_t j = 0; j < m; j++) // col
      mat[i + n * j] = mat1[i + n * j] + mat2[i + n * j];
}

template<size_t n, size_t m>
inline void matsub(const Mat<n, m> &mat1, Mat<n, m> &mat) {
  for (size_t i = 0; i < n; i++) // col
    for (size_t j = 0; j < m; j++) // col
      mat[i + n * j] -= mat1[i + n * j];
}

template<size_t n, size_t m>
inline void matsub(const Mat<n, m> &mat1, const Mat<n, m> &mat2, Mat<n, m> &mat) {
  for (size_t i = 0; i < n; i++) // col
    for (size_t j = 0; j < m; j++) // col
      mat[i + n * j] = mat1[i + n * j] - mat2[i + n * j];
}

template<size_t n>
inline void mataddIdentity(Mat<n, n> &mat) {
  for (size_t i = 0; i < n; i++)
    mat[i + n * i] += 1;
}


template<size_t size>
inline void vecadd(const Vec<size> &vec1, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] += vec1[j];
}

inline void vecadd(const VecDyn &vec1, VecDyn &vec) {
  for (size_t j = 0; j < vec1.n; j++) // col
    vec[j] += vec1[j];
}

template<size_t size>
inline void vecadd(const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] = vec1[j] + vec2[j];
}

// vec = a*vec1
template<size_t size>
inline void add_aX(double a, const Vec<size> &vec1, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] += a * vec1[j];
}

// vec = vec2+a*vec1
template<size_t size>
inline void add_b_p_aX(double a, const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &vec) {
  for (size_t j = 0; j < 3; j++) // col
    vec[j] = vec2[j] + a * vec1[j];
}

template<size_t size>
inline void vecDot(const Vec<size> &vec1, const Vec<size> &vec2, double &scalar) {
  scalar = std::inner_product(vec1.ptr(), vec1.ptr() + size, vec2.ptr(), 0.0);
}

template<size_t size>
inline void vecDot(const double* vec1, const Vec<size> &vec2, double &scalar) {
  scalar = std::inner_product(vec1, vec1 + size, vec2.ptr(), 0.0);
}

template<size_t size>
inline double vecDot(const double* vec1,const double* vec2) {
  return std::inner_product(vec1, vec1 + size, vec2, 0.0);
}

template<size_t size>
inline double vecDot(const Vec<size> vec1, const Vec<size> &vec2) {
  return std::inner_product(vec1.ptr(), vec1.ptr() + size, vec2.ptr(), 0.0);
}

inline double vecDot(const Vec<3> vec1, const Vec<3>& vec2) {
  return std::inner_product(vec1.ptr(), vec1.ptr() + 3, vec2.ptr(), 0.0);
}

inline void vecDot(const VecDyn &vec1, const VecDyn &vec2, double &scalar) {
  scalar = 0;
  for(size_t i  = 0; i < vec1.n; i++) {
    scalar += vec1[i] * vec2[i];
  }
}

template<size_t size>
inline void vecDivide(const Vec<size> &vec1, const double scalar, Vec<size> &vec) {
  for (size_t j = 0; j < size; j++) // col
    vec[j] = vec1[j] / scalar;
}

template<size_t n, size_t m>
inline void matDivide(const Mat<n,m> &mat1, const double scalar, Mat<n,m> &mat) {
  for (size_t j = 0; j < n*m; j++) // col
    mat[j] = mat1[j] / scalar;
}

inline void cross(const Vec<3> &vec1, const Vec<3> &vec2, Vec<3> &vec) {
  vec[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void cross(const double* vec1, const Vec<3> &vec2, Vec<3> &vec) {
  vec[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void cross(const Vec<3> &vec1, const Vec<3> &vec2, double *vec) {
  vec[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void cross(const double *vec1, const double *vec2, double *vec) {
  vec[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

// answer also skips 3
inline void cross_skip3_skip3_negative(const double *vec1_skip, const double *vec2, double *vec) {
  vec[0] = -vec1_skip[3] * vec2[2] + vec1_skip[6] * vec2[1];
  vec[3] = -vec1_skip[6] * vec2[0] + vec1_skip[0] * vec2[2];
  vec[6] = -vec1_skip[0] * vec2[1] + vec1_skip[3] * vec2[0];
}

inline void crossThenAdd(const Vec<3> &vec1, const Vec<3> &vec2, Vec<3> &vec) {
  vec[0] += vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] += vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] += vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void crossThenAdd(const Vec<3> &vec1, const Vec<3> &vec2, double* vec) {
  vec[0] += vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] += vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] += vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void crossThenAdd(const double *vec1, const double *vec2, double *vec) {
  vec[0] += vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] += vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] += vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void crossThenSub(const Vec<3> &vec1, const Vec<3> &vec2, Vec<3> &vec) {
  vec[0] -= vec1[1] * vec2[2] - vec1[2] * vec2[1];
  vec[1] -= vec1[2] * vec2[0] - vec1[0] * vec2[2];
  vec[2] -= vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

inline void crossMulScalarThenAdd(const double a, const Vec<3> &vec1, const Vec<3> &vec2, Vec<3> &vec) {
  vec[0] += (vec1[1] * vec2[2] - vec1[2] * vec2[1]) * a;
  vec[1] += (vec1[2] * vec2[0] - vec1[0] * vec2[2]) * a;
  vec[2] += (vec1[0] * vec2[1] - vec1[1] * vec2[0]) * a;
}

inline void VecXMatVecThenAdd(const Vec<3> &vec1, const Mat<3, 3> &mat1, const Vec<3> &vec2, Vec<3> &vec) {
  Vec<3> temp;
  matvecmul(mat1, vec2, temp);
  crossThenAdd(vec1, temp, vec);
}

inline void VecXMatVec(const Vec<3> &vec1, const Mat<3, 3> &mat1, const Vec<3> &vec2, Vec<3> &vec) {
  Vec<3> temp;
  matvecmul(mat1, vec2, temp);
  cross(vec1, temp, vec);
}

inline void vecScalarMul(const double scalar, const Vec<3> &vec, Vec<3> &product) {
  for (size_t i = 0; i < 3; i++)
    product[i] = scalar * vec[i];
}

template<size_t size>
inline void vecScalarMul(const double scalar, const Vec<size> &vec, Vec<size> &product) {
  for (size_t i = 0; i < size; i++)
    product[i] = scalar * vec[i];
}

inline void vecScalarMul(const double scalar, const VecDyn &vec, VecDyn &product) {
  for (size_t i = 0; i < vec.n; i++)
    product[i] = scalar * vec[i];
}

template<size_t size>
inline void vecScalarMulThenAdd(const double scalar, const double *vec, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] += scalar * vec[i];
}

inline void vecScalarMulThenAdd(const double scalar, const double *vec, VecDyn &sum) {
  for (size_t i = 0; i < sum.n; i++)
    sum[i] += scalar * vec[i];
}

inline void vecScalarMulThenAdd(const double scalar, const VecDyn &vec1, const VecDyn &vec2, VecDyn &sum) {
  for (size_t i = 0; i < sum.n; i++)
    sum[i] = scalar * vec1[i] + vec2[i];
}

inline void vecScalarMulThenAdd(const double scalar, const VecDyn &vec, VecDyn &sum) {
  for (size_t i = 0; i < sum.n; i++)
    sum[i] += scalar * vec[i];
}

template<size_t size>
inline void vecScalarMulThenAdd(const double scalar, const Vec<size> &vec, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] += scalar * vec[i];
}

template<size_t size>
inline void vecvecCwiseMulThenAdd(const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] += vec1[i] * vec2[i];
}

inline void vecvecCwiseMulThenAdd(const VecDyn &vec1, const VecDyn &vec2, VecDyn &sum) {
  for (size_t i = 0; i < vec1.n; i++)
    sum[i] += vec1[i] * vec2[i];
}

template<size_t size>
inline void vecvecCwiseMulThenSub(const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] -= vec1[i] * vec2[i];
}

inline void vecvecCwiseMulThenSub(const VecDyn &vec1, const VecDyn &vec2, VecDyn &sum) {
  for (size_t i = 0; i < vec1.n; i++)
    sum[i] -= vec1[i] * vec2[i];
}

template<size_t size>
inline void vecvecScalarCwiseMulThenSub(double scalar, const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] -= scalar * vec1[i] * vec2[i];
}

template<size_t size>
inline void vecvecCwiseMul(const Vec<size> &vec1, const Vec<size> &vec2, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] = vec1[i] * vec2[i];
}

inline void vecvecCwiseMul(const VecDyn &vec1, const VecDyn &vec2, VecDyn &sum) {
  for (size_t i = 0; i < vec1.n; i++)
    sum[i] = vec1[i] * vec2[i];
}

template<size_t size>
inline void vecScalarMulThenSub(const double scalar, const Vec<size> &vec, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] -= scalar * vec[i];
}

template<size_t size>
inline void vecScalarMulThenSub(const double scalar, const double *vec, Vec<size> &sum) {
  for (size_t i = 0; i < size; i++)
    sum[i] -= scalar * vec[i];
}

inline void vecScalarMul(const double scalar, Vec<3> &product) {
  for (size_t i = 0; i < 3; i++)
    product[i] *= scalar;
}

inline void vecScalarMul(const double scalar, VecDyn &product) {
  for (size_t i = 0; i < product.n; i++)
    product[i] *= scalar;
}

inline void skewSymMat(const Vec<3> &vec, Mat<3,3> &mat) {
  mat[0] = 0;
  mat[4] = 0;
  mat[8] = 0;
  mat[1] = vec[2];
  mat[2] = -vec[1];
  mat[3] = -vec[2];
  mat[5] = vec[0];
  mat[6] = vec[1];
  mat[7] = -vec[0];
}

inline void vecTransposeMatVecMul(const Vec<3> &vec, const Mat<3, 3> &mat, double &scalar) {
  /// x^T M x
  scalar = vec[0] * (mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2])
      + vec[1] * (mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2])
      + vec[2] * (mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2]);
}

inline void addCentrifugalTerm(const Vec<3> &omega, const Vec<3> &displacement, Vec<3> &linAcc) {
  Vec<3> temp;
  cross(omega, displacement, temp);
  crossThenAdd(omega, temp, linAcc);
}

inline void rotationIntegration(Mat<3,3>& rotationMatrix, double dt, const Vec<3>& angVel) {

  /// exponential map
  const double norm = angVel.norm();
  const double angle = norm*dt;
  if (angle < 1e-11) return;
  const double normInv = 1.0/norm;

  const double x = angVel[0] * normInv;
  const double y = angVel[1] * normInv;
  const double z = angVel[2] * normInv;

  const double s = sin(angle);
  const double c = 1.0 - cos(angle);

  const double t2 = c*x*y;
  const double t3 = z*z;
  const double t4 = s*y;
  const double t5 = c*x*z;
  const double t6 = c*y*z;
  const double t7 = x*x;
  const double t8 = y*y;

  Mat<3,3> expM;

  expM[0] = -c*t3-c*t8+1.0;
  expM[3] = t2-s*z;
  expM[6] = t4+t5;
  expM[1] = t2+s*z;
  expM[4] = -c*t3-c*t7+1.0;
  expM[7] = t6-s*x;
  expM[2] = -t4+t5;
  expM[5] = t6+s*x;
  expM[8] = -c*t7-c*t8+1.0;

  Mat<3,3> temp;

  /// TODO:: figure out where you want to normalize the rotation
  matmul(expM, rotationMatrix, temp);
  rotationMatrix = temp;
}

inline void rotationIntegration(Mat<3,3>& rotationMatrix, double dt, const double* angVel) {

  /// exponential map
  const double norm = sqrt(angVel[0]*angVel[0]+angVel[1]*angVel[1]+angVel[2]*angVel[2]);
  const double angle = norm*dt;
  if (angle < 1e-11) return;

  const double x = angVel[0]/norm;
  const double y = angVel[1]/norm;
  const double z = angVel[2]/norm;

  const double s = sin(angle);
  const double c = 1.0 - cos(angle);

  const double t2 = c*x*y;
  const double t3 = z*z;
  const double t4 = s*y;
  const double t5 = c*x*z;
  const double t6 = c*y*z;
  const double t7 = x*x;
  const double t8 = y*y;

  Mat<3,3> expM;

  expM[0] = -c*t3-c*t8+1.0;
  expM[3] = t2-s*z;
  expM[6] = t4+t5;
  expM[1] = t2+s*z;
  expM[4] = -c*t3-c*t7+1.0;
  expM[7] = t6-s*x;
  expM[2] = -t4+t5;
  expM[5] = t6+s*x;
  expM[8] = -c*t7-c*t8+1.0;

  Mat<3,3> temp;

  matmul(expM, rotationMatrix, temp);
  rotationMatrix = temp;
}

template<size_t size>
inline double squaredNormOfDiff(const Vec<size>& vec1, const Vec<size>& vec2){
  double sum=0;

  for(size_t i=0; i<size; i++) {
    const double diff = vec1[i] - vec2[i];
    sum += diff * diff;
  }
  return sum;
}

inline void zaxisToRotMat(const Vec<3>& zaxis, Mat<3,3>& rotMat) {
  const Vec<3> singularity = {0,0,-1};
  double angle;
  Vec<3> rotAxis = {0.,0.,1.};
  rotAxis -= zaxis;
  if(rotAxis.squaredNorm() < 1e-18) {
    rotMat.setIdentity();
    return;
  }

  if(sqrt(squaredNormOfDiff(zaxis, singularity)) < 1e-8) {
    rotAxis = {1,0,0};
    angle = M_PI;
  } else {
    Vec<3> originalZ = {0,0,1};
    cross(originalZ, zaxis, rotAxis);
    rotAxis /= rotAxis.norm();
    angle = acos(vecDot(zaxis, originalZ));
  }

  angleAxisToRotMat(rotAxis, angle, rotMat);
}

inline void addToDiagonal(const VecDyn& vec, MatDyn& mat) {
  for(size_t i=0; i<vec.n; i++)
    mat[i*vec.n+i] += vec[i];
}

inline void matmul(const SparseJacobian& jaco, const VecDyn vec, Vec<3>& result) {
  result.setZero();
  for(size_t i=0; i< jaco.size; i++)
    for(size_t j=0; j<3; j++)
      result[j] += jaco[i*3+j] * vec[jaco.idx[i]];
}


} // raisim

#endif //RAISIM_MATH_HPP
