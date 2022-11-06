//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAIMATH__PRODUCT_HPP_
#define RAIMATH__PRODUCT_HPP_
#include "Matrix.hpp"
#include "Operation.hpp"

namespace raisim_math {


enum {
  FOUR = 2,
  ELSE = 3
};


template<int Rows, int Cols, int Depth> struct product_type_selector;

enum ProductImplType
{ DefaultProduct=0, FourFour };

template<int M, int N>  struct product_type_selector<M,N,1> { enum { ret = DefaultProduct }; };
template<>  struct product_type_selector<4,4,4> { enum { ret = FourFour }; };


}

#endif //RAIMATH__PRODUCT_HPP_
