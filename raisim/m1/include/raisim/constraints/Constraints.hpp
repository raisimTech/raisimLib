//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CONSTRAINTS_HPP
#define RAISIM_CONSTRAINTS_HPP

namespace raisim {

class Constraints {
 public:
  virtual ~Constraints() = default;

 protected:
  std::string name_;
};

}

#endif //RAISIM_CONSTRAINTS_HPP
