//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CONSTRAINTS_HPP
#define RAISIM_CONSTRAINTS_HPP

namespace raisim {

class Constraints {
  friend class raisim::RaisimServer;

 public:
  Constraints() { color_ = {1.0, 1.0, 1.0, 1.0}; }
  virtual ~Constraints() = default;
  [[nodiscard]] const Vec<4>& getColor() const { return color_; };
  void setColor(const Vec<4>& color) { color_ = color; };

    /**
     * locks chart mutex. This can be used if you use raisim in a multi-threaded environment.
     */
    void lockMutex() { mutex_.lock(); }
    void lock() { mutex_.lock(); } // this is for RAII compatibility

    /**
     * unlock chart mutex. This can be used if you use raisim in a multi-threaded environment.
     */
    void unlockMutex() { mutex_.unlock(); }
    void unlock() { mutex_.unlock(); } // this is for RAII compatibility

 protected:
  std::string name_;
  Vec<4> color_;
  uint32_t visualTag = 0;
  std::mutex mutex_;
};

}

#endif //RAISIM_CONSTRAINTS_HPP
