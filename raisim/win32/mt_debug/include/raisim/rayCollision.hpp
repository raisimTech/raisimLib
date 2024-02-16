//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_RAYCOLLISION_HPP
#define RAISIM_RAYCOLLISION_HPP

#include "raisim/object/Object.hpp"
#include "ode/collision.h"

namespace raisim {

class RayCollisionItem {

 public:
  friend class raisim::World;

  const Object* getObject() const {
    return obj;
  }

  const Eigen::Vector3d& getPosition() const {
    return pos;
  }

 protected:
  dContactGeom geoms;

 private:

  Eigen::Vector3d pos;
  Object* obj;
};

class RayCollisionList {

 public:

  class iterator
  {
   public:
    typedef iterator self_type;
    typedef RayCollisionItem value_type;
    typedef RayCollisionItem& reference;
    typedef RayCollisionItem* pointer;
    typedef std::forward_iterator_tag iterator_category;
    typedef int difference_type;
    explicit iterator(pointer ptr) : ptr_(ptr) { }
    self_type operator++() { self_type i = *this; ptr_++; return i; }
    self_type operator++(int junk) { ptr_++; return *this; }
    reference operator*() { return *ptr_; }
    pointer operator->() { return ptr_; }
    bool operator==(const self_type& rhs) { return ptr_ == rhs.ptr_; }
    bool operator!=(const self_type& rhs) { return ptr_ != rhs.ptr_; }
   private:
    pointer ptr_;
  };

  friend class raisim::World;

  RayCollisionItem & operator [](size_t i) {return list_[i];}
  const RayCollisionItem & operator [](size_t i) const {return list_[i];}

  size_t size() const {
    return size_;
  }

  iterator begin() {
    return iterator(&list_[0]);
  }

  iterator end() {
    return iterator(&list_[size_]);
  }

  iterator back() {
    return iterator(&list_[size_-1]);
  }

  void setSize(size_t size) {
    size_ = size;
  }

  void resize(size_t size) {
    list_.resize(size);
  }

  void reserve(size_t size) {
    list_.reserve(size);
  }

 protected:
  std::vector<RayCollisionItem> list_;
  size_t size_;
};
}

#endif //RAISIM_RAYCOLLISION_HPP
