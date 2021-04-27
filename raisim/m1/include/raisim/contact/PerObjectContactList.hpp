//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_PEROBJECTCONTACT_HPP
#define RAISIM_PEROBJECTCONTACT_HPP

#include "raisim/helper.hpp"
#include "raisim/math.hpp"
#include "raisim/contact/Contact.hpp"

namespace raisim {
namespace contact {

class PerObjectContactList {
  /// Each Objects has list of Contact
 public:

  PerObjectContactList() {
    contacts_.reserve(10);
  }

  void addContact(Contact& contact) {
    contacts_.push_back(contact);
  }

  void setDelassusAndTauStar(const DelassusType &delassusAndTauStar) {
    delassusAndTauStar_ = delassusAndTauStar;
  }

  void clearContacts() {
    contacts_.clear();
  }

  size_t getNumContacts() const {
    return contacts_.size();
  }

  Contact &getContactAt(int index) {
    return contacts_[index];
  }

  const std::vector<Contact> &getContacts() const {
    return contacts_;
  }

  std::vector<Contact> &getContacts() {
    return contacts_;
  }

  DelassusType &getDelassusAndTauStar() {
    return delassusAndTauStar_;
  }

  double &getImpactVel(size_t idx) {
    return contacts_[idx].getImpactVel();
  }


 private:
  DelassusType delassusAndTauStar_;
  std::vector<Contact> contacts_;
  std::vector<Vec<3>> impulsesSaved_;
  std::vector<int> localIdxSaved_;
  std::vector<int> objIdxSaved_;
  size_t contactN_ = 0;

};

} // contact
} // raisim

#endif //RAISIM_PEROBJECTCONTACT_HPP
