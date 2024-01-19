//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#pragma once

#include "raisim/raisim_message.hpp"
#include "tinyxml_rai/tinyxml_rai.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace raisim {

class RaiSimTinyXmlWriterElement {
 public:
  RaiSimTinyXmlWriterElement(const std::string& name) {
    ptr_ = new TiXmlElement(name);
    elements_.reserve(20);
  }

  RaiSimTinyXmlWriterElement& operator[](const std::string &element) {
    elements_.emplace_back(element);
    return elements_.back();
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, int attribute) {
    ptr_->SetAttribute(name, attribute);
    return *this;
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string& name, unsigned long attribute) {
    ptr_->SetAttribute(name, std::to_string(attribute));
    return *this;
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, unsigned long long attribute) {
    if(attribute == size_t(-1))
      ptr_->SetAttribute(name, -1);
    else
      ptr_->SetAttribute(name, int(attribute));
    return *this;
  }
    
  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, const std::string& attribute) {
    ptr_->SetAttribute(name, attribute);
    return *this;
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, double attribute) {
    ptr_->SetDoubleAttribute(name, attribute);
    return *this;
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, float attribute) {
    ptr_->SetDoubleAttribute(name, attribute);
    return *this;
  }

  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, const char* attribute) {
    ptr_->SetAttribute(name, attribute);
    return *this;
  }

  template<typename T>
  RaiSimTinyXmlWriterElement& setAttribute(const std::string &name, T attribute) {
    std::string list;
    for(int i=0; i<attribute.size()-1; i++)
      list += std::to_string(attribute[i]) + ", ";
    
    list += std::to_string(attribute[attribute.size()-1]);
    ptr_->SetAttribute(name, list);
    return *this;
  }

  template<typename T>
  RaiSimTinyXmlWriterElement& setVectorAttribute(const std::string &name, T attribute, size_t size) {
    std::string list;
    for(int i=0; i<size-1; i++)
      list += std::to_string(attribute[i]) + ", ";

    list += std::to_string(attribute[size-1]);
    ptr_->SetAttribute(name, list);
    return *this;
  }

  void link() {
    for(auto& element : elements_) {
      ptr_->LinkEndChild(element.ptr_);
      element.link();
    }    
  }

  TiXmlElement* ptr() {
    return ptr_;
  }

 private:
  TiXmlElement* ptr_;
  std::vector<RaiSimTinyXmlWriterElement> elements_;
};

class RaiSimTinyXmlWriter {
 public:
  RaiSimTinyXmlWriter() {
    auto decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);
  }

  RaiSimTinyXmlWriterElement& operator[](const std::string &element){
    elements_.emplace_back(element);
    return elements_.back();
  }

  void saveFile(const std::string& fileName) {
    RSINFO("Saving world to: "<<fileName)
    for(auto& ele: elements_) {
      ele.link();
      doc.LinkEndChild(ele.ptr());
    }
    RSWARN_IF(!doc.SaveFile(fileName.c_str()), "failed to save file: "<<fileName<<". Make sure the directory exists");
  }

 private:
  TiXmlDocument doc;
  std::vector<RaiSimTinyXmlWriterElement> elements_;
};



} // namespace raisim