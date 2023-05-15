//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#pragma once

#include "raisim/raisim_message.hpp"
#include "tinyxml_rai/tinyxml_rai.h"
#include <string>
#include <vector>
#include "raisim/Path.hpp"
#include "raisim/math.hpp"
#include "raisim/configure.hpp"

namespace raisim {

class RaiSimTinyXmlWrapper {

public:

  RaiSimTinyXmlWrapper() = default;

  RaiSimTinyXmlWrapper(const std::string &filePath, const std::string &childElement) {
    if (filePath.find('<') == std::string::npos) {
      fileName_ = Path(filePath).getPath();      
      RSFATAL_IF(!doc_.LoadFile(filePath.c_str()), "cannot read the xml file: " << "\'" << filePath<< "\'")
    } else {
      fileName_ = "URDF from a string";
      doc_.Parse(filePath.c_str(), nullptr, TIXML_ENCODING_UTF8);
    }

    nodeTree_.push_back(childElement);
    node_ = doc_.FirstChildElement(childElement);
    RSFATAL_IF(!node_, filePath << " file does not contain node " << "\'" << childElement << "\'")
  }

  void setFileName(const std::string& name) {
    fileName_ = name;
  }

  [[nodiscard]] std::vector<RaiSimTinyXmlWrapper> getChildren(const std::string &child) const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    std::vector<RaiSimTinyXmlWrapper> children;
    for(TiXmlElement *oc = node_->FirstChildElement(child.c_str()); oc != NULL;
         oc = oc->NextSiblingElement(child.c_str()))
      children.push_back(RaiSimTinyXmlWrapper(oc, oc->ValueStr(), nodeTree_, fileName_));

    return children;
  }

  bool removeFirstChild(const std::string& child) {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    return node_->RemoveChild(node_->FirstChildElement(child.c_str()));
  }

  [[nodiscard]] std::vector<RaiSimTinyXmlWrapper> getChildrenMust(const std::string &child) const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    auto children = getChildren(child);
    RSFATAL_IF(children.empty(), "in \'" << getFileName()
                   << "\' at "<< "row:" << std::to_string(node_->Row()) 
                   << " col:" << std::to_string(node_->Column())
                   << ", the node "<<"\'"<< getNodeName()<<"\'"
                   << " does not contain a child called " <<"\'"<<child <<"\'")
                   
    return children;
  }

  [[nodiscard]] std::vector<RaiSimTinyXmlWrapper> getChildrenMust() const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    auto children = getChildren();
    RSFATAL_IF(children.empty(), "in \'" << getFileName()
                   << "\' at "<< "row:" << std::to_string(node_->Row()) 
                   << " col:" << std::to_string(node_->Column())
                   << ", the node "<<"\'"<< getNodeName()<<"\'"
                   << " does not contain a child")
                   
    return children;
  }

  [[nodiscard]] std::vector<RaiSimTinyXmlWrapper> getChildren() const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    std::vector<RaiSimTinyXmlWrapper> children;
    for (TiXmlElement *oc = node_->FirstChildElement(); oc != nullptr;
         oc = oc->NextSiblingElement()) {
      children.push_back(
          RaiSimTinyXmlWrapper(oc, oc->ValueStr(), nodeTree_, fileName_));
    }
    return children;
  }

  template<typename T>
  std::vector<RaiSimTinyXmlWrapper> getChildrenWithAttribute(const std::string& att, T value) {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    auto children = getChildren();
    std::vector<RaiSimTinyXmlWrapper> childrenWithAtt;
    for(auto& c: children) {
      T valueFromChild;
      if(c.template getAttributeIfExists(att, valueFromChild) && valueFromChild==value) {
        childrenWithAtt.push_back(c);
      }
    }
    return childrenWithAtt;
  }

  template <typename T>
  T getAttributeMust(const std::string &attName) const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    T attribute;
    auto result = getAttributeIfExists(attName, attribute);
    RSFATAL_IF(!result, "in \'" << getFileName()
                << "\' at "<< "row:" << std::to_string(node_->Row()) 
                << " col:" << std::to_string(node_->Column())
                << ", the node "<<"\'"<< getNodeName()<<"\'"
                << " does not contain an attribute called " <<"\'"<< attName <<"\'")
    return attribute;
  }

  template<typename T>
  void setAttribute(const std::string &name, T attribute) {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    node_->SetAttribute(name, attribute);
  }

  void errorMessage(const std::string& msg) const {
    RSFATAL_IF(!node_, "You have accessed a dummy node")
    RSFATAL("in \'" << getFileName()
            << "\' at "<< "row:" << std::to_string(node_->Row()) 
            << " col:" << std::to_string(node_->Column())
            << ", the node "<<"\'"<< getNodeName()<<"\': " << msg)
  }

  template <typename T>
  bool getAttributeIfExists(const std::string &attName, T &attribute) const {
    if(!node_) return false;
    auto result = node_->Attribute(attName);
    if (!result)
      return false;

    std::string s = *result;
    char delimiter;
    if (s.find(',') != std::string::npos)
      delimiter = ',';
    else if(s.find(' ') != std::string::npos) {
      for(size_t i=0; i < s.size()-1; i++) {
        if(s.substr(i, 2) == "  ") {
          s.erase(i, 1);
          i--;
        }
      }
      delimiter = ' ';
    } else {
      attribute.resize(1);
      attribute[0] = std::stod(s);
      return true;
    }

    if(s[0]==' ')
      s.erase(0, 1);

    if(s[s.size()-1]==' ')
      s.erase(s.size()-1, 1);

    size_t size = std::count(s.begin(), s.end(), delimiter) + 1;
    attribute.resize(size);

    for (int i = 0; i < size; i++) {
      while (s.substr(0, 1) == " ")
        s = s.substr(1, s.size()-1);
      
      std::string token = s.substr(0, s.find(delimiter));
      attribute[i] = std::stod(token);
      s.erase(0, s.find(delimiter) + 1);
    }

    return true;
  }

  template <size_t n>
  bool getAttributeIfExists(const std::string &attName, Vec<n> &attribute) const {
    if(!node_) return false;
    auto result = node_->Attribute(attName);
    if (!result)
      return false;

    std::string s = *result;
    char delimiter;
    if (s.find(',') != std::string::npos)
      delimiter = ',';
    else if(s.find(' ') != std::string::npos) {
      for(size_t i=0; i < s.size()-1; i++) {
        if(s.substr(i, 2) == "  ") {
          s.erase(i, 1);
          i--;
        }
      }
      delimiter = ' ';
    }
    if(s[0]==' ')
      s.erase(0, 1);

    if(s[s.size()-1]==' ')
      s.erase(s.size()-1, 1);

    size_t size = std::count(s.begin(), s.end(), delimiter) + 1;
    RSFATAL_IF(size!=n, "Dimension mismatch. " << "File: " << getFileName()+ ", Attribute: " + getFullTree() + attName +
    ", at row:" << std::to_string(node_->Row()) << " col:" << std::to_string(node_->Column()) + " should have a dimension of " + std::to_string(n))

    for (int i = 0; i < size; i++) {
      while (s.substr(0, 1) == " ")
        s = s.substr(1, s.size()-1);
      std::string token = s.substr(0, s.find(delimiter));
      attribute[i] = std::stod(token);
      s.erase(0, s.find(delimiter) + 1);
    }
    return true;
  }

  bool getAttributeIfExists(const std::string &attName, int &attribute) const {
    if(!node_) return false;
    return bool(node_->Attribute(attName, &attribute));
  }

  bool getAttributeIfExists(const std::string &attName, double &attribute) const {
    if(!node_) return false;
    return bool(node_->Attribute(attName, &attribute));
  }

  bool getAttributeIfExists(const std::string &attName, std::string &attribute) const {
    if(!node_) return false;
    const std::string *result = node_->Attribute(attName);
    if (result)
      attribute = *result;
    return bool(result);
  }

  bool getAttributeIfExists(const std::string &attName, unsigned long &attribute) const {
    if(!node_) return false;
    auto result = node_->Attribute(attName);
    if (result)
      attribute = std::stoul(*result);
    return bool(result);
  }

  bool getAttributeIfExists(const std::string &attName, unsigned long long &attribute) const {
    if(!node_) return false;
    auto result = node_->Attribute(attName);
    if (result) attribute = std::stoul(*result);
    return bool(result);
  }

  bool getAttributeIfExists(const std::string &attName, ObjectType &attribute) const {
    if(!node_) return false;
    auto result = node_->Attribute(attName);
    if (result)
      attribute = stringToObjectType(*result);
    return bool(result);
  }

  template <typename T>
  bool getAttributeIfExists(const std::string& arg, const std::string& arg2, T &attribute) {
    if (getAttributeIfExists(arg, attribute)) {
      return true;
    } else {
      return getAttributeIfExists(arg2, attribute);
    }
  }

  template <typename T>
  bool getAttributeIfExists(const std::string& arg, const std::string& arg2, const std::string& arg3, T &attribute) {
    if (getAttributeIfExists(arg, attribute)) {
      return true;
    } else {
      return getAttributeIfExists(arg2, arg3, attribute);
    }
  }

  [[nodiscard]] std::string getFullTree() const {
    std::string tree;

    for (auto &ele : nodeTree_)
      tree += ele + "/";

    return tree;
  }

  [[nodiscard]] const std::string &getFileName() const { return fileName_; }

  [[nodiscard]] const std::string &getNodeName() const { return node_->ValueTStr(); }

private:
  RaiSimTinyXmlWrapper(TiXmlElement *node, const std::string &child,
                       const std::vector<std::string> &parentNodeTree,
                       const std::string &fileName) {
    nodeTree_ = parentNodeTree;
    nodeTree_.push_back(child);
    node_ = node;
    fileName_ = fileName;
  }

  TiXmlDocument doc_;
  TiXmlElement *node_;
  std::string fileName_;
  std::vector<std::string> nodeTree_;
};

} // namespace raisim