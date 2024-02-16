//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_RAICOLLISIONMESH_HPP
#define RAISIM_RAICOLLISIONMESH_HPP

#include "SingleBodyObject.hpp"

namespace raisim {

class Mesh final : public SingleBodyObject {

 public:

  explicit Mesh(const std::string &filename, dSpaceID space, double mass, const Mat<3, 3> &inertia, const Vec<3> &COM,
                double scale=1.0);

  explicit Mesh(const std::string &filename, dSpaceID space, double mass, double scale=1.0);

  explicit Mesh(const std::vector<float> &verticies, const std::vector<unsigned int> &idx, dSpaceID space, double mass,
                const Mat<3, 3> &inertia, const Vec<3> &COM, double scale=1.0);

  ~Mesh() final;

  // return mesh file name if it was created with a file. Otherwise, returns an empty string
  const std::string& getMeshFileName() const { return meshFileName_; }

  const std::vector<float>& getVerticies() const { return verticies_; }

  const std::vector<unsigned int>& getIndicies() const { return idx_; }

  double getScale() const { return scale_ ; };

  static void loadObj(const std::string &filename, std::vector<float>& verticies, std::vector<dTriIndex>& idx, double scale=1);

 protected:

  void _init(dSpaceID space,
             double mass,
             const Mat<3, 3> &inertia,
             const Vec<3> &COM);

  std::vector<unsigned int> idx_;
  std::vector<float> verticies_;
  dTriMeshDataID Data_;
  std::string meshFileName_;
  double scale_ = 1.0;

};

} // raisim

#endif //RAISIM_RAICOLLISIONSPHERE_HPP
