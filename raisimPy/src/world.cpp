//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

/* The code started from an open-source raisim python wrapper, raisimpy, by
 * Brian Delhaisse <briandelhaisse@gmail.com> */


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>     // automatic conversion between std::vector, std::list, etc to Python list/tuples/dict
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types

#include <iostream>

#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, etc.
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;

void init_world(py::module &m) {

  py::class_<raisim::PolyLine>(m, "Polyline")
      .def("setColor", &raisim::PolyLine::setColor, R"mydelimiter(
	    Set the color of the visual body.
        Args:
            r: red value (1 is max).
            g: green value (1 is max).
            b: blue value (1 is max).
            a: alpha value (1 is max).
	    )mydelimiter", py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"))
      .def("clearPoints", &raisim::PolyLine::clearPoints, R"mydelimiter(
	    Clear points.
	    )mydelimiter")
      .def("addPoint", &raisim::PolyLine::addPoint, R"mydelimiter(
	    Add a point.
        Args:
          point: 3d numpy array
	    )mydelimiter", py::arg("point"));

  py::class_<raisim::ArticulatedSystemVisual>(m, "ArticulatedSystemVisual")
      .def("setColor", &raisim::ArticulatedSystemVisual::setColor, R"mydelimiter(
	    Set the color of the visual body.
        Args:
            r: red value (1 is max).
            g: green value (1 is max).
            b: blue value (1 is max).
            a: alpha value (1 is max).
	    )mydelimiter", py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"))

      .def("setGeneralizedCoordinate", &raisim::ArticulatedSystemVisual::setGeneralizedCoordinate,
           R"mydelimiter(
	    Set the generalized Coordinate.
        Args:
            gc: the generalizedCoordinate.
	    )mydelimiter", py::arg("gc"));

  py::class_<raisim::Visuals>(m, "Visual")
      .def("setPosition", py::overload_cast<const Eigen::Vector3d &>(&raisim::Visuals::setPosition), R"mydelimiter(
	    Set the position of the visual body.
        Args:
            position: the new position of the visual body.
	    )mydelimiter", py::arg("position"))

      .def("setOrientation", py::overload_cast<const Eigen::Vector4d &>(&raisim::Visuals::setOrientation), R"mydelimiter(
	    Set the orientation of the visual body.
        Args:
            orientation: the new orientation of the visual body.
	    )mydelimiter", py::arg("orientation"))

      .def("setColor", &raisim::Visuals::setColor, R"mydelimiter(
	    Set the color of the visual body.
        Args:
            r: red value (1 is max).
            g: green value (1 is max).
            b: blue value (1 is max).
            a: alpha value (1 is max).
	    )mydelimiter", py::arg("r"), py::arg("g"), py::arg("b"), py::arg("a"))

      .def("setBoxSize", &raisim::Visuals::setBoxSize, R"mydelimiter(
	    Set the size of the visual box.
        Args:
            x: length (in x axis).
            y: width (in y axis).
            z: height (in z axis).
	    )mydelimiter", py::arg("x"), py::arg("y"), py::arg("z"))

      .def("setCylinderSize", &raisim::Visuals::setCylinderSize, R"mydelimiter(
	    Set the size of the visual cylinder.
        Args:
            radius: radius.
            height: height.
	    )mydelimiter", py::arg("radius"), py::arg("height"))

      .def("setCapsuleSize", &raisim::Visuals::setCapsuleSize, R"mydelimiter(
	    Set the size of the visual capsule.
        Args:
            radius: radius.
            height: height.
	    )mydelimiter", py::arg("radius"), py::arg("height"))

      .def("setSphereSize", &raisim::Visuals::setSphereSize, R"mydelimiter(
	    Set the size of the visual sphere.
        Args:
            x: radius.
	    )mydelimiter", py::arg("radius"));

  py::class_<raisim::VisualMesh, raisim::Visuals>(m, "VisualMesh")
      .def("updateMesh", &raisim::VisualMesh::updateMesh, R"mydelimiter(
      Set new mesh vertices and colors
        Args:
          vertex array: new vertex array
          color array: new color array
      )mydelimiter", py::arg("vertexArray"), py::arg("colorArray"));

  py::class_<raisim::RaisimServer>(m, "RaisimServer")
      .def(py::init<raisim::World *>())
      .def("launchServer", &raisim::RaisimServer::launchServer)
      .def("killServer", &raisim::RaisimServer::killServer)
      .def("integrateWorldThreadSafe", &raisim::RaisimServer::integrateWorldThreadSafe)
      .def("lockVisualizationServerMutex", &raisim::RaisimServer::lockVisualizationServerMutex)
      .def("unlockVisualizationServerMutex", &raisim::RaisimServer::unlockVisualizationServerMutex)

      .def("startRecordingVideo", &raisim::RaisimServer::startRecordingVideo, R"mydelimiter(
          Start recording RaisimUnity visualization. RaisimUnity only supports video recording in linux.
          Args:
            videoName: name of the video file
	    )mydelimiter",
           py::arg("videoName"))

      .def("stopRecordingVideo", &raisim::RaisimServer::stopRecordingVideo)

      .def("focusOn", [](raisim::RaisimServer& self, raisim::Object& object) {
          // convert the arrays to Vec<3>
          // return the stiff wire instance.
          return self.focusOn(&object);
        },R"mydelimiter(Focus camera on an object
            Args:
              object: name
	    )mydelimiter",
           py::arg("object"))

      .def("addVisualArticulatedSystem",
          &raisim::RaisimServer::addVisualArticulatedSystem,
           R"mydelimiter(
          Add a visual capsule without physics
          Args:
            name: name
            urdfFile: radius
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1)

          Returns:
              pointer to the visual articulated system
	    )mydelimiter",
           py::arg("name"),
           py::arg("urdfFile"),
           py::arg("colorR") = 0,
           py::arg("colorG") = 0,
           py::arg("colorB") = 0,
           py::arg("colorA") = 0,
           py::return_value_policy::reference_internal)

      .def("addVisualCapsule",
           &raisim::RaisimServer::addVisualCapsule,
           R"mydelimiter(
          Add a visual capsule without physics
          Args:
            name: name
            radius: radius
            length: length
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1),
            material = "",
            glow(not supported) = false
            shadow(not supported) = false

          Returns:
              pointer to the visual capsule
	    )mydelimiter",
           py::arg("name"),
           py::arg("radius"),
           py::arg("length"),
           py::arg("colorR") = 1,
           py::arg("colorG") = 1,
           py::arg("colorB") = 1,
           py::arg("colorA") = 1,
           py::arg("material") = "",
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal)

      .def("addVisualCylinder",
           &raisim::RaisimServer::addVisualCylinder,
           R"mydelimiter(
	    Add a visual cylinder without physics
        Args:
            name: name
            radius: radius
            length: length
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1),
            material = "",
            glow(not supported) = false
            shadow(not supported) = false

        Returns:
            pointer to the visual cylinder
	    )mydelimiter",
           py::arg("name"),
           py::arg("radius"),
           py::arg("length"),
           py::arg("colorR") = 1,
           py::arg("colorG") = 1,
           py::arg("colorB") = 1,
           py::arg("colorA") = 1,
           py::arg("material") = "",
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal)

      .def("addVisualArrow",
           &raisim::RaisimServer::addVisualArrow,
           R"mydelimiter(
	    Add a visual cylinder without physics
        Args:
            name: name
            radius: radius
            height: length
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1),
            glow(not supported) = false
            shadow(not supported) = false

        Returns:
            pointer to the visual cylinder
	    )mydelimiter",
           py::arg("name"),
           py::arg("radius"),
           py::arg("height"),
           py::arg("colorR") = 0,
           py::arg("colorG") = 0,
           py::arg("colorB") = 0,
           py::arg("colorA") = -1,
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal)

      .def("addVisualBox",
           &raisim::RaisimServer::addVisualBox,
           R"mydelimiter(
	    Add a visual box without physics
        Args:
            name: name
            x: length (in x axis)
            y: width (in y axis)
            z: height (in z axis)
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1),
            material = "",
            glow(not supported) = false
            shadow(not supported) = false

        Returns:
            pointer to the visual box
	    )mydelimiter",
           py::arg("name"),
           py::arg("x"),
           py::arg("y"),
           py::arg("z"),
           py::arg("colorR") = 1,
           py::arg("colorG") = 1,
           py::arg("colorB") = 1,
           py::arg("colorA") = 1,
           py::arg("material") = "",
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal)

      .def("addVisualSphere",
           &raisim::RaisimServer::addVisualSphere,
           R"mydelimiter(
	    Add a visual sphere without physics
        Args:
            name: name
            radius: radius
            colorR: red value (max:1),
            colorG: green value (max:1),
            colorB: blue value (max:1),
            colorA: alpha value (max:1),
            material = "",
            glow(not supported) = false
            shadow(not supported) = false
        Returns:
            pointer to the visual sphere
	    )mydelimiter",
           py::arg("name"),
           py::arg("radius"),
           py::arg("colorR") = 1,
           py::arg("colorG") = 1,
           py::arg("colorB") = 1,
           py::arg("colorA") = 1,
           py::arg("material") = "",
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal)

      .def("addVisualPolyLine", &raisim::RaisimServer::addVisualPolyLine, R"mydelimiter(
	    Add a visual polyline without physics
        Args:
            name: name

        Returns:
            pointer to the visual polyline
	    )mydelimiter", py::arg("name"), py::return_value_policy::reference_internal)

      .def("addVisualMesh", [](raisim::RaisimServer &self, const std::string & name, const std::string & file,
                               double scale, double R, double G, double B, double A, bool glow, bool shadow)
      {
        raisim::Vec<3> sc; sc.setConstant(scale);
        return self.addVisualMesh(name, file, sc, R, G, B, A, glow, shadow);
        }, R"mydelimiter(
      Add a visual mesh without physics
        Args:
            name: name of the visual object
            file: location of the mesh file
            scale: scale of the visual mesh
            colorR: red color value
            colorG: green color value
            colorB: blue color value
            colorA: alpha color value
            glow(not supported): if glow
            shadow(not supported): if casts shadow

        Returns:
            pointer to the visual mesh
      )mydelimiter",
           py::arg("name"),
           py::arg("file"),
           py::arg("scale") = 1,
           py::arg("colorR") = 0,
           py::arg("colorG") = 0,
           py::arg("colorB") = 0,
           py::arg("colorA") = 1,
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal
      )

      .def("addVisualMesh", [](raisim::RaisimServer &self, const std::string & name, const std::vector<float>& vertex,
                               const std::vector<uint8_t>& color, const std::vector<int32_t>& index,
                               double R, double G, double B, double A, bool glow, bool shadow)
           {
             return self.addVisualMesh(name, vertex, color, index, R, G, B, A, glow, shadow);
           }, R"mydelimiter(
      Add a visual mesh without physics
        Args:
            name: name of the visual object
            vertex: vertex array which contains the positions of the vertices
            color: color array which contains the colors of the vertices
            index: index array which contains the vertex index triplets that form a triangle
            colorR: red color value
            colorG: green color value
            colorB: blue color value
            colorA: alpha color value
            glow(not supported): if glow
            shadow(not supported): if casts shadow

        Returns:
            pointer to the visual mesh
      )mydelimiter",
           py::arg("name"),
           py::arg("vertex"),
           py::arg("color"),
           py::arg("index"),
           py::arg("colorR") = 0,
           py::arg("colorG") = 0,
           py::arg("colorB") = 0,
           py::arg("colorA") = 1,
           py::arg("glow") = false,
           py::arg("shadow") = false,
           py::return_value_policy::reference_internal
      );

  /*********/
  /* World */
  /*********/
  py::class_<raisim::World>(m,
                            "World",
                            "Raisim world.",
                            py::dynamic_attr()) // enable dynamic attributes for C++ class in Python
      .def(py::init<>(), "Initialize the World.")
      .def(py::init<const std::string &>(), "Initialize the World from the given config file.", py::arg("configFile"))

      .def_static("setLicenseFile", &raisim::World::setActivationKey, R"mydelimiter(
        Set the license file path.

        Args:
            path: Path to the license file.
        )mydelimiter", py::arg("licenseFile"))

      .def("setTimeStep", &raisim::World::setTimeStep, R"mydelimiter(
	    Set the given time step `dt` in the simulator.

	    Args:
	        dt (float): time step to be set in the simulator.
	    )mydelimiter",
           py::arg("dt"))

      .def("getTimeStep", &raisim::World::getTimeStep, R"mydelimiter(
	    Get the current time step that has been set in the simulator.

	    Returns:
	        float: time step.
	    )mydelimiter")

      .def("addSphere", &raisim::World::addSphere, R"mydelimiter(
	    Add dynamically a sphere into the world.

	    Args:
	        radius (float): radius of the sphere.
	        mass (float): mass of the sphere.
	        material (str): material to be applied to the sphere.
	        collision_group (unsigned long): collision group.
	        collision_mask (unsigned long): collision mask.

	    Returns:
	        Sphere: the sphere instance.
	    )mydelimiter",
           py::arg("radius"), py::arg("mass"), py::arg("material") = "default", py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addBox", &raisim::World::addBox, R"mydelimiter(
	    Add dynamically a box into the world.

	    Args:
	        x (float): length along the x axis.
	        y (float): length along the y axis.
	        z (float): length along the z axis.
	        mass (float): mass of the box.
	        material (str): material to be applied to the box.
	        collision_group (unsigned long): collision group.
	        collision_mask (unsigned long): collision mask.

	    Returns:
	        Box: the box instance.
	    )mydelimiter",
           py::arg("x"), py::arg("y"), py::arg("z"), py::arg("mass"), py::arg("material") = "default",
           py::arg("collision_group") = 1, py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addCylinder", &raisim::World::addCylinder, R"mydelimiter(
	    Add dynamically a cylinder into the world.

	    Args:
	        radius (float): radius of the cylinder.
	        height (float): height of the cylinder.
	        mass (float): mass of the cylinder.
	        material (str): material to be applied to the cylinder.
	        collision_group (unsigned long): collision group.
	        collision_mask (unsigned long): collision mask.

	    Returns:
	        Cylinder: the cylinder instance.
	    )mydelimiter",
           py::arg("radius"), py::arg("height"), py::arg("mass"), py::arg("material") = "default",
           py::arg("collision_group") = 1, py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addCapsule", &raisim::World::addCapsule, R"mydelimiter(
	    Add dynamically a capsule into the world.

	    Args:
	        radius (float): radius of the capsule.
	        height (float): height of the capsule.
	        mass (float): mass of the capsule.
	        material (str): material to be applied to the capsule.
	        collision_group (unsigned long): collision group.
	        collision_mask (unsigned long): collision mask.

	    Returns:
	        Capsule: the capsule instance.
	    )mydelimiter",
           py::arg("radius"), py::arg("height"), py::arg("mass"), py::arg("material") = "default",
           py::arg("collision_group") = 1, py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addGround", &raisim::World::addGround, R"mydelimiter(
	    Add dynamically a ground into the world.

	    Args:
	        height (float): height of the ground.
	        material (str): material to be applied to the ground.
	        collision_mask (unsigned long): collision mask.

	    Returns:
	        Ground: the ground instance.
	    )mydelimiter",
           py::arg("height") = 0., py::arg("material") = "default", py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addHeightMap",
           py::overload_cast<size_t, size_t, double, double, double, double, const std::vector<double> &,
                             const std::string &, CollisionGroup, CollisionGroup>(&raisim::World::addHeightMap),
           R"mydelimiter(
	    Add a heightmap into the world.

	    Args:
	        x_samples (int): the number of samples in x.
	        y_samples (int): the number of samples in y.
            x_scale (float): the scale in the x direction.
            y_scale (float): the scale in the y direction.
            x_center (float): the x center of the heightmap in the world.
            y_center (float): the y center of the heightmap in the world.
            heights (list[float]): list of desired heights.
            material (str): material.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        HeightMap: the heightmap instance.
	    )mydelimiter",
           py::arg("x_samples"),
           py::arg("y_samples"),
           py::arg("x_scale"),
           py::arg("y_scale"),
           py::arg("x_center"),
           py::arg("y_center"),
           py::arg("heights"),
           py::arg("material") = "default",
           py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addHeightMap", py::overload_cast<const std::string &, double, double, const std::string &,
                                             CollisionGroup, CollisionGroup>(&raisim::World::addHeightMap), R"mydelimiter(
	    Add a heightmap into the world.

	    Args:
	        filename (str): raisim heightmap filename.
            x_center (float): the x center of the heightmap in the world.
            y_center (float): the y center of the heightmap in the world.
            material (str): material.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        HeightMap: the heightmap instance.
	    )mydelimiter",
           py::arg("filename"), py::arg("x_center"), py::arg("y_center"), py::arg("material") = "default",
           py::arg("collision_group") = 1, py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addHeightMap", py::overload_cast<const std::string &,
                                             double,
                                             double,
                                             double,
                                             double,
                                             double,
                                             double,
                                             const std::string &,
                                             CollisionGroup,
                                             CollisionGroup>(&raisim::World::addHeightMap), R"mydelimiter(
	    Add a heightmap into the world.

	    Args:
	        filename (str): filename to the PNG.
	        x_center (float): the x center of the heightmap in the world.
            y_center (float): the y center of the heightmap in the world.
            x_size (float): the size in the x direction.
            y_size (float): the size in the y direction.
	        height_scale (float): the height scale.
	        height_offset (float): the height offset.
            material (str): material.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        HeightMap: the heightmap instance.
	    )mydelimiter",
           py::arg("filename"), py::arg("x_center"), py::arg("y_center"), py::arg("x_size"), py::arg("y_size"),
           py::arg("height_scale"), py::arg("height_offset"), py::arg("material") = "default",
           py::arg("collision_group") = 1, py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addHeightMap",
           py::overload_cast<double, double, raisim::TerrainProperties &, const std::string &,
                             CollisionGroup, CollisionGroup>(&raisim::World::addHeightMap),
           R"mydelimiter(
	    Add a heightmap into the world.

	    Args:
            x_center (float): the x center of the heightmap in the world.
            y_center (float): the y center of the heightmap in the world.
            terrain_properties (TerrainProperties): the terrain properties.
            material (str): material.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        HeightMap: the heightmap instance.
	    )mydelimiter",
           py::arg("x_center"),
           py::arg("y_center"),
           py::arg("terrain_properties"),
           py::arg("material") = "default",
           py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addArticulatedSystem",
           py::overload_cast<const std::string &, const std::string &,
                             const std::vector<std::string> &, CollisionGroup, CollisionGroup,
                             ArticulatedSystemOption>(&raisim::World::addArticulatedSystem),
           R"mydelimiter(
	    Add an articulated system in the world.

	    Args:
            urdf_path (str): path to the URDF file.
            res_path (str): path to the resource directory. Leave it empty ('') if it is the urdf file directory.
            joint_order (list[str]): joint order.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.
            options (ArticulatedSystemOption): articulated system options.

	    Returns:
	        ArticulatedSystem: the articulated system instance.
	    )mydelimiter",
           py::arg("urdf_path"),
           py::arg("res_path") = "",
           py::arg("joint_order") = std::vector<std::string>(),
           py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::arg("options") = raisim::ArticulatedSystemOption(),
           py::return_value_policy::reference_internal)

      .def("addCompound",
           [](raisim::World &self,
              const std::vector<raisim::Compound::CompoundObjectChild> &children,
              double mass,
              py::array_t<double> center_of_mass,
              py::array_t<double> inertia,
              CollisionGroup group = 1,
              CollisionGroup mask = CollisionGroup(-1)) {
             // convert np.array to Mat<3,3>
             Mat<3, 3> I = convert_np_to_mat<3, 3>(inertia);
             Vec<3> com = convert_np_to_vec<3>(center_of_mass);

             // return compound object
             return self.addCompound(children, mass, com, I, group, mask);
           },
           R"mydelimiter(
	    Add a compound body in the world.

	    Args:
            children (list[CompoundObjectChild]): list of child object instance.
            mass (float): mass of the compound object.
            center_of_mass(np.array[float[3]]): center of mass of the compound
            inertia (np.array[float[3,3]]): inertia matrix of the object.
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        Compound: the compound body instance.
	    )mydelimiter",
           py::arg("children"),
           py::arg("mass") = "",
           py::arg("center_of_mass"),
           py::arg("inertia"),
           py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addMesh",
           [](raisim::World &self, const std::string &file_name,
              double mass, py::array_t<double> inertia, py::array_t<double> com,
              double scale = 1., const std::string &material = "default", CollisionGroup group = 1,
              CollisionGroup mask = CollisionGroup(-1)) {
             // convert np.array to raisim matrices
             Mat<3, 3> I;
             I = convert_np_to_mat<3, 3>(inertia);
             Vec<3> COM;
             COM = convert_np_to_vec<3>(com);

             // return compound object
             return self.addMesh(file_name, mass, I, COM, scale, material, group, mask);
           },
           R"mydelimiter(
	    Add a mesh in the world.

	    Args:
            file_name (str): full path of the mesh file.
            mass (float): mass of the compound object.
            inertia (np.array[float[3,3]]): inertia matrix of the object.
            com (np.array[float[3,1]]): the location of the center of mass
            material (str): material
            group (CollisionGroup): collision group.
            mask (CollisionGroup): collision mask.

	    Returns:
	        Mesh: the mesh instance.
	    )mydelimiter",
           py::arg("file_name"),
           py::arg("mass"),
           py::arg("inertia"),
           py::arg("com"),
           py::arg("scale") = 1.,
           py::arg("material") = "default",
           py::arg("collision_group") = 1,
           py::arg("collision_mask") = CollisionGroup(-1),
           py::return_value_policy::reference_internal)

      .def("addStiffWire", [](raisim::World &self, raisim::Object &object1, size_t local_idx1,
                              py::array_t<double> pos_body1, raisim::Object &object2, size_t local_idx2,
                              py::array_t<double> pos_body2, double length) {

             // convert the arrays to Vec<3>
             raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
             raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

             // return the stiff wire instance.
             return self.addStiffWire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length);
           }, R"mydelimiter(
	    Add a stiff wire constraint between two bodies in the world.

	    Args:
            object1 (Object): first object/body instance.
	        local_idx1 (int): local index of the first object/body.
	        pos_body1 (np.array[float[3]]): position of the constraint on the first body.
            object2 (Object): second object/body instance.
	        local_idx2 (int): local index of the second object/body.
	        pos_body2 (np.array[float[3]]): position of the constraint on the second body.
            length (float): length of the wire constraint.

	    Returns:
	        StiffWire: the stiff wire constraint instance.
	    )mydelimiter",
           py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
           py::arg("pos_body2"), py::arg("length"),
           py::return_value_policy::reference_internal)

      .def("addCustomWire", [](raisim::World &self, raisim::Object &object1, size_t local_idx1,
                              py::array_t<double> pos_body1, raisim::Object &object2, size_t local_idx2,
                              py::array_t<double> pos_body2, double length) {

             // convert the arrays to Vec<3>
             raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
             raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

             // return the stiff wire instance.
             return self.addCustomWire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length);
           }, R"mydelimiter(
	    Add a custom wire constraint between two bodies in the world.

	    Args:
            object1 (Object): first object/body instance.
	        local_idx1 (int): local index of the first object/body.
	        pos_body1 (np.array[float[3]]): position of the constraint on the first body.
            object2 (Object): second object/body instance.
	        local_idx2 (int): local index of the second object/body.
	        pos_body2 (np.array[float[3]]): position of the constraint on the second body.
            length (float): length of the wire constraint.

	    Returns:
	        StiffWire: the stiff wire constraint instance.
	    )mydelimiter",
           py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
           py::arg("pos_body2"), py::arg("length"),
           py::return_value_policy::reference_internal)

      .def("addCompliantWire", [](raisim::World &self, raisim::Object &object1, size_t local_idx1,
                                  py::array_t<double> pos_body1, raisim::Object &object2, size_t local_idx2,
                                  py::array_t<double> pos_body2, double length, double stiffness) {

             // convert the arrays to Vec<3>
             raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
             raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

             // return the compliant wire instance.
             return self.addCompliantWire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length, stiffness);
           }, R"mydelimiter(
	    Add a compliant wire constraint between two bodies in the world.

	    Args:
            object1 (Object): first object/body instance.
	        local_idx1 (int): local index of the first object/body.
	        pos_body1 (np.array[float[3]]): position of the constraint on the first body.
            object2 (Object): second object/body instance.
	        local_idx2 (int): local index of the second object/body.
	        pos_body2 (np.array[float[3]]): position of the constraint on the second body.
            length (float): length of the wire constraint.
            stiffness (float): stiffness of the wire.

	    Returns:
	        CompliantWire: the compliant wire constraint instance.
	    )mydelimiter",
           py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
           py::arg("pos_body2"), py::arg("length"), py::arg("stiffness"),
           py::return_value_policy::reference_internal)

      .def("getObject", py::overload_cast<const std::string &>(&raisim::World::getObject), R"mydelimiter(
	    Get the specified object instance from its unique name.

	    Args:
            name (str): unique name of the object instance we want to get.

	    Returns:
	        Object, None: the specified object instance. None, if it didn't find the object.
	    )mydelimiter",
           py::arg("name"),
           py::return_value_policy::reference_internal)

      .def("getObject", py::overload_cast<std::size_t>(&raisim::World::getObject), R"mydelimiter(
	    Get the specified object instance from its unique name.

	    Args:
            name (str): unique name of the object instance we want to get.

	    Returns:
	        Object, None: the specified object instance. None, if it didn't find the object.
	    )mydelimiter",
           py::arg("world_index"),
           py::return_value_policy::reference_internal)

      .def("getConstraint", &raisim::World::getConstraint, R"mydelimiter(
	    Get the specified constraint instance from its unique name.

	    Args:
            name (str): unique name of the constraint instance we want to get.

	    Returns:
	        Constraints, None: the specified constraint instance. None, if it didn't find the constraint.
	    )mydelimiter",
           py::arg("name"),
           py::return_value_policy::reference_internal)

      .def("getWire", &raisim::World::getWire, R"mydelimiter(
	    Get the specified wire instance from its unique name.

	    Args:
            name (str): unique name of the wire instance we want to get.

	    Returns:
	        Constraints: the specified wire instance. None, if it didn't find the wire.
	    )mydelimiter",
           py::arg("name"),
           py::return_value_policy::reference_internal)

      .def("getConfigurationNumber", &raisim::World::getConfigurationNumber, R"mydelimiter(
	    Get the number of elements that are in the world. The returned number is updated everytime that we add or
	    remove an object from the world.

	    Returns:
	        int: the number of objects in the world.
	    )mydelimiter")

      .def("removeObject", py::overload_cast<raisim::Object *>(&raisim::World::removeObject), R"mydelimiter(
	    Remove dynamically an object from the world.

	    Args:
	        obj (Object): the object to be removed from the world.
	    )mydelimiter",
           py::arg("obj"))

      .def("removeObject", py::overload_cast<raisim::LengthConstraint *>(&raisim::World::removeObject), R"mydelimiter(
	    Remove dynamically a wire from the world.

	    Args:
	        wire: the wire to be removed from the world.
	    )mydelimiter",
           py::arg("wire"))

      .def("integrate",
           &raisim::World::integrate,
           "this function is simply calling both `integrate1()` and `integrate2()` one-by-one.")

      .def("integrate1", &raisim::World::integrate1, R"mydelimiter(
        It performs:
        1. deletion contacts from previous time step
        2. collision detection
        3. register contacts to each body
        4. calls `preContactSolverUpdate1()` of each object
        )mydelimiter")

      .def("integrate2", &raisim::World::integrate2, R"mydelimiter(
        It performs
        1. calls `preContactSolverUpdate2()` of each body
        2. run collision solver
        3. calls `integrate` method of each object
        )mydelimiter")


          // TODO: improve the doc for the below method
      .def("getContactProblem", &raisim::World::getContactProblem, R"mydelimiter(
        Return the list of contacts.
        )mydelimiter")

      .def("getObjList", &raisim::World::getObjList, R"mydelimiter(
        Return the list of object instances that are in the world.

        Returns:
            list[Object]: list of object instances.
        )mydelimiter")

      .def("updateMaterialProp", &raisim::World::updateMaterialProp, R"mydelimiter(
        Update material property.

        Args:
            prop (MaterialManager): material manager property instance.
        )mydelimiter",
           py::arg("prop"))

      .def("setMaterialPairProp",
           py::overload_cast<const std::string &, const std::string &, double, double, double>(&raisim::World::setMaterialPairProp),
           R"mydelimiter(
        Set material pair properties.

        Args:
            material1 (str): first material.
            material2 (str): second material.
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            threshold (float): restitution threshold.
        )mydelimiter",
           py::arg("material1"),
           py::arg("material2"),
           py::arg("friction"),
           py::arg("restitution"),
           py::arg("threshold"))

      .def("setMaterialPairProp",
           py::overload_cast<const std::string &, const std::string &, double, double, double, double, double>(&raisim::World::setMaterialPairProp),
           R"mydelimiter(
        Set material pair properties.

        Args:
            material1 (str): first material.
            material2 (str): second material.
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            restitution threshold (float): restitution threshold.
            static friction (float): coefficient of static friction
            static friction velocity threshold (float): If the relative velocity of two contact points is bigger than this value, then the dynamic coefficient of friction is applied. Otherwise, the coefficient of friction is interpolated between the static and dynamic one proportional to the relative velocity.

                )mydelimiter",
                py::arg("material1"),
                py::arg("material2"),
                py::arg("friction"),
                py::arg("restitution"),
                py::arg("restitution_threshold"),
                py::arg("static_friction"),
                py::arg("static_friction_velocity_threshold"))

                .def("setDefaultMaterial", py::overload_cast<double, double, double, double, double>(&raisim::World::setDefaultMaterial), R"mydelimiter(
        Set the default material.

        Args:
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            restitution threshold (float): restitution threshold.
            static friction (float): coefficient of static friction.
            static friction velocity threshold (float): If the relative velocity of two contact points is bigger than this value, then the dynamic coefficient of friction is applied. Otherwise, the coefficient of friction is interpolated between the static and dynamic one proportional to the relative velocity.
        )mydelimiter",
        py::arg("friction"), py::arg("restitution"), py::arg("threshold"), py::arg("static_friction"), py::arg("static_friction_velocity_threshold"))

        .def("setDefaultMaterial", py::overload_cast<double, double, double>(&raisim::World::setDefaultMaterial), R"mydelimiter(
        Set the default material.

        Args:
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            restitution threshold (float): restitution threshold.
        )mydelimiter",
        py::arg("friction"), py::arg("restitution"), py::arg("restitution threshold"))


      .def("getGravity", [](raisim::World &world) {
        Vec<3> gravity = world.getGravity();
        return convert_vec_to_np(gravity);
      }, R"mydelimiter(
        Get the gravity vector from the world.

        Returns:
            np.array[float[3]]: gravity vector.
        )mydelimiter")

      .def("setGravity", [](raisim::World &world, py::array_t<double> array) {
        raisim::Vec<3> gravity = convert_np_to_vec<3>(array);
        world.setGravity(gravity);
      }, R"mydelimiter(
        Set the gravity vector in the world.

        Args:
            np.array[float[3]]: gravity vector.
        )mydelimiter", py::arg("gravity"))

      .def("setERP",
           &raisim::World::setERP,
           "Set the error reduction parameter (ERP).",
           py::arg("erp"),
           py::arg("erp2") = 0)

      .def("set_contact_solver_parameters",
           &raisim::World::setContactSolverParam,
           R"mydelimiter(
        Set contact solver parameters.

        Args:
            alpha_init (float): alpha init.
            alpha_min (float): alpha minimum.
            alpha_decay (float): alpha decay.
            max_iters (float): maximum number of iterations.
            threshold (float): threshold.
        )mydelimiter",
           py::arg("alpha_init"),
           py::arg("alpha_min"),
           py::arg("alpha_decay"),
           py::arg("max_iters"),
           py::arg("threshold"))

      .def("getWorldTime", &raisim::World::getWorldTime, R"mydelimiter(
        Return the total integrated time (which is updated at every `integrate2()`` call).

        Returns:
            float: world time.
        )mydelimiter")

      .def("setWorldTime", &raisim::World::setWorldTime, R"mydelimiter(
        Set the world time.

        Args:
            time (float): world time
        )mydelimiter", py::arg("time"))

      .def("getContactSolver", py::overload_cast<>(&raisim::World::getContactSolver), R"mydelimiter(
        Return the bisection contact solver used.

        Returns:
            BisectionContactSolver: contact solver.
        )mydelimiter")

      .def("rayTest", &raisim::World::rayTest, R"mydelimiter(
        Returns the internal reference of the ray collision list it
        contains the geoms (position, normal, object world/local id) and the number of intersections

        Args:
            start The start position of the ray.
            direction The direction of the ray.
            length The length of the ray.
            closestOnly Only stores the first collision.
            objectId ignores collisions with an object with the same objectId and localId
            localId ignores collisions with an object with the same objectId and localId
            collisionMask Collision mask to filter collisions. By default, it records collisions with all collision groups.
            @return A reference to the internal container which contains all ray collisions.
        )mydelimiter", py::arg("start"), py::arg("direction"), py::arg("length"), py::arg("closestOnly") = true, py::arg("objectId") = size_t(-10), py::arg("localId") = size_t(-10), py::arg("collisionMask") = raisim::CollisionGroup(-1))
      ;

  py::class_<RayCollisionItem>(m, "RayCollisionItem")
      .def("getPosition", &RayCollisionItem::getPosition)
      .def("getObject", &RayCollisionItem::getObject);

  py::class_<RayCollisionList>(m, "RayCollisionList")
      .def("size", &RayCollisionList::size)
      .def("len", &RayCollisionList::size)
      .def("at", [](RayCollisionList& self, int idx){ return self[idx]; });
}