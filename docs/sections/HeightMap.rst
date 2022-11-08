#############################
Height Map
#############################

Height map is a grid of points which are triangulated to form a surface.
Since it is axis-aligned, collision checking is very efficient and it is a recommended way to create a terrain.

You can get the following two properties from a heightMap instance

* height: :code:`getHeight()`
* normal vector: :code:`getNormal()`

Example
=============================

.. toctree::
   :maxdepth: 2

   HeightMap_example_png
   HeightMap_example_raw_values
   HeightMap_example_terrain_generator
   HeightMap_example_txt
