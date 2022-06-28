0. CMake
--------

Download CMake: https://cmake.org/

1. Dependenciees
----------------

1.1. Eigen
----------

1. Pull from Eigen repository: https://gitlab.com/libeigen/eigen.git in a folder, called [MY_EIGEN_DIR] in the following


1.2. LibIGL
-----------

1. Pull from LibIGL repository: https://github.com/libigl/libigl.git in a folder, called [MY_LIBIGL_DIR] in the following
2. Create a folder [MY_LIBIGL_DIR]/external
3. Pull from LibIGL/tetgen repository: https://github.com/libigl/tetgen.git in the folder [MY_LIBIGL_DIR]/external/tetgen
4. Pull from LibIGL/triangle repository: https://github.com/libigl/triangle.git in the folder [MY_LIBIGL_DIR]/external/triangle

1.2.1. Small correction
-----------------------

There is some unidentified problem with this TetGen release. To solve it:

copy [MY_PHYSIM_DIR]/tetgen.cxx
paste [MY_LIBIGL_DIR]/external/tetgen/tetgen.cxx

Where [MY_PHYSIM_DIR] is the folder where you pulled PhySim

1.2.2. Building TetGen
----------------------

1. Run CMake-GUI
2. Select the source code folder [MY_LIBIGL_DIR]/external/tetgen
3. Select the build folder to be created [MY_LIBIGL_DIR]/external/tetgen/build
4. Click configure.
5. Select your preferred generator (e.g. VS 14 2015)
6. Select your preferred run platform (generally x64)
7. Click generate.
8. Build the generated project in both debug and release mode

1.2.3. Building Triangle
------------------------

1. Run CMake-GUI
2. Select the source code folder [MY_LIBIGL_DIR]/external/triangle
3. Select the build folder to be created [MY_LIBIGL_DIR]/external/triangle/build
4. Follow 1.2.2. 4-8

1.3 TinyObj
-----------

1. Run CMake-GUI
2. Select the source code folder PhySim/external/tinyobj
3. Select the build folder to be created PhySim/external/tinyobj/build
4. Click configure.
5. Select your preferred generator (e.g. VS 14 2015)
6. Select your preferred run platform (generally x64)
7. Click generate.
8. Build the generated project in both debug and release mode

1.4 Nanoflann
-------------

1. Pull from Nanoflann repository: https://github.com/jlblancoc/nanoflann.git in a folder, called [MY_NANOFLANN_DIR] in the following

1.4 Cereal
----------

1. Pull from Cereal repository: https://github.com/USCiLab/cereal.git in a folder, called [MY_CEREAL_DIR] in the following

2. Building PhySim
------------------

1. Pull from PhySim repository: https://gitlab.com/jesus.prod/PhySim.git in a folder, called [MY_PHYSIM_DIR] in the following
2. Make a copy of CMakeLocalTemplate.txt and rename it as CMakeLocal.txt
3. Edit the following lines in CMakeLocal.txt to point to your folders

set(EIGEN_DIR "[MY_EIGEN_DIR]")
set(LIBIGL_DIR "[MY_LIBIGL_DIR]")
set(CEREAL_DIR "[MY_CEREAL_DIR]")
set(NANOFLANN_DIR "[MY_NANOFLANN_DIR]")

4. Run CMake-GUI
5. Select the source code folder [MY_PHYSIM_DIR]
6. Select the build folder to be created [MY_LIBIGL_DIR]/build
7. Follow 1.3, 4-8

3. Using PhySim library
-----------------------

1. In your project, add PhySim source to the include directories

[MY_PHYSIM_DIR]/include

2. In your project, add PhySim libraries to the linker libraries. 

[MY_PHYSIM_DIR]/build/Debug/PhySim.lib, for the Debug build
[MY_PHYSIM_DIR]/build/Release/PhySim.lib, for the Release build