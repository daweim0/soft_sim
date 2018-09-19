### Description ###
Design, simulate, and analyze meshes consisting of linear and torsional springs.

### Contents ###
* **meshtools** - generate meshes and save/load them to/from xml files (MATLAB)
* **odesim** - simulate a mesh using the open dynamics engine (C++)
* **solvers** - find a mesh configuration that locally minimizes potential energy (MATLAB)

### Dependencies ###

* Open Dynamics Engine (including Drawstuff)
* Eigen 3
* TinyXml 2

### Tested Environments ###

* Ubuntu 14.04

### Setup on Ubuntu 14.04 ###

Install the Eigen 3 and TinyXml 2 C++ libraries:

~~~
sudo apt-get install libeigen3-dev libtinyxml-dev
~~~

Download ode-0.13 from: https://sourceforge.net/projects/opende/files/

Extract (unzip) ode-0.13.tar.bz2

Navigate into the ode-0.13 directory using Terminal and run the commands

~~~
./configure
make
sudo make install
sudo cp -p drawstuff/src/.libs/libdrawstuff.a /usr/local/lib/
sudo cp -p drawstuff/src/libdrawstuff.la /usr/local/lib/
sudo cp -rp include/drawstuff /usr/local/include/
sudo cp -rp drawstuff/textures /usr/local/include/drawstuff/
~~~

Navigate into the `flexible_arm/odesim` directory using Terminal and run the commands

~~~
make
./sim
~~~

If you encounter compiling or linking errors, you may have to update the paths in the `Makefile` (i.e. -I/... and -L/...) and `sim.cpp`.