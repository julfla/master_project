master_project
==============

# Getting started
## Installing requirements
1. Common requirements : 

  ```bash
  sudo apt-get install libboost-all-dev libcppunit-dev
  ```
2. Requirements for cross-compilation (convertion off fichiers .skp)

  ```bash
  sudo add-apt-repository ppa:ubuntu-wine/ppa -y && sudo apt-get update
  sudo apt-get install wine mingw32
  ```
3. PointCloud library

  ```bash
  sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
  sudo apt-get update
  sudo apt-get install libpcl-all`
  ```
4. OpenGL

  ```bash
  sudo apt-get install libglm-dev lib-glew-dev libglfw-dev
  ```
