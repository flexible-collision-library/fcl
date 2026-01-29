sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo apt-get -qq update

sudo apt-get -qq --yes --force-yes install libeigen3-dev
sudo apt-get -qq --yes --force-yes install libccd-dev

# Octomap
git clone https://github.com/OctoMap/octomap
cd octomap
git checkout tags/v1.10.0
mkdir build
cd build
cmake \
  -DBUILD_DYNAMICETD3D_SUBPROJECT=off \
  -DBUILD_OCTOVIS_SUBPROJECT=off \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.10 \
  ..
make
sudo make install
