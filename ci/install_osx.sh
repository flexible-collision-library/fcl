# Note: this file is vestigial until issue 661 is resolved and macos CI builds
# are restored.

brew update > /dev/null

brew install git
brew install cmake
brew install eigen
brew install libccd

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
