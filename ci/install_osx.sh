brew update > /dev/null

brew install git
brew install cmake
brew install eigen
brew install libccd

# Octomap
# git clone https://github.com/OctoMap/octomap
# When [] is merged, go back to OctoMap/octomap and the appropriate tag.
git clone https://github.com/SeanCurtis-TRI/octomap
cd octomap
# git checkout tags/v1.10.0
git checkout PR_correct_warnings
mkdir build
cd build
# Note: octomap has some return-type warnings that make AppleClang unhappy.
# FCL CI should not serve as octomap CI.
cmake \
  -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -w" \
  -DBUILD_DYNAMICETD3D_SUBPROJECT=off \
  -DBUILD_OCTOVIS_SUBPROJECT=off \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.10 \
  ..
make
sudo make install
