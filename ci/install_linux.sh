sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo apt-get -qq update

sudo apt-get -qq --yes --force-yes install libeigen3-dev
sudo apt-get -qq --yes --force-yes install libccd-dev

# Octomap
# Note: Octomap has a number of build defects on macOS. We're trying to push a
# fix through to the main repo, but while we wait, we'll reference a forked
# repo with the fixes. We are pulling the same commit here so that linux and
# mac builds are consistent.
git clone https://github.com/SeanCurtis-TRI/octomap
cd octomap
git checkout PR_correct_warnings
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
