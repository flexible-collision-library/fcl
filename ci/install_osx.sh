brew update > /dev/null

brew install git
brew install cmake
#brew install eigen
brew extract --version=3.4.0 eigen homebrew/local
brew install homebrew/local/eigen@3.4.0
brew pin eigen@3.4.0
brew install libccd

# Octomap
# Note: Octomap has a number of build defects on macOS. We're trying to push a
# fix through to the main repo, but while we wait, we'll reference a forked
# repo with the fixes.
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
