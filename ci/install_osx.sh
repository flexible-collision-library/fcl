brew update > /dev/null

brew install git
brew install cmake
brew install eigen
brew install libccd

# Octomap
git clone https://github.com/OctoMap/octomap
cd octomap
git checkout tags/v1.8.0
mkdir build
cd build
cmake ..
make
sudo make install
