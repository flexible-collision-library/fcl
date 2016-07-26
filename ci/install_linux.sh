sudo add-apt-repository --yes ppa:libccd-debs/ppa
sudo apt-get -qq update

sudo apt-get -qq --yes --force-yes install libeigen3-dev
sudo apt-get -qq --yes --force-yes install libccd-dev

# Octomap
git clone https://github.com/OctoMap/octomap
cd octomap
git checkout tags/v1.8.0
mkdir build
cd build
cmake ..
make
sudo make install
