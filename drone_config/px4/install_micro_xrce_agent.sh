mkdir -p ~/thirdparty/
cd ~/thirdparty/
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git -b v2.4.3
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/