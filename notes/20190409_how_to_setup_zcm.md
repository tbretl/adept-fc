# Installation

Set java path by adding this line to the end of `~/.bashrc`:
```
export JAVA_HOME=/usr/lib/jvm/jdk-8-oracle-arm32-vfp-hflt
```

## [Install sodium](https://libsodium.gitbook.io/doc/installation)

Download the latest tarball from [here](https://download.libsodium.org/libsodium/releases). You can also do this from the command line as:
```
wget https://download.libsodium.org/libsodium/releases/LATEST.tar.gz
```
Assuming this file is called `LATEST.tar.gz`, then do this:
```
tar zxvf LATEST.tar.gz 
cd libsodium-stable 
./configure 
make && make check 
sudo make install 
```
It will take awhile.

## [Install zmq](http://zeromq.org/)

Install dependencies:
```
sudo apt-get install make autoconf automake libtool -y 
```
Either [download the latest tarball](http://zeromq.org/intro:get-the-software) and do:
```
tar zxvf zeromq-4.1.4.tar.gz
cd zeromq-4.1.4
```
Or git clone the stable release repo:
```
git clone https://github.com/zeromq/zeromq4-x.git
cd zeromq4-x
./autogen.sh
```
Then, build from source
```
./configure
make
sudo make install
sudo ldconfig
```
Finally, install more dependencies (this will likely only install `python-zmq` as new):
```
sudo apt-get install python-pip python-all-dev python-zmq
```

## [Install zcm](https://github.com/ZeroCM/zcm):

Clone the repo:
```
git clone https://github.com/ZeroCM/zcm.git
cd zcm
```
Build (FIXME - probably want to add python support with `--use-python`, but this will require a cython install, which means you'll likely want to start using `virtualenv`... so holding off for now):
```
./waf configure --use-ipc --use-zmq
./waf build
sudo ./waf install 
```


# Test (FIXME)

build command: 
```
g++ -o interface interface.cpp ../message_types/actuators_t.hpp ../message_types/sensor_data_t.hpp -lzcm
g++ -o hitl hitl_autopilot_main.cpp ../message_types/sensor_data_t.hpp ../message_types/actuators_t.hpp -lzcm
```

generate header files for message types in cpp: 
```
zcm-gen --cpp msg_t.zcm 
```


