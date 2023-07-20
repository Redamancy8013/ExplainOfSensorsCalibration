dnet库安装
打开终端输入：
sudo cp *.h /usr/include
ln -s libdnet.so.4.2.0 libdnet.so
sudo cp libdnet.so* /usr/lib

cd /usr/lib
ls | grep libdnet.so
