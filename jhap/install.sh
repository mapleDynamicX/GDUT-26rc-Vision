#/usr/bin/sh
echo "Installing JHCap camera library."
echo "Shenzhen Jinghang Technologies"
echo "(C) Copyright 2018"
echo "......\n"
CURDIR=$(pwd)
ln -f -s $CURDIR/JHCap2/libJHCap.so /usr/lib/libJHCap.so
ln -f -s $CURDIR/JHCap2/libJHCap.a /usr/lib/libJHCap.a

cp -f $CURDIR/driver/jhusb.conf /etc/jhusb.conf
cp -f $CURDIR/driver/88-jhusb.rules /etc/udev/rules.d

echo "JHCap library nstalled successfully."
