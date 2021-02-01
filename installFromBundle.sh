@echo off
if ! [ $(id -u) = 0 ]; then
    echo "The script need to be run as root." >&2
    exit 1
fi

# Set system library path for samples shared libraries
if [ -e /etc/ld.so.conf.d/solar_sample.conf ]; then
   echo "File /etc/ld.so.conf.d/solar_sample.conf already exists"   
   
else
   echo "Create file /etc/ld.so.conf.d/solar_sample.conf"
   sudo touch /etc/ld.so.conf.d/solar_sample.conf
fi

if ! grep -Fxq "$PWD/bin/Debug" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Debug path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Debug" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Debug path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi

if ! grep -Fxq "$PWD/bin/Debug/modules" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Debug/modules path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Debug/modules" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Debug/modules path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi

if ! grep -Fxq "$PWD/bin/Release" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Release path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Release" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Release path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi

if ! grep -Fxq "$PWD/bin/Release/modules" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Release/modules path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Release/modules" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Release/modules path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi
