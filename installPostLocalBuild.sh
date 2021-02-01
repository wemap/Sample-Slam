if ! [ $(id -u) = 0 ]; then
    echo "The script need to be run as root." >&2
    exit 1
fi

# Update configuration files by replacing win-cl-1.1 by linux in module paths
echo "**** Update module path in configuration file (win-cl-14.1 -> linux)"
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Debug/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Release/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARSample*/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARPipeline*/tests/SolARPipelineTest_SLAM_*/*_conf.xml

# Set system library path for samples shared libraries
echo "**** Add modules to system library path"
if [ -e /etc/ld.so.conf.d/solar_sample.conf ]; then
   echo "File /etc/ld.so.conf.d/solar_sample.conf already exists"   
   
else
   echo "Create file /etc/ld.so.conf.d/solar_sample.conf"
   sudo touch /etc/ld.so.conf.d/solar_sample.conf
fi

for file in $(find ./SolARSample* ./SolARPipeline* -path "*_conf.xml")
do
   for modulePath in $(grep -o "\$REMAKEN_PKG_ROOT.*lib" $file)
   do
      if ! grep -Fxq "$modulePath/x86_64/shared/debug" /etc/ld.so.conf.d/solar_sample.conf
      then
         echo "Add $modulePath/x86_64/shared/debug path to /etc/ld.so.conf.d/solar_sample.conf"
         sudo echo "$modulePath/x86_64/shared/debug" >> /etc/ld.so.conf.d/solar_sample.conf
      else
         echo "Path $modulePath/x86_64/shared/debug path already defined in /etc/ld.so.conf.d/solar_sample.conf"
      fi 
      if ! grep -Fxq "$modulePath/x86_64/shared/release" /etc/ld.so.conf.d/solar_sample.conf
      then
         echo "Add $modulePath/x86_64/shared/release path to /etc/ld.so.conf.d/solar_sample.conf"
         sudo echo "$modulePath/x86_64/shared/release" >> /etc/ld.so.conf.d/solar_sample.conf
      else
         echo "Path $modulePath/x86_64/shared/release path already defined in /etc/ld.so.conf.d/solar_sample.conf"
      fi 
   done
done

echo "**** Add bin folder to system library path"
if ! grep -Fxq "$PWD/bin/Debug" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Debug path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Debug" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Debug path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi
if ! grep -Fxq "$PWD/bin/Release" /etc/ld.so.conf.d/solar_sample.conf
then
   echo "Add $PWD/bin/Release path to /etc/ld.so.conf.d/solar_sample.conf"
   sudo echo "$PWD/bin/Release" >> /etc/ld.so.conf.d/solar_sample.conf
else
   echo "Path $PWD/bin/Release path already defined in /etc/ld.so.conf.d/solar_sample.conf"
fi 

sudo ldconfig   



