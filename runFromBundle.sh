for dataConfFile in "../../data/*_conf.xml" ; do
   sed -i 's/win-cl-14.1/linux-gcc/' $dataConfFile
done

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD:$PWD/modules $1 $2


