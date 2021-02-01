# Update configuration files by replacing win-cl-1.1 by linux in module paths
echo "**** Update module path in configuration file (win-cl-14.1 -> linux)"
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Debug/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Release/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARSample*/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARPipeline*/tests/SolARPipelineTest*/*_conf.xml

echo "**** Install dependencies locally"
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
for file in $(find ./SolARSample* ./SolARPipeline*/tests/SolARPipelineTest* -path "*_conf.xml")
do
   echo "install dependencies for config file: $file"
   remaken bundleXpcf $file -d ./bin/Release -s modules
   remaken bundleXpcf $file -d ./bin/Debug -s modules -c debug
done
