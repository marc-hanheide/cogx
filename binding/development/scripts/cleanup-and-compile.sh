find . -name autogen -exec rm -rf {} \;
rm -rf BUILD/subarchitectures/
rm -rf BUILD/tools/
rm -rf output/include/
rm output/lib/*
rm -rf output/classes/
rm build.include
rm build.exclude
ant all;
cd BUILD;
make install -j2
