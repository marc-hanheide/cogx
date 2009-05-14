
cd tools/balt/doc/
./docgen.sh
cd ../../../
cd tools/cast/doc/
./docgen.sh
cd ../../../
cd tools/comedian-architecture/doc/
./docgen.sh
cd ../../../
cd subarchitectures/binding.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/vision/doc/
./docgen.sh
cd ../../../
cd subarchitectures/planning.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/comsys.mk4/doc
./docgen.sh
cd ../../../
cd specifications/template/
doxygen
cd ../../
cd specifications/scenario/
doxygen
cd ../../
cd specifications/planning/
doxygen
cd ../../

tar -czvf html_doc.tar.gz */*/doc/*/html/* */*/html/*