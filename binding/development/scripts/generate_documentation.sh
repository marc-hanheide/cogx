find . -name "*_tagfile" -exec rm {} \;

echo subarch documentation

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
cd subarchitectures/comsys.mk4/doc/
./docgen.sh
cd ../../../
cd subarchitectures/spatial.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/central.mechanisms/doc/
./docgen.sh
cd ../../../
cd subarchitectures/benchmark.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/coma.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/nav.sa/doc/
./docgen.sh
cd ../../../
cd subarchitectures/manipulation.sa/doc/
./docgen.sh
cd ../../../

echo specifications

cd specifications/template/
doxygen
cd ../../
cd specifications/main/
doxygen
cd ../../
cd specifications/scenario/
doxygen
cd ../../
cd specifications/planning/
doxygen
cd ../../
cd specifications/binding/
doxygen
cd ../../
cd specifications/vision/
doxygen
cd ../../
cd specifications/comsys/
doxygen
cd ../../
cd specifications/coma_reasoning/
doxygen
cd ../../
cd specifications/navigation/
doxygen
cd ../../
cd specifications/motivation/
doxygen
cd ../../
cd specifications/laser_features_extractor/
doxygen
cd ../../

tar -czvf html_doc.tar.gz */*/doc/*_tagfile specifications/*/*_tagfile */*/doc/*/html/ specifications/*/html/ */*/README* */*/*/README*

echo html_doc.tar.gz created
