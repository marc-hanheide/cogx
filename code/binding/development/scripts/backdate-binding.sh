#!/bin/bash
pushd subarchitectures/category.binding.sa
svn up -r 1208
svn up -r 1266 src/idl/BindingData.properties
svn up src/idl/NewBindingData.idl
svn up build.xml
svn up -r 1475 src/java/org/cognitivesystems/binding/workingmemory/
svn up src/c++/binding/ExampleBindingComponent/

popd

#pushd subarchitectures/comsys.mk3
#svn up -r 1208
#svn up src/idl/ComsysEssentials.properties
#svn up build.xml
#svn up src/java/org/cognitivesystems/comsys/general/
#svn up src/java/org/cognitivesystems/comsys/workingmemory/
#svn up src/java/org/cognitivesystems/comsys/ontology
#mv src/java/org/cognitivesystems/comsys/components/TTS.java_disabled  src/java/org/cognitivesystems/comsys/components/TTS.java

#popd
