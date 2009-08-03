export CLASSPATH=lib/Ice.jar:lib/ant-ice.jar:lib/cast.jar:output/classes
export DYLD_LIBRARY_PATH=/opt/Ice-3.3.1/lib
./local-cast-server.sh &
cast-client subarchitectures/binder/config/binder_onlygui.cast