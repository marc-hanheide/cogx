echo building planner executables
export SRC_DIR=src/python
export BASE_PLANNER_DIR=$SRC_DIR/standalone/base_planners
export FF_DIR=$BASE_PLANNER_DIR/ContinualAxFF
echo create python stubs from slice files
pushd $SRC_DIR
./build_slice.py
pushd
echo compile base planners
pushd $FF_DIR
make
pushd
echo clean up
# nothing currently
