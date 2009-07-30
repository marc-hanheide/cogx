export SRC_DIR=src/python
export BASE_PLANNER_DIR=$SRC_DIR/standalone/base_planners
export FF_DIR=$BASE_PLANNER_DIR/ContinualAxFF

rm `find . -name "*.pyc"`
rm `find . -name "*~"`
rm -r src/python/standalone/tmp/
pushd $FF_DIR
make clean
pushd
