export PYTHONPATH=output/python:${PYTHONPATH}
export CLASSPATH=output/classes:./subarchitectures/*/lib/*.jar:./subarchitectures/binder/lib/jgraphx.jar:${CLASSPATH}
export DYLD_LIBRARY_PATH=output/lib:${DYLD_LIBRARY_PATH}
export LD_LIBRARY_PATH=output/lib:${LD_LIBRARY_PATH}