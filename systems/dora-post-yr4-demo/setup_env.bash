# Source this in your .bashrc

COGX_ROOT=$(dirname $(readlink -f $BASH_SOURCE))

export CLASSPATH="/usr/local/share/java/cast.jar:$COGX_ROOT/output/jar/*:$COGX_ROOT/output/classes:/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar:/usr/share/java/log4j-1.2.jar:/usr/share/java/xstream-1.3.1.jar:$CLASSPATH"

export LD_LIBRARY_PATH="/usr/local/lib/cast:$COGX_ROOT/output/lib:/usr/local/lib/cure:/usr/local/cuda/lib:/opt/MATLAB/MATLAB_Compiler_Runtime/v78/runtime/glnx86:$COGX_ROOT/tools/v4r/lib:/usr/lib/PhysX/v2.8.3:$LD_LIBRARY_PATH"
