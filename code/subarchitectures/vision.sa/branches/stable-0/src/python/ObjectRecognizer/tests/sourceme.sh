
path=../../../../../../output/lib
cur=$(pwd)
abs=$(cd $path; pwd)
cd $cur
export LD_LIBRARY_PATH=$abs:$LD_LIBRARY_PATH

