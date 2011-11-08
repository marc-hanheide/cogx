#!/bin/sh

if [ "$1" ]; then
	workdir="$1"
else
	workdir=`pwd`
fi

mkdir -pv "$workdir/BUILD"
cd "$workdir/BUILD" \
	&& (../tools/scripts/cmake-apply . ../cmake-caches/dora-yr3.txt || cmake -C ../cmake-caches/dora-yr3.txt ..) \
	&& make -j4 install \
	&& cd "$workdir" \
	&& ant \
	&& echo done
