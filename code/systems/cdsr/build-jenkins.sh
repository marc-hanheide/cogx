#!/bin/sh

if [ "$1" ]; then
	workdir="$1"
else
	workdir=`pwd`
fi


rm -rf "$workdir/BUILD" "$workdir/output"
mkdir -pv "$workdir/BUILD"
cd "$workdir/BUILD" \
	&& cmake -C ../cmake-caches/dora-yr3.txt .. \
	&& make -j4 install \
	&& cd "$workdir" \
	&& ant \
	&& echo done
