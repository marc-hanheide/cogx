#!/bin/sh
mkdir -pv "$WORKSPACE/BUILD"
cd "$WORKSPACE/BUILD" \
	&& (../tools/scripts/cmake-apply . ../cmake-caches/dora-yr3.txt || cmake -C ../cmake-caches/dora-yr3.txt ..) \
	&& make -j4 install \
	&& cd "$WORKSPACE" \
	&& ant \
	&& echo done
