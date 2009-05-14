#!/bin/bash

mkdir -p release

cp -a doc release
cp -a src release
cp -a INSTALL release
cp -a TESTING release
cp -a OVERVIEW release
cp -a build.xml release

rm -rf release/.svn
rm -rf release/*/.svn
rm -rf release/*/*/.svn
rm -rf release/*/*/*/.svn
rm -rf release/*/*/*/*/.svn
rm -rf release/*/*/*/*/*/.svn
rm -rf release/*/*/*/*/*/*/.svn
rm -rf release/*/*/*/*/*/*/*/.svn
rm -rf release/*/*/*/*/*/*/*/*/.svn