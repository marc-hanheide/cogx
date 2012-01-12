#!/bin/bash
mkdir -p gen-src/java
protoc --java_out=gen-src/java src/cdsr_messages.proto 

export RSB_CLASSPATH=/usr/share/java/protobuf.jar:/usr/local/share/java/rsb.jar
mkdir -p output/classes
javac -classpath $RSB_CLASSPATH -d output/classes/ src/java/cdsr/*/* gen-src/java/cdsr/rsb/* rsb-src/java/cdsr/rsb/*
