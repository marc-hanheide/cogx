
This document describes how to use the Robotics Service Bus to send/receive data between Java, LISP and C++ using the code written at Birmingham

GSH 7 Dec 2011

Prepare - read all of this section before doing anything else!
-------

Install the Robotics Service Bus from https://code.cor-lab.de/projects/rsb

Follow the installation instructions on the website - NOTE:

You should be able to install the .deb packages for RSC and RSB which are available via the link lower down on the front page

RSB requires

 * Spread 4.1.0 from the source at http://www.spread.org/
 * protoc 2.4 - to compile protocol buffers

For the Java version it needs
 * libprotobuf-java (version 2.4 or higher)

For the LISP version it needs

quicklisp 

*** unfinished ***


For all languages:

Create a file called rsb.conf in your .config directory which should contain these lines:

[transport.spread]
enabled = 1
host    = localhost
port    = 4803
[transport.inprocess]
enabled = 0

------------------------------------------------------------------

Compile
-------

cd {workspace}/cdsr

Java:

protoc --java_out=gen-src/java src/cdsr_messages.proto 

export RSB_CLASSPATH=/usr/share/java/protobuf.jar:/usr/local/share/java/rsb.jar
mkdir -p output/classes
javac -classpath $RSB_CLASSPATH -d output/classes/ src/java/cdsr/*/* gen-src/java/cdsr/rsb/*

C++:

mkdir build
cd build
ccmake ..

Change the CMAKE_INSTALL_PREFIX to ../output, then (c)onfigure and (g)enerate

make
make install

--------------------------------------------------------------------

Run
---

start a Spread daemon:

cd /usr/local/sbin
./spread -c /usr/local/etc/spread.conf -n localhost

In a second shell:

Check it is OK:

spuser -s 4803

cd {workspace}/cdsr
cd output/bin
./cdsr_test

In a third shell:

cd {workspace}/cdsr
java -classpath $RSB_CLASSPATH:output/classes cdsr.rsb.CdsrHub classroom data/classroom-1.cdsr


