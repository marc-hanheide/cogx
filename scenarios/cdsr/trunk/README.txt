
This document describes how to use the Robotics Service Bus to send/receive data between Java, LISP and C++ using the code written at Birmingham

GSH 7 Dec 2011

Prepare - read all of this section before doing anything else!
-------

Install the Robotics Service Bus from https://code.cor-lab.de/projects/rsb

Follow the installation instructions on the website - NOTE:

You should be able to install the .deb packages for RSC and RSB which are available via the link lower down on the front page

But building from source is OK.

RSB requires

 * Spread 4.1.0 from the source at http://www.spread.org/
 * libprotobuf-dev, protobuf-compiler, libprotobuf-java - 2.3 or higher, from Ubuntu packages or source

For C++

Use $prefix = /usr/local

Note for RSB C++ Core, the webpage uses RSC not rsc - it should be in lower case:

cd core/build && cmake .. -DCMAKE_INSTALL_PREFIX=$prefix -DRSC_DIR=$prefix/share/rsc && make && make install

For Java 

Create a build.properties file containing:

pbuf.protoc = /usr/bin/protoc
pbuf.protopath = /usr/local/share/rsbprotocol
pbuf.lib = /usr/share/java/protobuf.jar
spread.daemon = /usr/local/sbin/spread

For LISP 

Tested using Steel Bank Common Lisp

Install quicklisp 

Copy the contents of rsb/cl to quicklisp/local-projects

*** unfinished ***

-------------------------------------------------------------------
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


Lisp:

Quicklisp Bug fix

remove :force t in line 47 of
$HOME/quicklisp/dists/quicklisp/software/asdf-system-connections-20101006-darcs/dev/asdf-system-connections.lisp

(load "quicklisp.lisp")
(load "/home/graham/quicklisp/setup.lisp")
(ql:quickload :asdf-system-connections)
(ql:quickload :cl-spread)
(ql:quickload :cl-rsb)


