
This document describes how to use the Robotics Service Bus to send/receive data between Java, LISP and C++ using the code written at Birmingham

GSH 28 Nov 2011

Prepare
-------

Install the Robotics Service Bus from https://code.cor-lab.de/projects/rsb

which requires

 * Spread 4.1.0 from the source at http://www.spread.org/ (GSH had problems with Spread 3.17)
 * protoc 2.4 - to compile protocol buffers

For the Java version it needs
 * libprotobuf-java (version 2.4 or higher)

For the LISP version it needs

For the C++ version it needs

----

Follow the installation instructions on the website - download the source or get a package (not tried but I'm told it works and includes Spread 4.0)



------------------------------------------------------------------

Compile
-------

protoc --java_out=gen-src/java --cpp_out=gen-src/cpp src/cdsr_messages.proto 

export RSB_CLASSPATH=/usr/share/java/protobuf.jar:/usr/local/share/java/rsb.jar
mkdir -p output/classes
javac -classpath $RSB_CLASSPATH -d output/classes/ src/java/cdsr/*/* gen-src/java/cdsr/rsb/*


--------------------------------------------------------------------

Run
---

edit spread.conf (in /usr/local/etc/spread.conf if you built Spread from source)

** todo **

start a Spread daemon:

spread -c /usr/local/etc/spread.conf -n localhost

In a second shell:

Check it is OK:

spuser -s 4803

java -classpath $RSB_CLASSPATH:output/classes cdsr.rsb.SpatialRegionsServer

In a third shell:

java -classpath $RSB_CLASSPATH:output/classes cdsr.rsb.SpatialRegionsClient data/classroom-1.cdsr


