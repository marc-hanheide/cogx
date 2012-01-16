
This document describes how to use the Robotics Service Bus to send/receive data between Java, LISP and C++ using the code written at Birmingham

GSH 8 Dec 2011; updated 10 Jan 2012

# Prepare - read all of this section before doing anything else!
-------

Install the Robotics Service Bus from https://code.cor-lab.de/projects/rsb

Follow the installation instructions on the website - NOTE:

You should be able to install the .deb packages for RSC and RSB which are available via the link lower down on the front page

But building from source is OK. For Lisp you need the source.

RSB requires

 * Spread 4.1.0 from the source at http://www.spread.org/
 * libprotobuf-dev, protobuf-compiler, libprotobuf-java - 2.3 or higher, from Ubuntu packages or source




## For all languages
-------

Create a file called rsb.conf in your .config directory which should contain these lines:

[transport.spread]
enabled = 1
host    = localhost
port    = 4803
[transport.inprocess]
enabled = 0

## For C++
-------

Use $prefix = /usr/local

Note for RSB C++ Core, the webpage uses RSC not rsc - it should be in lower case:

cd core/build && cmake .. -DCMAKE_INSTALL_PREFIX=$prefix -DRSC_DIR=$prefix/share/rsc && make && make install

## For Java 
-------

Create a build.properties file containing:

env.prefix = /usr/local
pbuf.protoc = /usr/bin/protoc
pbuf.protopath = /usr/local/share/rsbprotocol
pbuf.lib = /usr/share/java/protobuf.jar
spread.daemon = /usr/local/sbin/spread

Then use ant as described.

## For LISP 
-------

### OSX 
-------

Using 2 terminals:

**Terminal 1**:

sudo port install bzr 
sudo port install sbcl +threads

mkdir $HOME/patches
cd patches
curl -O https://code.cor-lab.org/attachments/download/174/asdf-system-connections.patch
curl -O https://code.cor-lab.org/attachments/download/175/iterate.patch

mkdir -p $HOME/quicklisp/local-projects
cd $HOME/quicklisp/local-projects

svn co https://code.cor-lab.org/svn/rsb/trunk/cl/cl-rsb/ 
svn co https://code.cor-lab.org/svn/rsb/trunk/cl/cl-dynamic-classes
svn co https://code.cor-lab.org/svn/rsb/trunk/cl/cl-protobuf
svn co https://code.cor-lab.org/svn/rsb/trunk/cl/cl-spread
bzr branch lp:cl-hooks
curl -O http://beta.quicklisp.org/quicklisp.lisp

ln -s ~/.config/rsb.conf

sbcl (the following commands are entered in sbcl, do not exit when switching terminals)

(load "quicklisp.lisp")
(quicklisp-quickstart:install)
(ql:quickload "yacc")
(ql:quickload :asdf-system-connections)


**Switch to Terminal 2**:

cd $HOME
patch -p0 --strip=1 < patches/asdf-system-connections.patch 

cp $prefix/share/rsbprotocol/rsb/protocol/Notification.proto $HOME/quicklisp/local-projects/cl-rsb/data/rsb/protocol/

**Switch to Terminal 1**:

(ql:quickload :cl-spread)

**Switch to Terminal 2**:

cd $HOME
patch -p0 --strip=1 < patches/iterate.patch

**Switch to Terminal 1**:

(ql:quickload :cl-rsb)


If you get an error like:

Component :SPLIT-SEQUENCE not found, required by #<SYSTEM "cl-protobuf">

then just retry as it seems to work the second time.


### Graham's original instructions (Ubuntu I presume) 
-------

Tested using Steel Bank Common Lisp

Download quicklisp.lisp from http://www.quicklisp.org/

Then in sbcl (omit proxy or change, as required) - to install as ~/quicklisp

(load "quicklisp.lisp")
(quicklisp-quickstart:install) or (quicklisp-quickstart:install :proxy "http://webcache.cs.bham.ac.uk:3128/")

Copy the contents of rsb/cl to quicklisp/local-projects

Back in sbcl:

(ql:quickload "yacc")
(ql:quickload :asdf-system-connections)

Now do a bug fix (done by the patch file below, so you don't have to do it by hand)

remove :force t in line 47 of
$HOME/quicklisp/dists/quicklisp/software/asdf-system-connections-20101006-darcs/dev/asdf-system-connections.lisp

And a hack or 2:

cp $prefix/share/rsbprotocol/rsb/protocol/Notification.proto $HOME/quicklisp/local-projectys/cl-rsb/data/rsb/protocol/

Copy the rsb.conf file from your .config directory (see below for creation of rsb.conf)

In sbcl:

(ql:quickload :cl-rsb)
(ql:quickload :cl-spread)

Now for some bug fixes:

Assuming the patch files are in  ~/patches and quicklisp is in ~/quicklisp

cd ~/patches
patch -p0 -b -i iterate.patch
patch -p0 -b -i asdf-system-connections.patch

Lisp is now good to go!


# Compile
-------


cd {workspace}/cdsr

## Java:
-------

bash compile.sh

## C++:
-------

mkdir build
cd build
ccmake -DCMAKE_INSTALL_PREFIX=../output ..

Press (c)onfigure and (g)enerate

make
make install


# Run
-------

start a Spread daemon:

cd /usr/local/sbin
./spread -c /usr/local/etc/spread.conf -n localhost

In a second shell:

Check it is OK:

spuser -s 4803

## C++

cd {workspace}/cdsr

### OSX

DYLD_LIBRARY_PATH=output/lib ./output/bin/cdsr_test 

### Linux

LD_LIBRARY_PATH=output/lib ./output/bin/cdsr_test 

## Java

In a third shell:

cd {workspace}/cdsr
export RSB_CLASSPATH=/usr/share/java/protobuf.jar:/usr/local/share/java/rsb.jar
java -classpath $RSB_CLASSPATH:output/classes cdsr.rsb.CdsrHub classroom data/classroom-1.cdsr data/classroom-1-out.cdsr

Afterwards, to convert to text files for LISP:

java -classpath output/classes cdsr.marshall.ProblemSetConverter data/classroom-1-out.cdsr classroom

## Lisp

Replace $USER in the following:

### OSX



cd $HOME/quicklisp/local-projects/
sbcl
(load "quicklisp.lisp")
(load "../setup.lisp")
(ql:quickload :asdf-system-connections)
(ql:quickload :cl-spread)
(ql:quickload :cl-rsb)

Then to run example code:

(load "cl-rsb/examples/listener.lisp")

or

(load "cl-rsb/examples/informer.lisp")

### Linux

(load "quicklisp.lisp")
(load "/home/$USER/quicklisp/setup.lisp")
(ql:quickload :asdf-system-connections)
(ql:quickload :cl-spread)
(ql:quickload :cl-rsb)

Then to run example code:

(load "listener.lisp")

or 

(load "informer.lisp")

To exit SBCL:

(quit)
