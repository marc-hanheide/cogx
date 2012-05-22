export http_proxy=http://webcache:3128/
export https_proxy=http://webcache:3128/

dname=`dirname $0`

export http_proxy
export https_proxy
export SVNUSER=cogx
env

/usr/bin/make -C $dname clean

if [ "$1" == "" ]
   /usr/bin/make -C $dname george SVNUSER=cogx
else
   /usr/bin/make -C $dname $1 SVNUSER=cogx
fi
