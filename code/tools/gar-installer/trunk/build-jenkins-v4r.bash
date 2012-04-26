export http_proxy=http://webcache:3128/
export https_proxy=http://webcache:3128/

dname=`dirname $0`

export http_proxy
export https_proxy
export SVNUSER=cogx
env

/usr/bin/make -C $dname clean
/usr/bin/make -C $dname v4r SVNUSER=cogx
