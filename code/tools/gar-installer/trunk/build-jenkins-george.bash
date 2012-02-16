export http_proxy=http://webcache:3128/
export https_proxy=http://webcache:3128/

dname=`dirname $0`

export http_proxy
export https_proxy
export SVNUSER=cogx
export CASTV4RPASS=cogxxgoc
env

/usr/bin/make -C $dname clean
/usr/bin/make -C $dname george SVNUSER=cogx
