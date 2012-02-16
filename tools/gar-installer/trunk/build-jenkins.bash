export http_proxy=http://webcache:3128/
export https_proxy=http://webcache:3128/

dname=`dirname $0`

jobname="$1"

if [ -z "$jobname" ]; then
   jobname="dora"
fi

export http_proxy
export https_proxy
export SVNUSER=cogx
env

/usr/bin/make -C $dname clean
/usr/bin/make -C $dname $jobname SVNUSER=cogx
