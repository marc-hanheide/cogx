export http_proxy=http://webcache:3128
export https_proxy=http://webcache:3128

dname=`dirname $0`

cd $dname

echo "make sure you have this in a file in /etc/sudoers.d:"
echo %jenkins	ALL=NOPASSWD:SETENV:/usr/bin/make -C $dname dora

sudo /usr/bin/make -C $dname dora
