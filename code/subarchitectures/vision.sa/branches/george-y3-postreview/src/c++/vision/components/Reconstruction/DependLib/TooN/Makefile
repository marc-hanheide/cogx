#Standard boilerplate
prefix = /usr/local
exec_prefix = ${prefix}
mandir = ${prefix}/man
includedir = ${prefix}/include


hdr = $(DESTDIR)/$(includedir)/TooN

all:

install:
	mkdir -p $(hdr)
	cp *.h *.hh $(hdr)
	cp -r optimization $(hdr)/

clean:
	rm -rf html

docs:
	doxygen doc/Doxyfile
