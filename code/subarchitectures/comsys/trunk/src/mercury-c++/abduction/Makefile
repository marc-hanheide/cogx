.PHONY: main
main: server cli

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -#

.PHONY: lib
lib:
	make -C lib

.PHONY: server
server: lib
	make -C server
	[ -e abducer-server-bin ] || ln -s server/abducer-server-bin

.PHONY: cli
cli: lib
	make -C cli
	[ -e check-rules-file ] || ln -s cli/check-rules-file

.PHONY: tests
tests: lib
	make -C tests

.PHONY: clean
clean:
	make -C lib clean
	make -C server clean
	make -C tests clean
	rm -f abducer-server-bin check-rules-file
