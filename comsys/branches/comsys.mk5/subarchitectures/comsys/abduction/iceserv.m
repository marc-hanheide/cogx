:- module iceserv.

:- interface.
:- import_module io, int.

:- import_module iceserv_lib.

:- pred main(io::di, io::uo) is det.

:- pred aserv_main(int::out, io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module string, list.

:- pragma foreign_decl("C", "#include \"aserv.h\"").

main(!IO) :-
	aserv_main(RetVal, !IO),
	format("Returned with status %d.\n", [i(RetVal)], !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_proc("C", aserv_main(RetVal::out, IO0::di, IO::uo),
		[may_call_mercury, promise_pure],
"
	RetVal = aserv_main();
	IO = IO0;
").
