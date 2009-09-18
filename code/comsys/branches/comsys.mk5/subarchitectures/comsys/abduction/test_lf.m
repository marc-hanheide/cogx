:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module string, map, set.
:- import_module lf, lf_io, formula.
:- import_module world_model.

main(!IO) :-
%	LF = at(of_sort("v1", "object"), and(p("box"), r("colour", and(i(of_sort("c", "colour")), p("something"))))),
	LF = at(of_sort("v1", "object"), p("box")),
	std_lf(LF),
	print(lf_to_string(LF), !IO),
	nl(!IO),

	nl(!IO),

	test_add_lf(LF, world_model.init, _WM, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred std_lf(lf::in) is det.

std_lf(_).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_add_lf(lf::in, world_model::in, world_model::out, io::di, io::uo) is det.

test_add_lf(LF, !WM, !IO) :-
	(if add_lf(LF, !WM)
	then
		print_wm(!.WM, !IO),
		nl(!IO)
	else
		print("epic fail", !IO),
		nl(!IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_wm(world_model::in, io::di, io::uo) is det.

print_wm(WM, !IO) :-
	print("names:\n", !IO),
	map.foldl((pred(Name::in, Sort::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ Name ++ ":" ++ Sort, !IO),
		nl(!IO)
			), WM^names, !IO),

	nl(!IO),

	print("reachability:\n", !IO),
	set.fold((pred({Rel, Id1, Id2}::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id1) ++ "<" ++ Rel ++ ">" ++ string(Id2), !IO),
		nl(!IO)
			), WM^reach, !IO),

	nl(!IO),

	print("props:\n", !IO),
	map.foldl((pred(Id::in, Props::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id) ++ " ... {" ++ string.join_list(", ", set.to_sorted_list(Props)) ++ "}", !IO),
		nl(!IO)
			), WM^props, !IO).
