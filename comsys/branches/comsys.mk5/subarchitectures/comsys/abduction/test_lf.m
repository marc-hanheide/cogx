:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module solutions.
:- import_module string, map, set, list, pair.
:- import_module lf, lf_io, formula.
:- import_module world_model.

main(!IO) :-
	test_lf( r("colour", p("red")), world_model.init, _, !IO),
	test_lf( r("colour", i(of_sort("v", "thing"))), world_model.init, _, !IO),
	test_lf( r("colour", and(r("tint", p("weird")), p("red"))), world_model.init, _, !IO),
	test_lf( at(of_sort("v1", "object"), i(of_sort("v1", "object"))), world_model.init, _, !IO),
	test_lf( at(of_sort("v1", "object"), i(of_sort("v2", "object"))), world_model.init, _, !IO),

	test_lf( at(of_sort("v1", "object"), p("box")), world_model.init, _, !IO),

	test_lf( at(of_sort("v1", "object"), and(p("box"), r("colour", and(i(of_sort("c", "colour")), p("something"))))), world_model.init, _, !IO),

	test_lf( r("colour", and(r("tint", p("weird")), p("red"))), world_model.init, WM1, !IO),
	test_lf( r("colour", i(of_sort("c", "colour"))), WM1, WM3, !IO),
%	test_lf( r("colour", i(of_sort("d", "colour"))), WM2, WM3, !IO),  % this should make it inconsistent
	test_lf( at(of_sort("c", "colour"), r("tint", i(of_sort("t", "tint")))), WM3, WM4, !IO),

	print_simplified_models(WM4, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred std_lf(lf::in) is det.

std_lf(_).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_lf(lf::in, world_model::in, world_model::out, io::di, io::uo) is det.

test_lf(LF, !WM, !IO) :-
	print(lf_to_string(LF), !IO),
	nl(!IO),
	nl(!IO),
	test_add_lf(LF, !WM, !IO),
	print("--------------------------------------------------------------\n", !IO).

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
	NamesStrs = list.map((func(Name-Sort) = S :- S = Name ++ ":" ++ Sort), map.to_assoc_list(WM^names)),
	print("  " ++ string.join_list("\n  ", NamesStrs) ++ "\n", !IO),
	nl(!IO),

	print("reachability:\n", !IO),
	set.fold((pred({Rel, Id1, Id2}::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id1) ++ " <" ++ Rel ++ "> " ++ string(Id2), !IO),
		nl(!IO)
			), WM^reach, !IO),

	nl(!IO),

	print("props:\n", !IO),
	map.foldl((pred(Id::in, Props::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id) ++ " ... {" ++ string.join_list(", ", set.to_sorted_list(Props)) ++ "}", !IO),
		nl(!IO)
			), WM^props, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_simplified_models(world_model::in, io::di, io::uo) is det.

print_simplified_models(WM, !IO) :-
	SWMs = solutions((pred(SWM::out) is nondet :-
		simplify_pred(WM, SWM)
			)),

	print("Simplified models: [\n", !IO),

	list.foldl((pred(SWM::in, !.IO::di, !:IO::uo) is det :-
		print_wm(SWM, !IO),
		print("- - - - - - - - - - - - - -\n", !IO)
			), SWMs, !IO),

	print("].\n", !IO).
