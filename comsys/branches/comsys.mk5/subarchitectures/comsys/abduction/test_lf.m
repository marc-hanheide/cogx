:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module solutions, require.
:- import_module string, map, set, list, pair, unit.
:- import_module utils.
:- import_module lf, lf_io, formula, term_io, parser, formula_ops.
:- import_module model, ontology.

main(!IO) :-
	io.command_line_arguments(CmdArgs, !IO),
	(if
		CmdArgs = [FileName]
	then

		Ont = ssp(from_list([
				"object"-"entity",
				"house"-"object",
				"car"-"object"
				])),
		RT = ss(from_list(["colour", "tint"])),

		read_file_as_lines(FileName, Strs0, !IO),
		strip_ignore_comments(Strs0, Strs),

		some [!WM] (
			!:WM = model.init,

			list.foldl2((pred(Line::in, !.WM::in, !:WM::out, !.IO::di, !:IO::uo) is det :-
				(if Line = "CLEAR"
				then
					print("-------------------------------------------------------------\n", !IO),
					!:WM = model.init
				else
				(if Line = "PRINT"
				then
					print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),
					print_wm(!.WM, !IO)
				else
				(if Line = "REDUCE"
				then
					print("= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n", !IO),
					print_reduced_model(!.WM, !IO)
				else
				(if Line = "LFS"
				then
					print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),
					set.fold((pred(LF::in, !.IO::di, !:IO::uo) is det :-
						print(" *  " ++ lf_to_string(LF) ++ "\n", !IO)
							), lfs(!.WM), !IO),
					print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),
					(if reduced(Ont, !.WM) = RWM
					then
						set.fold((pred(LF::in, !.IO::di, !:IO::uo) is det :-
							print("(*) " ++ lf_to_string(LF) ++ "\n", !IO)
								), lfs(RWM), !IO)
					else
						print("  (model irreducible)\n", !IO)
					)
				else
				(if string.first_char(Line, '?', RestLine)
				then
					LF = s2lf(strip(RestLine)),
					print("??  ", !IO),
					print(lf_to_string(LF), !IO),
					print(" ... ", !IO),
					(if satisfies(Ont, !.WM, LF) then Sat = "t" else Sat = "f"),
					(if RM = reduced(Ont, !.WM)
					then (if satisfies(Ont, RM, LF) then SatR = "t" else SatR = "f")
					else SatR = "-"
					),
					print(Sat ++ SatR ++ "\n", !IO)
				else
					LF = s2lf(Line),
					print("+   ", !IO),
					print(lf_to_string(LF), !IO),
					print(" ... ", !IO),
					(if
						%add_lf(Ont, RT, !.WM, LF, !:WM)
						add_lf(Ont, RT, model.init, LF, XM),
						union(Ont, RT, !.WM, XM, !:WM)
					then
						(if satisfies(Ont, !.WM, LF) then Sat = "t" else Sat = "f"),
						(if RM = reduced(Ont, !.WM)
						then (if satisfies(Ont, RM, LF) then SatR = "t" else SatR = "f")
						else SatR = "-"
						),
						print("ok " ++ Sat ++ SatR ++ "\n", !IO)
					else
						print("FAIL, ignoring\n", !IO)
					)
				)))))
					), Strs, !WM, !IO)
		)
	else
		io.progname("?", ProgName, !IO),
		format(stderr_stream, "Usage: %s TEST_FILE\n", [s(ProgName)], !IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type set_strings
	--->	ss(set(string)).

:- instance accessibility(set_strings, string) where [
	(exclusive(ss(Ss), S) :- member(S, Ss))
].

:- type set_string_pairs
	--->	ssp(set(pair(string, string))).

:- instance isa_ontology(set_string_pairs, string) where [
	(direct_isa(ssp(Ss), X, Y) :- member(X-Y, Ss))
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func s2lf(string) = lf.

s2lf(S) = LF :-
	read_term_from_string_with_op_table(init_wabd_op_table, "", S, _, ReadResult),
	(if ReadResult = term(_, T)
	then LF = det_ground_atomic_formula_to_lf(det_formula_to_ground_formula(det_term_to_atomic_formula(T)))
	else error("parsing error in s2lf in \"" ++ S ++ "\"")
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred std_lf(lf::in) is det.

std_lf(_).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
/*
:- pred test_slf(string::in, model::in, model::out, io::di, io::uo) is det.

test_slf(S, !WM, !IO) :-
	test_lf(s2lf(S), !WM, !IO).
*/
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

/*
:- pred test_lf(lf::in, model::in, model::out, io::di, io::uo) is det.

test_lf(LF, !WM, !IO) :-
	print(lf_to_string(LF), !IO),
	nl(!IO),
	nl(!IO),
	test_add_lf(LF, !WM, !IO),
	print("--------------------------------------------------------------\n", !IO).
*/

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

/*
:- pred test_add_lf(lf::in, model::in, model::out, io::di, io::uo) is det.

test_add_lf(LF, !WM, !IO) :-
	(if add_lf(unit, unit, !.WM, LF, !:WM)
	then
		print_wm(!.WM, !IO),
		nl(!IO)
	else
		print("epic fail", !IO),
		nl(!IO)
	).
*/

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_wm(model::in, io::di, io::uo) is det.

print_wm(WM, !IO) :-
	print("names:", !IO),
	NamesStrs = list.map((func(Name-Sort) = S :- S = Name ++ ":" ++ Sort), map.to_assoc_list(WM^worlds)),
	print(string.join_list("\n  ", [""|NamesStrs]) ++ "\n\n", !IO),

	print("accessibility:\n", !IO),
	set.fold((pred({Rel, Id1, Id2}::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id1) ++ " <" ++ Rel ++ "> " ++ string(Id2), !IO),
		nl(!IO)
			), WM^access, !IO),

	nl(!IO),

	print("props:\n", !IO),
	map.foldl((pred(Id::in, Props::in, !.IO::di, !:IO::uo) is det :-
		print("  " ++ string(Id) ++ " ... {" ++ string.join_list(", ", set.to_sorted_list(Props)) ++ "}", !IO),
		nl(!IO)
			), WM^props, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func reduced_models(model) = set(model).

reduced_models(WM) = RWMs :-
	RWMs = solutions_set((pred(RWM::out) is nondet :-
		RWM = reduced(unit, WM)
			)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_reduced_model(model::in, io::di, io::uo) is det.

print_reduced_model(WM, !IO) :-
	(if RWM = reduced(unit, WM)
	then print_wm(RWM, !IO)
	else print("model functionally irreducible.\n", !IO)
	).
