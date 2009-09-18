:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module solutions, require.
:- import_module string, map, set, list, pair.
:- import_module lf, lf_io, formula, term_io, parser, formula_ops.
:- import_module world_model.

main(!IO) :-
	io.command_line_arguments(CmdArgs, !IO),
	(if
		CmdArgs = [FileName]
	then
		read_file_as_lines(FileName, Strs0, !IO),
		preprocess_file(Strs0, Strs),

		some [!WM] (
			!:WM = world_model.init,

			list.foldl2((pred(Line::in, !.WM::in, !:WM::out, !.IO::di, !:IO::uo) is det :-
				(if Line = "CLEAR"
				then
					print("-------------------------------------------------------------\n", !IO),
					!:WM = world_model.init
				else
					(if Line = "PRINT"
					then
						print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),
						print_wm(!.WM, !IO)
					else
						(if Line = "SIMPLIFY"
						then
							print("= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =\n", !IO),
							print_simplified_models(!.WM, !IO)
						else
							(if Line = "LFS"
							then
								print("> > > > > > > > > > > > > > > > > > > > > > > > > > > > > > >\n", !IO),
								print("  not implemented.\n", !IO)
							else
								LF = s2lf(Line),
								print(lf_to_string(LF), !IO),
								print(" ... ", !IO),
								(if add_lf(LF, !WM)
								then print("ok (" ++ from_int(count(simplified_models(!.WM))) ++")\n", !IO)
								else print("FAIL, ignoring\n", !IO)
								)
							)
						)
					)
				)
					), Strs, !WM, !IO)
		)
	else
		io.progname("?", ProgName, !IO),
		format(stderr_stream, "Usage: %s TEST_FILE\n", [s(ProgName)], !IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred read_file_as_lines(string::in, list(string)::out, io::di, io::uo) is det.

read_file_as_lines(FileName, Lines, !IO) :-
	see(FileName, SeeResult, !IO),
	(
		SeeResult = ok,
		read_file_as_string(ReadResult, !IO),
		(
			ReadResult = ok(S),
			Lines = string.words_separator((pred(C::in) is semidet :- C = '\n'), S)
		;
			ReadResult = error(_, _),
			Lines = []
		),
		seen(!IO)
	;
		SeeResult = error(_),
		Lines = []
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred preprocess_file(list(string)::in, list(string)::out) is det.

preprocess_file(LIn, LOut) :-
	list.filter_map((pred(L0::in, L::out) is semidet :-
		L1 = string.strip(L0),
		L = L1,
		not string.first_char(L, '#', _)
			), LIn, LOut).


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

:- pred test_slf(string::in, world_model::in, world_model::out, io::di, io::uo) is det.

test_slf(S, !WM, !IO) :-
	test_lf(s2lf(S), !WM, !IO).

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
	print("names:", !IO),
	NamesStrs = list.map((func(Name-Sort) = S :- S = Name ++ ":" ++ Sort), map.to_assoc_list(WM^names)),
	print(string.join_list("\n  ", [""|NamesStrs]) ++ "\n\n", !IO),

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

:- func simplified_models(world_model) = set(world_model).

simplified_models(WM) = SWMs :-
	SWMs = solutions_set((pred(SWM::out) is nondet :-
		simplify_pred(WM, SWM)
			)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_simplified_models(world_model::in, io::di, io::uo) is det.

print_simplified_models(WM, !IO) :-
	SWMs = set.to_sorted_list(simplified_models(WM)),
	(if SWMs = []
	then
		print("no simplified models.\n", !IO)
	else
		list.foldl((pred(SWM::in, !.IO::di, !:IO::uo) is det :-
			print_wm(SWM, !IO),
			print("  = = = = = = = = = = = = = =\n", !IO)
				), SWMs, !IO)
	).
