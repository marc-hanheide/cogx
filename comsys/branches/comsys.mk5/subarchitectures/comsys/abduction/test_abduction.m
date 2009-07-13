% $Id$

:- module test_abduction.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string, float.
:- import_module abduction, kb, lf, formula, context, costs.

:- import_module parser, term_io, term, varset.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

main(!IO) :-
	io.command_line_arguments(CmdArgs, !IO),
	(if
		CmdArgs = [FileName, Goal, GoalAssumeCost],
		string.to_float(GoalAssumeCost, InitAssumeCost)
	then
		some [!KB] (
			!:KB = kb.init,

			read_file_as_lines(FileName, Strs0, !IO),
			preprocess_file(Strs0, Strs),
			list.foldl((pred(L::in, !.KB::in, !:KB::out) is det :-
				(if string_as_vsmrule(L, R)
				then kb.add_vsmrule(R, !KB)
				else
					(if string_as_vsmprop(L, P)
					then kb.add_vsmprop(P, !KB)
					else error("Can't convert `" ++ L ++ "' to fact or rule.")
					)
				)), Strs, !KB),

			vs(InitMProp, InitVarset) = det_string_to_vsmprop(Goal),

			P0 = proof(vs([[InitMProp-unsolved(const(InitAssumeCost))]], InitVarset), []),

			format("Proving\n  %s\n\nwith\n\n", [s(vsmprop_to_string(vs(InitMProp, InitVarset)))], !IO),

			print_kb(!.KB, !IO),

			nl(!IO),

			DC0 = new_d_ctx,

			Proofs0 = set.to_sorted_list(solutions_set((pred(Cost-P::out) is nondet :-
				prove(P0, P, !.KB),
				Cost = cost(DC0, P)
					))),

			list.sort((pred(CA-_::in, CB-_::in, Comp::out) is det :-
				(if CA < CB then Comp = (<) else (if CA > CB then Comp = (>) else Comp = (=)))
					), Proofs0, Proofs),

			format("Found %d proofs.\n", [i(length(Proofs))], !IO),

			list.foldl((pred(Cost-Proof::in, !.IO::di, !:IO::uo) is det :-
				(if
					Proof = proof(vs([_|_], _), _)
				then
					print("----------------------------------------------------------------------\n", !IO),
					format("Proof cost = %f\n\n", [f(Cost)], !IO),
					print_proof(Proof, !IO),

					nl(!IO)

				else
					true
				)
					), Proofs, !IO)
		)
	else
		io.progname("?", ProgName, !IO),
		format(stderr_stream, "Usage: %s FILE GOAL GOAL_ASSUMPTION_COST\n", [s(ProgName)], !IO)
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

:- pred print_kb(kb::in, io::di, io::uo) is det.

print_kb(KB, !IO) :-
	print("Facts:\n", !IO),
	set.fold((pred(Fact::in, !.IO::di, !:IO::uo) is det :-
		print("  ", !IO),
		print(vsmprop_to_string(Fact), !IO),
		nl(!IO)
			), KB^kb_facts, !IO),

	nl(!IO),

	print("Rules:\n", !IO),
	set.fold((pred(Rule::in, !.IO::di, !:IO::uo) is det :-
			% XXX global context
		print("  ", !IO),
		print(vsmrule_to_string(Rule), !IO),
		nl(!IO)
			), KB^kb_rules, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func step_to_string(step) = string.

step_to_string(assume(P, F)) = "assume("
		++ vsmprop_to_string(P) ++ ", cost=" ++ cost_function_to_string(F) ++ ")".
step_to_string(resolve_rule(vs(MRule, Varset), Subst)) = "resolve_rule(("
		++ vsmrule_to_string(vs(MRule, Varset)) ++ "), " ++ subst_to_string(Varset, Subst) ++ ")".
step_to_string(use_fact(vs(MProp, Varset), Subst)) = "use_fact("
		++ vsmprop_to_string(vs(MProp, Varset)) ++ ", " ++ subst_to_string(Varset, Subst)
		++ ", cost=1.0)".  % XXX DON'T have this hard-wired here!!!

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func subst_to_string(varset, subst) = string.

subst_to_string(Varset, Subst) = "{" ++ Str ++ "}" :-
	L = map.to_assoc_list(Subst),
	L0 = list.map((func(Var-Value) = S :-
		S = varset.lookup_name(Varset, Var) ++ "=" ++ formula_term_to_string(Varset, Value)), L),
	Str = string.join_list(", ", L0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_proof(proof::in, io::di, io::uo) is det.

print_proof(Proof, !IO) :-
	vs(LastGoal, Varset) = last_goal(Proof),

	print("Proven goal:\n", !IO),
	print("  " ++ string.join_list(", ", list.map((func(MProp-Marking) = S :-
			S = mprop_to_string(Varset, MProp) ++ "[" ++ string(Marking) ++ "]"
				), LastGoal)) ++ "\n", !IO),

	nl(!IO),

	print("Assumptions:\n", !IO),
	print("  " ++ string.join_list("\n  ", list.map((func(vs(MProp, VS)) = S :-
			S = mprop_to_string(VS, MProp)
				), set.to_sorted_list(assumed(Proof)))) ++ "\n", !IO),

	nl(!IO),

	print("Proof trace:\n", !IO),
	Proof^p_goals = vs(Goals, Varset0),
	GoalsStr = list.map((func(Goal) = GStr :-
		GStr = string.join_list(", ", list.map((func(MProp-Marking) = S :-
			S = mprop_to_string(Varset0, MProp) ++ "[" ++ string(Marking) ++ "]"
				), Goal))), list.reverse(Goals)),
	print("  " ++ string.join_list("\n  ", GoalsStr) ++ "\n", !IO),

	nl(!IO),

	print("Steps:\n", !IO),
	print("  " ++ string.join_list("\n  ", list.map(step_to_string, list.reverse(Proof^p_steps))), !IO),
	nl(!IO).
