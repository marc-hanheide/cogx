% $Id$

:- module test_abduction.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string, float, int, bag, bool.
:- import_module abduction, kb, formula, context, costs.

:- import_module parser, term_io, term, varset, formula_io.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

main(!IO) :-
	io.command_line_arguments(CmdArgs, !IO),
	(if
		CmdArgs = [Goal, GoalAssumeCost],
		string.to_float(GoalAssumeCost, InitAssumeCost)
	then
		some [!KB] (
			!:KB = kb.init,

%			read_file_as_lines(FileName, Strs0, !IO),
%			preprocess_file(Strs0, Strs),

			do_while((pred(Continue::out, !.KB::in, !:KB::out, !.IO::di, !:IO::uo) is det :-
				term_io.read_term_with_op_table(init_wabd_op_table, ReadResult, !IO),
				(
					ReadResult = term(VS, Term),
					generic_term(Term),
					(if term_to_mrule(Term, MRule)
					then kb.add_vsmrule(vs(MRule, VS), !KB), Continue = yes
					else
						(if term_to_mprop(Term, MProp)
						then kb.add_vsmprop(vs(MProp, VS), !KB), Continue = yes
						else error("Syntax error.")
						)
					)
				;
					ReadResult = error(Message, Linenumber),
					error(Message ++ " at line " ++ string.from_int(Linenumber) ++ ".")
				;
					ReadResult = eof,
					Continue = no
				)
					), !KB, !IO),

			vs(InitMProp, InitVarset) = det_string_to_vsmprop(Goal),

			P0 = proof(vs([[InitMProp-unsolved(const(InitAssumeCost))]], InitVarset), []),

			format("Goal\n  %s\n\n", [s(vsmprop_to_string(vs(InitMProp, InitVarset)))], !IO),

			print_kb(!.KB, !IO),

			nl(!IO),

			DC0 = new_d_ctx,

			Proofs0 = set.to_sorted_list(solutions_set((pred(Cost-P::out) is nondet :-
				prove(P0, P, !.KB),
				Cost = cost(DC0, P, 1.0)
					))),

			list.sort((pred(CA-_::in, CB-_::in, Comp::out) is det :-
				float_compare(CA, CB, Comp)
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

:- pred do_while(pred(bool, A, A, T, T), A, A, T, T).
:- mode do_while((pred(out, in, out, di, uo) is det), in, out, di, uo) is det.
:- mode do_while((pred(out, in, out, in, out) is det), in, out, in, out) is det.

do_while(Pred, A0, A, B0, B) :-
	call(Pred, Result, A0, A1, B0, B1),
	(
		Result = yes,
		do_while(Pred, A1, A, B1, B)
	;
		Result = no,
		A = A1, B = B1
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred float_compare(float::in, float::in, comparison_result::out) is det.

float_compare(A, B, R) :-
	(if A < B
	then R = (<)
	else
		(if A > B
		then R = (>)
		else R = (=)
		)
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
			), facts(KB), !IO),

	nl(!IO),

	print("Rules:\n", !IO),
	set.fold((pred(Rule::in, !.IO::di, !:IO::uo) is det :-
			% XXX global context
		print("  ", !IO),
		print(vsmrule_to_string(Rule), !IO),
		nl(!IO)
			), rules(KB), !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func step_to_string(step) = string.

step_to_string(assume(P, F)) = "assume("
		++ vsmprop_to_string(P) ++ "), cost=" ++ cost_function_to_string(F).
step_to_string(resolve_rule(vs(MRule, Varset), Subst)) = "resolve_rule("
		++ vsmrule_to_string(vs(MRule, Varset)) ++ "), " ++ subst_to_string(Varset, Subst).
step_to_string(use_fact(vs(MProp, Varset), Subst)) = "use_fact("
		++ vsmprop_to_string(vs(MProp, Varset)) ++ ", " ++ subst_to_string(Varset, Subst)
		++ ", cost=1.0".  % XXX DON'T have this hard-wired here!!!
step_to_string(factor(Subst, Varset)) = "factor, " ++ subst_to_string(Varset, Subst).

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
	print("  " ++ string.join_list("\n  ", list.map((func(vs(MProp, VS)-Count) = S :-
			S = string.from_int(Count) ++ "x " ++ mprop_to_string(VS, MProp)
				), bag.to_assoc_list(assumptions(Proof)))) ++ "\n", !IO),

	nl(!IO),

	print("Proof trace:\n", !IO),
	Proof^p_goals = vs(RevGoals, Varset0),
	Qs = reverse(RevGoals),
	InitQs = det_head(Qs),
	RemQss = det_tail(Qs),

	print("  " ++ proof_state_to_string(Varset0, InitQs) ++ "\n", !IO),

	GoalsStr = list.map((func(Step-Goal) = GStr :-
		GStr = "\t" ++ step_to_string(Step) ++ "\n  " ++ proof_state_to_string(Varset0, Goal)
				), from_corresponding_lists(Proof^p_steps, RemQss)),
	print("  " ++ string.join_list("\n", GoalsStr) ++ "\n", !IO),

	nl(!IO).

/*
	print("Steps:\n", !IO),
	print("  " ++ string.join_list("\n  ", list.map(step_to_string, list.reverse(Proof^p_steps))), !IO),
	nl(!IO).
*/

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func proof_state_to_string(varset, list(marked(mprop))) = string.

proof_state_to_string(Varset, L) = S :-
	S = string.join_list(", ", list.map((func(MProp-Marking) = QS :-
		QS = mprop_to_string(Varset, MProp) ++ "[" ++ string(Marking) ++ "]"
			), L)).
