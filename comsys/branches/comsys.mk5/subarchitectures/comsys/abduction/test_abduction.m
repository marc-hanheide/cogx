% $Id$

:- module test_abduction.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string.
:- import_module abduction, kb, lf, formula, context.

:- import_module parser, term_io, term, varset.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

main(!IO) :-
	some [!KB] (
		!:KB = kb.init,

			% knowledge base
/*
		add_to_kb("r(a), p(Y, X) -> q(X).", !KB),
		add_to_kb("p(a, c).", !KB),
		add_to_kb("r(a).", !KB),
		add_to_kb("r(X) -> q(X).", !KB),
*/

		kb.add_vsmprop(det_string_to_vsmprop("a0:agent(robot)."), !KB),
		kb.add_vsmprop(det_string_to_vsmprop("a0:agent(human)."), !KB),

%		kb.add_vsmrule(det_string_to_vsmrule("all:(i0:actor(S, A), i0:content(S, C) -> i0:sentence(S, s(A, C)))."), !KB),
		kb.add_vsmrule(det_string_to_vsmrule("a0:agent(A), i0:sentence(S) -> i0:actor(S, A)."), !KB),
		kb.add_vsmrule(det_string_to_vsmrule("a0:agent(Ag), i0:sentence(X) -> e0:utter(Ag, X)."), !KB),

		kb.add_vsmprop(det_string_to_vsmprop("i0:object(box)."), !KB),
		kb.add_vsmprop(det_string_to_vsmprop("a1:in_focus(box)."), !KB),
		kb.add_vsmprop(det_string_to_vsmprop("i0:object(ball)."), !KB),
		kb.add_vsmrule(det_string_to_vsmrule("i0:object(O), i0:sentence(S) -> i0:content(S, O)."), !KB),
		kb.add_vsmrule(det_string_to_vsmrule("i0:topic(S, X) -> i0:content(S, X)."), !KB),

		kb.add_vsmrule(det_string_to_vsmrule("e0:utter(Ag0, S), i0:topic(S, T) -> a1:in_focus(T)."), !KB),

/*
		add_to_kb("he(X), utter(\"utt\", E, P) -> add_to_cg(P).", !KB),
		add_to_kb("he(john(e)).", !KB),
		add_to_kb("he(X) -> leave(P, X).", !KB),
		add_to_kb("utter(\"utt\", e, p).", !KB),
*/
%		add_to_kb("a0:att(X) -> a1:att(X).", !KB),
%		add_to_kb("axiom:att(a).", !KB),

			% query
%		GT-GVS = string_to_varterm("add_to_cg(X)."),
		vs(InitMProp, InitVarset) = det_string_to_vsmprop("a1:in_focus(box)."),
%		GT-GVS = string_to_term_varset("e1:att(a)."),

		P0 = proof(vs([[InitMProp-unsolved]], InitVarset), []),

		print("KB = {\n", !IO),
		print_kb(!.KB, !IO),
		print("}\n", !IO),

		nl(!IO),

%		print_goal(G, GVS, !IO),

		DC0 = new_d_ctx,

		print("DC = ", !IO),
		print(DC0, !IO),
		nl(!IO),

		Proofs = solutions_set((pred(P::out) is nondet :-
			prove(P0, P, !.KB)
/*
			step(P0, P1, !.KB),
			step(P1, P2, !.KB),
			step(P2, P, !.KB)
*/
				)),

		set.fold((pred(Proof::in, !.IO::di, !:IO::uo) is det :-
			(if
				Proof = proof(vs([_|_], _), _)
			then
				print("--------------------------------------------------------------------------------\n", !IO),
				print_proof(Proof, !IO),

				nl(!IO),

				dialogue_turn(Proof, DC0, DC),
				print("Next DC = ", !IO),
				print(DC, !IO),
				nl(!IO)

			else
				true
			)
				), Proofs, !IO)

	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_kb(kb::in, io::di, io::uo) is det.

print_kb(KB, !IO) :-
	set.fold((pred(Fact::in, !.IO::di, !:IO::uo) is det :-
		print("  ", !IO),
		print(vsmprop_to_string(Fact), !IO),
		nl(!IO)
			), KB^kb_facts, !IO),
	set.fold((pred(Rule::in, !.IO::di, !:IO::uo) is det :-
			% XXX global context
		print("  ", !IO),
		print(vsmrule_to_string(Rule), !IO),
		nl(!IO)
			), KB^kb_rules, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func step_to_string(step) = string.

step_to_string(assume(P)) = "assume("
		++ vsmprop_to_string(P) ++ ")".
step_to_string(resolve_rule(vs(MRule, Varset), Subst)) = "resolve_rule(("
		++ vsmrule_to_string(vs(MRule, Varset)) ++ "), " ++ subst_to_string(Varset, Subst) ++ ")".
step_to_string(resolve_fact(vs(MProp, Varset), Subst)) = "resolve_fact("
		++ vsmprop_to_string(vs(MProp, Varset)) ++ ", " ++ subst_to_string(Varset, Subst) ++ ")".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func subst_to_string(varset, subst) = string.

subst_to_string(Varset, Subst) = Str :-
	L = map.to_assoc_list(Subst),
	L0 = list.map((func(Var-Value) = S :-
		S = varset.lookup_name(Varset, Var) ++ "=" ++ formula_term_to_string(Varset, Value)), L),
	Str = string.join_list(", ", L0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_proof(proof::in, io::di, io::uo) is det.

print_proof(proof(vs(Goals, Varset), Steps), !IO) :-
	GoalsStr = list.map((func(Goal) = GStr :-
		GStr = string.join_list(", ", list.map((func(MProp-Marking) = S :-
			S = mprop_to_string(Varset, MProp) ++ "[" ++ string(Marking) ++ "]"
				), Goal))), list.reverse(Goals)),
	print(string.join_list("\n  >> ", GoalsStr), !IO),
	nl(!IO),
	nl(!IO),

	print("* ", !IO),
	print(string.join_list("\n* ", list.map(step_to_string, list.reverse(Steps))), !IO),
	nl(!IO).
