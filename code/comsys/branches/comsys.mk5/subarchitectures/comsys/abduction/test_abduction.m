% $Id$

:- module test_abduction.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string.
:- import_module abduction, kb, lf, formulae, context.

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

		add_to_kb("a0:agent(robot).", !KB),
		add_to_kb("a0:agent(human).", !KB),

		add_to_kb("axiom:(i0:actor(S, A), i0:content(S, C) -> i0:sentence(S, s(A, C))).", !KB),
		add_to_kb("a0:agent(A), i0:sentence(S) -> i0:actor(S, A).", !KB),
		add_to_kb("a0:agent(Ag), i0:sentence(X) -> e0:utter(Ag, X).", !KB),

		add_to_kb("i0:object(box).", !KB),
		add_to_kb("i0:object(ball).", !KB),
		add_to_kb("i0:object(O), i0:sentence(S) -> i0:content(S, O).", !KB),
		add_to_kb("i0:topic(S, X) -> i0:content(S, X).", !KB),

		add_to_kb("e0:utter(Ag0, S), i0:topic(S, T) -> a1:in_focus(T).", !KB),

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
		GT-GVS = string_to_term_varset("a1:in_focus(box)."),
%		GT-GVS = string_to_term_varset("e1:att(a)."),

		P0 = proof([[([]-GT)-unsolved]], [], GVS),

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
				Proof = proof([Goal|_], _, PVS)
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

:- func fact_to_string(kbfact) = string.

fact_to_string(fact(Term, Varset)) = ctxvarterm_to_string(Term-Varset).

:- func rule_to_string(kbrule) = string.

rule_to_string(rule(RCtx, Ante, H, Varset)) = ctx_to_string(RCtx) ++ ":(" ++ string.join_list(", ", list.map(ctxterm_to_string(Varset), Ante))
		++ " -> " ++ ctxterm_to_string(Varset, H) ++ ")".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_kb(kb::in, io::di, io::uo) is det.

print_kb(KB, !IO) :-
	set.fold((pred(fact(Fact, Varset)::in, !.IO::di, !:IO::uo) is det :-
		print("  ", !IO),
		print(ctxterm_to_string(Varset, Fact), !IO),
		nl(!IO)
			), KB^kb_facts, !IO),
	set.fold((pred(Rule::in, !.IO::di, !:IO::uo) is det :-
			% XXX global context
		print("  ", !IO),
		print(rule_to_string(Rule), !IO),
		nl(!IO)
			), KB^kb_rules, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func step_to_string(step) = string.

step_to_string(assume(Term)) = "assume("
		++ ctxterm_to_string(init, Term) ++ ")".
step_to_string(resolve_rule(rule(RCtx, Ante, H, Varset), Subst)) = "resolve_rule(("
		++ rule_to_string(rule(RCtx, Ante, H, Varset)) ++ "), " ++ subst_to_string(Varset, Subst) ++ ")".
step_to_string(resolve_fact(fact(Fact, Varset), Subst)) = "resolve_fact("
		++ fact_to_string(fact(Fact, Varset)) ++ ", " ++ subst_to_string(Varset, Subst) ++ ")".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func subst_to_string(varset, substitution) = string.

subst_to_string(Varset, Subst) = Str :-
	L = map.to_assoc_list(Subst),
	L0 = list.map((func(Var-Term) = S :-
		S = varset.lookup_name(Varset, Var) ++ "=" ++ term_to_string(Varset, Term)), L),
	Str = string.join_list(", ", L0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred print_proof(proof::in, io::di, io::uo) is det.

print_proof(proof(Goals, Steps, Varset), !IO) :-
	GoalsStr = list.map((func(Goal) = GStr :-
		GStr = string.join_list(", ", list.map((func(Term-Marking) = S :-
			S = ctxterm_to_string(Varset, Term) ++ "[" ++ string(Marking) ++ "]"
				), Goal))), list.reverse(Goals)),
	print(string.join_list("\n  >> ", GoalsStr), !IO),
	nl(!IO),
	nl(!IO),

	print("* ", !IO),
	print(string.join_list("\n* ", list.map(step_to_string, list.reverse(Steps))), !IO),
	nl(!IO).
