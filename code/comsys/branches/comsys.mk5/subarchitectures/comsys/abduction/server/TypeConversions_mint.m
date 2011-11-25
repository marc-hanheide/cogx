:- module 'TypeConversions_mint'.

:- interface.
:- import_module float, list, bool.
:- import_module ctx_specific, ctx_modality, abduction, formula, belief_model, costs.
:- import_module varset.

:- pred new_with_const_cost_function(mprop(ctx_modality)::in, float::in, with_cost_function(mprop(ctx_modality))::out) is det.

:- pred new_mprop(list(ctx_modality)::in, atomic_formula::in, mprop(ctx_modality)::out, varset::in, varset::out) is det.

:- pred new_atomic_formula(string::in, list(formula.term)::in, atomic_formula::out, varset::in, varset::out) is det.
:- pred new_term(string::in, list(formula.term)::in, formula.term::out, varset::in, varset::out) is det.
:- pred new_var(string::in, formula.term::out, varset::in, varset::out) is det.

:- pred new_varset(varset::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred empty_term_list(list(formula.term)::out) is det.
:- pred cons_term_list(formula.term::in, list(formula.term)::in, list(formula.term)::out) is det.

:- pred empty_mprop_list(list(mprop(ctx_modality))::out) is det.
:- pred cons_mprop_list(mprop(ctx_modality)::in, list(mprop(ctx_modality))::in, list(mprop(ctx_modality))::out) is det.

:- pred empty_marked_query_list(list(query(ctx_modality))::out) is det.
:- pred cons_marked_query_list(query(ctx_modality)::in, list(query(ctx_modality))::in, list(query(ctx_modality))::out) is det.

:- pred empty_ctx_modality_list(list(ctx_modality)::out) is det.
:- pred cons_ctx_modality_list(ctx_modality::in, list(ctx_modality)::in, list(ctx_modality)::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred modality_event(ctx_modality::out) is det.
:- pred modality_info(ctx_modality::out) is det.
:- pred modality_att(ctx_modality::out) is det.
:- pred modality_k(ctx_modality::out) is det.

:- pred impure_print_list_modalities(list(ctx_modality)::in) is det.
:- pred impure_print_modality(ctx_modality::in) is det.

:- pred is_modality_event(ctx_modality::in) is semidet.
:- pred is_modality_info(ctx_modality::in) is semidet.
:- pred is_modality_att(ctx_modality::in) is semidet.
:- pred is_modality_k(ctx_modality::in, belief::out) is semidet.

:- pred belief_private(belief::out) is det.

:- pred is_belief_private(belief::in, string::out) is semidet.
:- pred is_belief_attrib(belief::in, string::out, string::out) is semidet.
:- pred is_belief_mutual(belief::in, list(string)::out) is semidet.

%------------------------------------------------------------------------------%

:- pred const_cost_function(float::in, cost_function::out) is det.
:- pred named_cost_function(string::in, cost_function::out) is det.

:- pred is_const_cost_function(cost_function::in, float::out) is semidet.
:- pred is_named_cost_function(cost_function::in, string::out) is semidet.

%------------------------------------------------------------------------------%

:- pred proved_query(mprop(ctx_modality)::in, query(ctx_modality)::out) is det.
:- pred unsolved_query(mprop(ctx_modality)::in, cost_function::in, query(ctx_modality)::out) is det.
:- pred assumed_query(mprop(ctx_modality)::in, cost_function::in, query(ctx_modality)::out) is det.
:- pred asserted_query(mprop(ctx_modality)::in, list(mprop(ctx_modality))::in, query(ctx_modality)::out) is det.

:- pred is_proved_query(query(ctx_modality)::in, mprop(ctx_modality)::out) is semidet.
:- pred is_unsolved_query(query(ctx_modality)::in, mprop(ctx_modality)::out, cost_function::out) is semidet.
:- pred is_assumed_query(query(ctx_modality)::in, mprop(ctx_modality)::out, cost_function::out) is semidet.
:- pred is_asserted_query(query(ctx_modality)::in, mprop(ctx_modality)::out, list(mprop(ctx_modality))::out) is semidet.

%------------------------------------------------------------------------------%

:- pred dissect_term(varset::in, formula.term::in, bool::out, string::out, list(formula.term)::out) is det.
:- pred dissect_predicate(varset::in, atomic_formula::in, string::out, list(formula.term)::out) is det.
:- pred dissect_mprop(mprop(ctx_modality)::in, list(ctx_modality)::out, atomic_formula::out) is det.
:- pred dissect_proof(ctx::in, proof(ctx_modality)::in, varset::out, list(mprop(ctx_modality))::out,
		float::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred new_proof(list(query(ctx_modality))::in, varset::in, proof(ctx_modality)::out) is det.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module io.
:- import_module set, map, list, bool, string, pair, bag, assoc_list.
:- import_module utils.
:- import_module term, term_io, formula.
:- import_module formula_io, formula_ops, ctx_io.
:- import_module solutions, require.
:- import_module varset, costs.
:- import_module modality, stringable, context, costs.
:- import_module belief_model, lf, stf.
:- import_module lf_io.
:- import_module varset.

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", new_mprop(in, in, out, in, out), "new_mprop").

new_mprop(Mod, F, MProp, !VS) :-
	MProp = m(Mod, F).
%	trace [io(!IO)] (print("MProp = " ++ string(MProp), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_atomic_formula(in, in, out, in, out), "new_atomic_formula").

new_atomic_formula(PredSym, Args, F, !VS) :-
	F = p(PredSym, Args).
%	trace [io(!IO)] (print("Atomic formula = " ++ string(F), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_term(in, in, out, in, out), "new_term").

new_term(Functor, Args, T, !VS) :-
	T = t(Functor, Args).
%	trace [io(!IO)] (print("Term = " ++ string(T), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_var(in, out, in, out), "new_var").

new_var(Name, T, !VS) :-
	varset.create_name_var_map(!.VS, VarNames),
	(if map.search(VarNames, Name, Var0)
	then Var = Var0
	else varset.new_named_var(!.VS, Name, Var, !:VS)
	),
	T = v(Var).
%	trace [io(!IO)] (print("Var = " ++ string(T), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_varset(out), "new_varset").

new_varset(varset.init).

:- pragma foreign_export("C", new_with_const_cost_function(in, in, out), "new_with_const_cost_function").

new_with_const_cost_function(MProp, Cost, cf(MProp, const(Cost))).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", empty_term_list(out), "empty_term_list").
:- pragma foreign_export("C", cons_term_list(in, in, out), "cons_term_list").

empty_term_list([]).
cons_term_list(H, T, [H|T]).

:- pragma foreign_export("C", empty_mprop_list(out), "empty_mprop_list").
:- pragma foreign_export("C", cons_mprop_list(in, in, out), "cons_mprop_list").

empty_mprop_list([]).
cons_mprop_list(H, T, [H|T]).

:- pragma foreign_export("C", empty_marked_query_list(out), "empty_marked_query_list").
:- pragma foreign_export("C", cons_marked_query_list(in, in, out), "cons_marked_query_list").

empty_marked_query_list([]).
cons_marked_query_list(H, T, [H|T]) :- trace[io(!IO)] (print(string([H|T]) ++ "\n", !IO)).

:- pragma foreign_export("C", empty_ctx_modality_list(out), "empty_ctx_modality_list").
:- pragma foreign_export("C", cons_ctx_modality_list(in, in, out), "cons_ctx_modality_list").

empty_ctx_modality_list([]).
cons_ctx_modality_list(H, T, [H|T]).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", modality_event(out), "modality_event").
:- pragma foreign_export("C", modality_info(out), "modality_info").
:- pragma foreign_export("C", modality_att(out), "modality_att").
:- pragma foreign_export("C", modality_k(out), "modality_k").

modality_event(e(now)).
modality_info(i).
modality_att(a(com)).
modality_k(k(now, private(human))).

:- pragma foreign_export("C", impure_print_modality(in), "print_modality").
:- pragma foreign_export("C", impure_print_list_modalities(in), "print_list_modalities").

impure_print_modality(Mod) :-
	trace[io(!IO)] (print(string(Mod) ++ "\n", !IO)).

impure_print_list_modalities(Mod) :-
	trace[io(!IO)] (print(string(Mod) ++ "\n", !IO)).

:- pragma foreign_export("C", is_modality_event(in), "is_modality_event").
:- pragma foreign_export("C", is_modality_info(in), "is_modality_info").
:- pragma foreign_export("C", is_modality_att(in), "is_modality_att").
:- pragma foreign_export("C", is_modality_k(in, out), "is_modality_k").

is_modality_event(e(now)).% :- trace[io(!IO)] (print("merc: event\n", !IO)).
is_modality_info(i).% :- trace[io(!IO)] (print("merc: info\n", !IO)).
is_modality_att(a(com)).% :- trace[io(!IO)] (print("merc: att\n", !IO)).
is_modality_k(k(now, Belief), Belief).% :- trace[io(!IO)] (print("merc: k, bel=" ++ string(Belief) ++ "\n", !IO)).

:- pragma foreign_export("C", belief_private(out), "belief_private").

belief_private(private(human)).

:- pragma foreign_export("C", is_belief_private(in, out), "is_belief_private").
:- pragma foreign_export("C", is_belief_attrib(in, out, out), "is_belief_attrib").
:- pragma foreign_export("C", is_belief_mutual(in, out), "is_belief_mutual").

is_belief_private(private(Ag), to_string(Ag)).
is_belief_attrib(attrib(AgA, AgB), to_string(AgA), to_string(AgB)).
is_belief_mutual(mutual(SetAgs), ListAgs) :-
	ListAgs = list.map((func(Ag) = to_string(Ag)), set.to_sorted_list(SetAgs)).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", const_cost_function(in, out), "const_cost_function").
:- pragma foreign_export("C", named_cost_function(in, out), "named_cost_function").

const_cost_function(Num, const(Num)).
named_cost_function(Name, f(Name)).

:- pragma foreign_export("C", is_const_cost_function(in, out), "is_const_cost_function").
:- pragma foreign_export("C", is_named_cost_function(in, out), "is_named_cost_function").

is_const_cost_function(const(Num), Num).
is_named_cost_function(f(Name), Name).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", proved_query(in, out), "proved_query").
:- pragma foreign_export("C", unsolved_query(in, in, out), "unsolved_query").
:- pragma foreign_export("C", assumed_query(in, in, out), "assumed_query").
:- pragma foreign_export("C", asserted_query(in, in, out), "asserted_query").

proved_query(MProp, proved(MProp)).
unsolved_query(MProp, CostFunc, unsolved(MProp, CostFunc)).
assumed_query(MProp, CostFunc, assumed(MProp, CostFunc)).
asserted_query(MProp, [], asserted(prop(MProp))).
asserted_query(MProp, [H|T], asserted(impl([H|T], MProp))).

:- pragma foreign_export("C", is_proved_query(in, out), "is_proved_query").
:- pragma foreign_export("C", is_unsolved_query(in, out, out), "is_unsolved_query").
:- pragma foreign_export("C", is_assumed_query(in, out, out), "is_assumed_query").
:- pragma foreign_export("C", is_asserted_query(in, out, out), "is_asserted_query").

is_proved_query(proved(MProp), MProp).
is_unsolved_query(unsolved(MProp, CostFunction), MProp, CostFunction).
is_assumed_query(assumed(MProp, CostFunction), MProp, CostFunction).
is_asserted_query(asserted(prop(MProp)), MProp, []).
is_asserted_query(asserted(impl(AnteProps, MProp)), MProp, AnteProps).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", dissect_term(in, in, out, out, out), "dissect_term").

dissect_term(VS, v(Var), yes, VarName, []) :-
	varset.lookup_name(VS, Var, "V_", VarName).

dissect_term(_VS, t(Functor, Args), no, Functor, Args).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_predicate(in, in, out, out), "dissect_predicate").

dissect_predicate(_VS, p(PredSym, Args), PredSym, Args).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_mprop(in, out, out), "dissect_mprop").

dissect_mprop(m(Mod, Pred), Mod, Pred).
%	trace [io(!IO)] (print("dissecting mprop: " ++ string(m(Mod, Pred)), !IO)),

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_proof(in, in, out, out, out), "dissect_proof").

dissect_proof(Ctx, Proof, VS, LastGoals, Cost) :-
	vs(Qs, VS) = last_goal(Proof),
	LastGoals = list.map((func(Q) = MProp :-
		( Q = proved(MProp)
		; Q = assumed(MProp, _)
		; Q = unsolved(MProp, _)
		; Q = asserted(MTest),
			( MTest = prop(MProp)
			; MTest = impl(_, MProp)
			)
		)), Qs),

	Costs = costs(1.0, 1.0, 0.1),
	Cost = cost(Ctx, Proof, Costs).
%	trace [io(!IO)] ( print(LastGoals, !IO), nl(!IO) ).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", new_proof(in, in, out), "new_proof").

new_proof(MQs, VS, proof(vs([MQs], VS), [])).