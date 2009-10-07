:- module 'MercuryAbducerServer_mint'.

:- interface.
:- import_module io.
:- import_module float, list.
:- import_module ctx_loadable, ctx_modality, abduction, formula.
:- import_module varset.

:- func srv_init_ctx = ctx.

:- pred srv_clear_rules(ctx::in, ctx::out) is det.
:- pred srv_load_rules_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.

:- pred srv_clear_facts(ctx::in, ctx::out) is det.
:- pred srv_clear_e_facts(ctx::in, ctx::out) is det.
:- pred srv_clear_a_facts(ctx::in, ctx::out) is det.
:- pred srv_clear_i_facts(ctx::in, ctx::out) is det.
:- pred srv_clear_k_facts(ctx::in, ctx::out) is det.

:- pred srv_load_facts_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.
:- pred srv_add_mprop_fact(varset::in, mprop(ctx_modality)::in, ctx::in, ctx::out) is det.

:- pred srv_clear_assumables(ctx::in, ctx::out) is det.
:- pred srv_add_assumable(string::in, mprop(ctx_modality)::in, float::in, ctx::in, ctx::out) is det.

%:- pred srv_prove_best(string::in, float::in, ctx::in, float::out, proof(ctx_modality)::out) is semidet.
:- pred srv_prove_best(proof(ctx_modality)::in, ctx::in, float::out, proof(ctx_modality)::out) is semidet.
:- pred srv_dissect_proof(proof(ctx_modality)::in, ctx::in, float::out, list(string)::out, list(string)::out) is det.
:- pred srv_print_ctx(ctx::in, io::di, io::uo) is det.
:- pred srv_proof_summary(proof(ctx_modality)::in, ctx::in, io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.
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
:- import_module ctx_loadable_io.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_init_ctx = out, "init_ctx").

srv_init_ctx = new_ctx.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func default_costs = costs.

default_costs = costs(1.0, 1.0, 0.1).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_print_ctx(in, di, uo), "print_ctx").

srv_print_ctx(C, !IO) :-
   	print_ctx(C, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_rules(in, out), "clear_rules").

srv_clear_rules(!Ctx) :-
	set_rules(set.init, !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_load_rules_from_file(in, in, out, di, uo), "load_rules_from_file").

srv_load_rules_from_file(Filename, !Ctx, !IO) :-
	see(Filename, SeeRes, !IO),
	(SeeRes = ok -> true ; error("can't open the rule file")),

	do_while((pred(Continue::out, !.Ctx::in, !:Ctx::out, !.IO::di, !:IO::uo) is det :-
		term_io.read_term_with_op_table(init_wabd_op_table, ReadResult, !IO),
		(
			ReadResult = term(VS, Term),
			generic_term(Term),
			(if term_to_mrule(Term, MRule)
			then add_rule(vs(MRule, VS), !Ctx), Continue = yes
			else
				context(_, Line) = get_term_context(Term),
				error("Syntax error in rule file " ++ Filename
						++ " at line " ++ string.from_int(Line) ++ ".")
			)
		;
			ReadResult = error(Message, Linenumber),
			error(Message ++ " at line " ++ string.from_int(Linenumber) ++ ".")
		;
			ReadResult = eof,
			Continue = no
		)
			), !Ctx, !IO),

	seen(!IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_facts(in, out), "clear_facts").
:- pragma foreign_export("C", srv_clear_e_facts(in, out), "clear_e_facts").
:- pragma foreign_export("C", srv_clear_a_facts(in, out), "clear_a_facts").
:- pragma foreign_export("C", srv_clear_i_facts(in, out), "clear_i_facts").
:- pragma foreign_export("C", srv_clear_k_facts(in, out), "clear_k_facts").

srv_clear_facts(!Ctx) :-
	set_facts(set.init, !Ctx).

srv_clear_e_facts(!Ctx) :-
	set_facts(set.filter((pred(vs(m(Mod, _), _)::in) is semidet :-
		Mod \= [e(_)|_]
			), !.Ctx^facts), !Ctx).

srv_clear_a_facts(!Ctx) :-
	set_facts(set.filter((pred(vs(m(Mod, _), _)::in) is semidet :-
		Mod \= [a(_)|_]
			), !.Ctx^facts), !Ctx).

srv_clear_i_facts(!Ctx) :-
	set_facts(set.filter((pred(vs(m(Mod, _), _)::in) is semidet :-
		Mod \= [i|_]
			), !.Ctx^facts), !Ctx).

srv_clear_k_facts(!Ctx) :-
	set_facts(set.filter((pred(vs(m(Mod, _), _)::in) is semidet :-
		Mod \= [k(_, _)|_]
			), !.Ctx^facts), !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_load_facts_from_file(in, in, out, di, uo), "load_facts_from_file").

srv_load_facts_from_file(Filename, !Ctx, !IO) :-
	see(Filename, SeeRes, !IO),
	(SeeRes = ok -> true ; error("can't open the rule file")),

	do_while((pred(Continue::out, !.Ctx::in, !:Ctx::out, !.IO::di, !:IO::uo) is det :-
		term_io.read_term_with_op_table(init_wabd_op_table, ReadResult, !IO),
		(
			ReadResult = term(VS, Term),
			generic_term(Term),
			(if term_to_mprop(Term, m(Mod, Prop))
			then add_fact(vs(m(Mod, Prop), VS), !Ctx), Continue = yes
			else
				context(_, Line) = get_term_context(Term),
				error("Syntax error in facts file " ++ Filename
						++ " at line " ++ string.from_int(Line) ++ ".")
			)
		;
			ReadResult = error(Message, Linenumber),
			error(Message ++ " at line " ++ string.from_int(Linenumber) ++ ".")
		;
			ReadResult = eof,
			Continue = no
		)
			), !Ctx, !IO),

	seen(!IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_add_mprop_fact(in, in, in, out), "add_mprop_fact").

srv_add_mprop_fact(VS, MProp, !Ctx) :-
	add_fact(vs(MProp, VS), !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_assumables(in, out), "clear_assumables").

srv_clear_assumables(!Ctx) :-
	set_assumables(map.init, !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_add_assumable(in, in, in, in, out), "add_assumable").

srv_add_assumable(Function, m(Mod, Prop), Cost, !Ctx) :-
	Ass = !.Ctx^assumables,
	(if map.search(Ass, Function, Mapping0)
	then Mapping = Mapping0
	else Mapping = map.init
	),
	map.set(Mapping, m(Mod, det_formula_to_ground_formula(Prop)), Cost, MappingNew),
	map.set(Ass, Function, MappingNew, Ass1),
	set_assumables(Ass1, !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_prove_best(in, in, out, out), "prove_best").

srv_prove_best(P0, Ctx, ProofCost, Proof) :-
%	VSMProp = vs(InitMProp, InitVarset),
%	vs(InitMProp, InitVarset) = det_string_to_vsmprop(GoalStr),

%	P0 = proof(vs([list.map((func(cf(MProp, Func)) = unsolved(MProp, Func)), AnnotMProps)], VS), []),

	Proofs0 = set.to_sorted_list(solutions_set((pred(Cost-P::out) is nondet :-
		prove(0.0, 200.0, P0, P, default_costs, Ctx),
		Cost = cost(Ctx, P, default_costs)
			))),

	list.sort((pred(CA-_::in, CB-_::in, Comp::out) is det :-
		float_compare(CA, CB, Comp)
			), Proofs0, [ProofCost-Proof|_]).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

%:- pragma foreign_export("C", srv_dissect_proof(in, in, out, out, out), "dissect_proof").

srv_dissect_proof(Proof, _Ctx, Cost, Assumed, Asserted) :-
%	Costs = costs(1.0, 1.0, 0.1),
	Cost = 0.0,

	LastGoal = last_goal(Proof),
	Assumed = list.map((func(cf(m(Mod, GProp), _AssFunc)) = String :-
		String = mprop_to_string(varset.init, m(Mod, ground_formula_to_formula(GProp)))
			), bag.to_list(goal_assumptions(LastGoal))),

	Asserted = [],

	(if Asserted = []
	then true %error("conversion error!")
	else true
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_proof_summary(in, in, di, uo), "proof_summary").

srv_proof_summary(Proof, Ctx, !IO) :-
	LastGoal = last_goal(Proof),

	print_ctx(Ctx, !IO),

	format("proof cost = %f\n\n", [f(cost(Ctx, Proof, default_costs))], !IO),
	print("proven goal:\n  " ++ goal_to_string(LastGoal) ++ "\n", !IO),
	nl(!IO),

	print("assumptions:\n", !IO),
	print("  " ++ assumptions_to_string(Ctx, goal_assumptions(LastGoal)) ++ "\n", !IO),
	nl(!IO),

	print("assertions:\n", !IO),
	print("  " ++ assertions_to_string(Ctx, goal_assertions(LastGoal)) ++ "\n", !IO),
	nl(!IO).

%	print_proof_trace(Ctx, Proof, !IO),
%	nl(!IO),
%	print("that's it for the summary.\n", !IO).
