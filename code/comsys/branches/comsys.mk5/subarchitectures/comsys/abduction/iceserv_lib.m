:- module iceserv_lib.

:- interface.
:- import_module io.
:- import_module float.
:- import_module ctx_specific, ctx_modality, abduction.

:- func srv_init_ctx = ctx.
:- pred srv_clear_rules(ctx::in, ctx::out) is det.
:- pred srv_load_rules_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.
:- pred srv_prove_best(string::in, float::in, ctx::in, float::out, proof(ctx_modality)::out) is semidet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module set, list, bool, string, pair.
:- import_module utils.
:- import_module term, term_io, formula.
:- import_module formula_io, formula_ops, ctx_io.
:- import_module solutions, require.
:- import_module varset, costs.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_init_ctx = out, "init_ctx").

srv_init_ctx = new_ctx.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_rules(in, out), "clear_rules").

srv_clear_rules(!Ctx) :-
	set_explicit_rules(set.init, !Ctx).

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
			then add_explicit_rule(vs(MRule, VS), !Ctx), Continue = yes
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

:- pragma foreign_export("C", srv_prove_best(in, in, in, out, out), "prove_best").

srv_prove_best(GoalStr, AssCost, Ctx, ProofCost, Proof) :-
	vs(InitMProp, InitVarset) = det_string_to_vsmprop(GoalStr),

	P0 = proof(vs([[unsolved(InitMProp, const(AssCost))]], InitVarset), []),

	Proofs0 = set.to_sorted_list(solutions_set((pred(Cost-P::out) is nondet :-
		Costs = costs(1.0, 1.0, 0.1),
		prove(0.0, 10.0, P0, P, Costs, Ctx),
		Cost = cost(Ctx, P, Costs)
			))),

	list.sort((pred(CA-_::in, CB-_::in, Comp::out) is det :-
		float_compare(CA, CB, Comp)
			), Proofs0, [ProofCost-Proof|_]).
