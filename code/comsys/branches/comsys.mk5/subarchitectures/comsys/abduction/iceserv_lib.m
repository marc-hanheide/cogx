:- module iceserv_lib.

:- interface.
:- import_module io.
:- import_module ctx_specific, ctx_modality, abduction.

:- func srv_init_ctx = ctx.
:- pred srv_clear_rules(ctx::in, ctx::out) is det.
:- pred srv_load_rules_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.
:- pred srv_get_best_proof(ctx::in, proof(ctx_modality)::out) is semidet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module set, list, bool, string.
:- import_module term, term_io, formula.
:- import_module formula_io, formula_ops, ctx_io.
:- import_module solutions, require.
:- import_module varset.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_init_ctx = out, "init_ctx").

srv_init_ctx = new_ctx.

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

:- pragma foreign_export("C", srv_get_best_proof(in, out), "get_best_proof").

srv_get_best_proof(Ctx, Proof) :-
	Proof = new_proof([], varset.init).
