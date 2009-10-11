:- module 'check-rules-file'.

:- interface.
:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module require.
:- import_module list, string, bool.
:- import_module term, term_io.
:- import_module utils.
:- import_module stringable.
:- import_module formula, formula_io, formula_ops, modality.
:- import_module ctx_loadable, ctx_loadable_io, ctx_io.

main(!IO) :-
	command_line_arguments(CmdLineArgs, !IO),
	(if
		CmdLineArgs = [F|Fs]
	then
		some [!Ctx] (
			!:Ctx = new_ctx,
			check_rules_files([F|Fs], !Ctx, !IO),
			nl(!IO),
			print("Rules:\n", !IO),
			print_rules(!.Ctx, "  ", !IO)
		)
	else
		print("Usage: check-rules-file FILENAME[S...]\n", !IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred check_rules_files(list(string)::in, ctx::in, ctx::out, io::di, io::uo) is det.

check_rules_files([], !Ctx, !IO).
check_rules_files([F|Fs], !Ctx, !IO) :-
	print("[" ++ F ++ "] ", !IO),
	load_rules_from_file(F, !Ctx, !IO),
	print("ok\n", !IO),
	check_rules_files(Fs, !Ctx, !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred load_rules_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.
	
load_rules_from_file(Filename, !Ctx, !IO) :-
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
				error("Cannot convert term to rule in `" ++ Filename
						++ "' at line " ++ string.from_int(Line) ++ ".")
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
