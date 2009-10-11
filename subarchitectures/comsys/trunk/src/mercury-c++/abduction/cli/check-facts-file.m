:- module 'check-facts-file'.

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
:- import_module ctx_loadable, ctx_io.

main(!IO) :-
	command_line_arguments(CmdLineArgs, !IO),
	(if
		CmdLineArgs = [Filename]
	then
		load_facts_from_file(Filename, new_ctx, _Ctx, !IO),
		print("File ok.\n", !IO)
	else
		print("Usage: check-facts-file FILENAME\n", !IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred load_facts_from_file(string::in, ctx::in, ctx::out, io::di, io::uo) is det.
	
load_facts_from_file(Filename, !Ctx, !IO) :-
	see(Filename, SeeRes, !IO),
	(SeeRes = ok -> true ; error("can't open the facts file")),

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
