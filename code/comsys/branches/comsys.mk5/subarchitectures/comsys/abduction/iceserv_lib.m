:- module iceserv_lib.

:- interface.
:- import_module ctx_loadable, abduction.

:- func srv_init_ctx = ctx.
:- pred srv_clear_facts(ctx::in, ctx::out) is det.
:- pred srv_clear_rules(ctx::in, ctx::out) is det.
:- pred srv_add_fact(string::in, ctx::in, ctx::out) is semidet.
:- pred srv_add_rule(string::in, ctx::in, ctx::out) is semidet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module set.
:- import_module formula_io, ctx_io.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_init_ctx = out, "init_ctx").

srv_init_ctx = new_ctx.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_facts(in, out), "clear_facts").

srv_clear_facts(!Ctx) :-
	set_facts(set.init, !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_clear_facts(in, out), "clear_rules").

srv_clear_rules(!Ctx) :-
	set_rules(set.init, !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_add_fact(in, in, out), "add_fact").

srv_add_fact(String, !Ctx) :-
	add_fact(string_to_vsmprop(String), !Ctx).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", srv_add_rule(in, in, out), "add_rule").

srv_add_rule(String, !Ctx) :-
	add_rule(string_to_vsmrule(String), !Ctx).
