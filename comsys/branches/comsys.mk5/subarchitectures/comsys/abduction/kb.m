% $Id$

:- module kb.

:- interface.

:- import_module string.
:- import_module set, list, pair.
:- import_module term, varset.
:- import_module formulae, abduction.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type kb
	--->	kb(
		kb_facts :: set(kbfact),
		kb_rules :: set(kbrule)
	).

:- type kbitem
	--->	r(kbrule)
	;	f(kbfact)
	.

:- func init = kb.

:- pred add_to_kb(string::in, kb::in, kb::out) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module term_io, parser.
:- import_module require.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

init = kb(init, init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_to_kb(String, kb(Fs0, Rs0), kb(Fs, Rs)) :-
	string_to_term_varset(String, T, VS),
	term_varset_to_kbitem(T, VS, It),
	(
		It = f(Fact),
		Fs = set.insert(Fs0, Fact),
		Rs = Rs0
	;
		It = r(Rule),
		Fs = Fs0,
		Rs = set.insert(Rs0, Rule)
	).

