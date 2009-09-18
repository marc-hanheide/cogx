% $Id$

:- module kb.

:- interface.

:- import_module set.
:- import_module formula.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type kb.

:- func init = kb.

:- pred add_vsmprop(vsmprop::in, kb::in, kb::out) is det.
:- pred add_vsmrule(vsmrule::in, kb::in, kb::out) is det.

	% for debugging purposes only!
:- func facts(kb) = set(vsmprop).
:- func rules(kb) = set(vsmrule).

:- pred fact(kb::in, vsmprop::out) is nondet.
:- pred rule(kb::in, vsmrule::out) is nondet.
:- pred assumable(kb::in, vsmprop::out) is nondet.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.

:- type kb
	--->	kb(
		kb_facts :: set(vsmprop),
		kb_rules :: set(vsmrule)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

init = kb(init, init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_vsmprop(P, kb(Fs0, Rs), kb(Fs, Rs)) :-
	Fs = set.insert(Fs0, P).

add_vsmrule(R, kb(Fs, Rs0), kb(Fs, Rs)) :-
	Rs = set.insert(Rs0, R).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

facts(KB) = KB^kb_facts.

rules(KB) = KB^kb_rules.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

fact(KB, Fact) :-
	set.member(Fact, KB^kb_facts).

rule(KB, Rule) :-
	set.member(Rule, KB^kb_rules).

assumable(KB, _) :-
	fail.
