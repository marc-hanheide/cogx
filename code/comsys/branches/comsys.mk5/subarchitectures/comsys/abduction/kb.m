% $Id$

:- module kb.

:- interface.

:- import_module set.
:- import_module formula.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type kb
	--->	kb(
		kb_facts :: set(vsmprop),
		kb_rules :: set(vsmrule)
	).

:- func init = kb.

:- pred add_vsmprop(vsmprop::in, kb::in, kb::out) is det.
:- pred add_vsmrule(vsmrule::in, kb::in, kb::out) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

init = kb(init, init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_vsmprop(P, kb(Fs0, Rs), kb(Fs, Rs)) :-
	Fs = set.insert(Fs0, P).

add_vsmrule(R, kb(Fs, Rs0), kb(Fs, Rs)) :-
	Rs = set.insert(Rs0, R).
