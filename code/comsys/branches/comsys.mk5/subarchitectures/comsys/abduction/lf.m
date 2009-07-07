% $Id$

:- module lf.

:- interface.

:- import_module string.

:- type index == string.
:- type sort == string.
:- type relation == string.
:- type proposition == string.

:- type id
	--->	id(index, sort).

:- type lf
	--->	'@'(id, lf)
	;	i(id)
	;	r(relation, lf)
	;	p(proposition)
	;	'^'(lf, lf)
	.

:- implementation.
