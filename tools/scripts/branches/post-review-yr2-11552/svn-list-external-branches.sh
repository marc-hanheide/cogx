#!/bin/sh

# make sure that every repository that has a trunk has branches/
#svn pg svn:externals . | grep '^\^.*/trunk' | tr "\t" " " | tr -s " " | cut -f1 -d" " | sed "s@\(.*\)/trunk@\1/branches/@" | xargs -n1 svn mkdir -m "added"


if [ -z "$1" ]; then
	echo "give name for new branch!" >&2
	exit 1
fi


EXT_TRUNKS=`svn pg svn:externals . | grep '^\^.*/trunk' | tr "\t" " " | tr -s " " | tr " " "@"`

echo $EXT_TRUNKS

for l in $EXT_TRUNKS; do 
	repRoot=`echo $l|cut -f1 -d@| sed "s@\(.*\)/trunk@\1@"`
	repTrunk=`echo $l|cut -f1 -d@`
	wcPath=`echo $l|cut -f2 -d@`
	repBranches="$repRoot/branches"

	# get trunk revision:

	svn ls $repBranches | grep $1 | tail -n1 | sed "s@\(.*\)@$repBranches/\1 $wcPath@" | tr  " " "\t" 

done
