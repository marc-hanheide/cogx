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
	repBranches="$repRoot/branches"

	# get trunk revision:
	trunkRev=`svn info "$repTrunk" | grep Revision: | cut -f2 -d: | tr -d " "`
	newBranch="$repBranches/$1-$trunkRev"
	echo "branch $repTrunk (rev: $trunkRev) into $newBranch"
	svn cp -m "branched from $repTrunk (rev: $trunkRev) into $newBranch" $repTrunk $newBranch
done
