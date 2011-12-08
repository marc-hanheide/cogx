#!/bin/bash
cd mkdiff
pack=gazebo-0.10.0
ln -sf ../work/main.d/$pack
diff -ru orig-$pack $pack | grep -v "^Only in" > patch.diff

