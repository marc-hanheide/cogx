#!/bin/bash
pack=golem-trunk
cd mkdiff
ln -sfn ../work/main.d/$pack
diff -ru orig-$pack $pack | grep -v "^Only in" > patch.diff

