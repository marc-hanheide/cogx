#!/bin/bash

out="optdefault.py"

echo '# This file is auto-generated. Do not edit' > $out

echo 'environment="""' >> $out
cat default_env.txt >> $out
echo '"""' >> $out

echo 'cleanup="""' >> $out
cat default_cleanup.txt >> $out
echo '"""' >> $out

echo 'useroptions="""' >> $out
cat default_user.txt >> $out
echo '"""' >> $out

