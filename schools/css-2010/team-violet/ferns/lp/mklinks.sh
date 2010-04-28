
models="chakakhan heartbreakers james jesusjones"
for m in $models ; do
   ln -s -f ${m}_img000.jpg $m.jpg
   ln -s -f ${m}_img000.jpg.roi $m.jpg.roi
done
