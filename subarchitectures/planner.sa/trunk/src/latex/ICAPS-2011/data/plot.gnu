set boxwidth 0.9 relative
set border 11
set style histogram rowstacked
set style data histograms
set style fill pattern 1.0 border lt -1
set key at 5.8,75
#set bars fullwidth
#set key noinvert box
set yrange [0:*]
set y2label "success ratio" offset -2,0
set y2tics 
#set xlabel "Setting"
unset xlabel
set y2range [0:1]
#set tics scale 0.0
set ytics nomirror
set xtics nomirror rotate by 45 offset -1.5,-1.3 font "Times-Roman,12"
set xtics ("cp" 0.0, "dt 20" 1.0, "dt 50" 2.0, "dt 100" 3.0, "dt 200" 4.0, "cp" 6.0, "dt 20" 7.0, "dt 50" 8.0, "dt 100" 9.0, "dt 200" 10.0, "cp" 12.0, "dt 20" 13.0, "dt 50" 14.0, "dt 100" 15.0, "dt 200" 16.0)


#set terminal epslatex size 8cm, cm
set terminal postscript eps size 8.7cm, 3.0cm "Times-Roman"
#set multiplot layout 1, 3

set ylabel "plan costs" offset 1,0

set output "dora1-quality.eps"
set title "Problem 1 with sensing prob. 0.9/0.65/0.4" offset 0,-0.5
plot newhistogram  fs pattern 4 lt 1, 'dora1-easy.time' using 5 t "plan costs", '' using ($4*50.0/24) t "success ratio" axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora1-med.time' using 5 notitle, '' using ($0+6):($4*50.0/24) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora1-hard.time' using 5 notitle, '' using ($0+12):($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3
unset key

set output "dora2-quality.eps"
set title "Problem 2 with sensing prob. 0.9/0.65/0.4" 
plot newhistogram  fs pattern 4 lt 1, 'dora2-easy.time' using 5 t "plan costs", '' using ($4*50.0/30) t "success ratio" axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora2-med.time' using 5 notitle, '' using ($0+6):($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora2-hard.time' using 5 notitle, '' using ($0+12):($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3

set output "dora3-quality.eps"
set title "Problem 3 with sensing prob. 0.9/0.65/0.4" 
plot newhistogram  fs pattern 4 lt 1, 'dora3-easy.time' using 5 t "plan costs", '' using ($4*50.0/30) t "success ratio" axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora3-med.time' using 5 notitle, '' using ($0+6):($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora3-hard.time' using 5 notitle, '' using ($0+12):($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3

set output "dora4-quality.eps"
set title "Problem 4 with sensing prob. 0.9/0.65/0.4" 
plot newhistogram  fs pattern 4 lt 1, 'dora4-easy.time' using 5 t "plan costs", '' using ($4*50.0/41) t "success ratio" axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora4-med.time' using 5 notitle, '' using ($0+6):($4*50.0/41) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora4-hard.time' using 5 notitle, '' using ($0+12):($4*50.0/41) notitle axes x1y2 with lines lt 1 lw 3

set output "dora-cat-quality.eps"
set xtics ("dt 20" 0.0, "dt 50" 1.0, "dt 100" 2.0, "dt 200" 3.0, "dt 20" 5.0, "dt 50" 6.0, "dt 100" 7.0, "dt 200" 8.0, "dt 20" 10.0, "dt 50" 11.0, "dt 100" 12.0, "dt 200" 13.0, "dt 20" 15.0, "dt 50" 16.0, "dt 100" 17.0, "dt 200" 18.0)
set title "Indirect sensing"
plot newhistogram  fs pattern 4 lt 1, 'dora1-cat.time' using 5 t "plan costs", '' using ($4*50.0/19) t "success ratio" axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora3-cat.time' using 5 notitle, '' using ($0+5):($4*50.0/35) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora2-cat.time' using 5 notitle, '' using ($0+10):($4*50.0/19) notitle axes x1y2 with lines lt 1 lw 3, \
     newhistogram  fs pattern 4 lt 1, 'dora4-cat.time' using 5 notitle, '' using ($0+15):($4*50.0/29) notitle axes x1y2 with lines lt 1 lw 3

set output "dora56-quality.eps"
set multiplot
unset y2label
unset y2tics 
set xtics ("cp" 0.0, "dt 20" 1.0, "dt 50" 2.0, "dt 100" 3.0, "dt 200" 4.0)

set origin 0, 0
set size 0.45, 1.0
set title "Problem 5" 
plot 'dora5.time' using 5 t "plan costs" fs pattern 4 lt 1, '' using ($4*50.0/34) t "success ratio" axes x1y2 with lines lt 1 lw 3

unset ylabel
set y2label "success ratio" offset -2,0
set y2tics 

set origin 0.45, 0
set size 0.55, 1.0
#set output "dora6-quality.eps"
set title "Problem 6" 
plot 'dora6.time' using 5 notitle fs pattern 4 lt 1, '' using ($4*50.0/30) notitle axes x1y2 with lines lt 1 lw 3
unset multiplot

set origin 0, 0
set size 1,1


set xtics ("cp" 0.0, "dt 20" 1.0, "dt 50" 2.0, "dt 100" 3.0, "dt 200" 4.0, "cp" 6.0, "dt 20" 7.0, "dt 50" 8.0, "dt 100" 9.0, "dt 200" 10.0, "cp" 12.0, "dt 20" 13.0, "dt 50" 14.0, "dt 100" 15.0, "dt 200" 16.0)

unset y2label
unset y2tics 
set terminal postscript eps size 7.5cm, 3.0cm "Times-Roman"

set key at 8,75
set ylabel "time [sec]" offset 1,0
set ytics mirror

set output "dora1-time.eps"
set title "Problem instance 1"
plot newhistogram  fs pattern 1 lt 1, 'dora1-easy.time' using 2 t "continual planner", '' using 3  t "dt planner", \
     newhistogram  fs pattern 1 lt 1, 'dora1-med.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora1-hard.time' using 2 notitle, '' using 3  notitle

unset key

set title "Problem instance 2"
set output "dora2-time.eps"
plot newhistogram  fs pattern 1 lt 1, 'dora2-easy.time' using 2 t "continual planner", '' using 3  t "dt planner", \
     newhistogram  fs pattern 1 lt 1, 'dora2-med.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora2-hard.time' using 2 notitle, '' using 3  notitle

set title "Problem instance 3"
set output "dora3-time.eps"
plot newhistogram  fs pattern 1 lt 1, 'dora3-easy.time' using 2 t "continual planner", '' using 3  t "dt planner", \
     newhistogram  fs pattern 1 lt 1, 'dora3-med.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora3-hard.time' using 2 notitle, '' using 3  notitle

set title "Problem instance 4"
set output "dora4-time.eps"
plot newhistogram  fs pattern 1 lt 1, 'dora4-easy.time' using 2 t "continual planner", '' using 3  t "dt planner", \
     newhistogram  fs pattern 1 lt 1, 'dora4-med.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora4-hard.time' using 2 notitle, '' using 3  notitle

set output "dora-cat-time.eps"
set xtics ("dt 20" 0.0, "dt 50" 1.0, "dt 100" 2.0, "dt 200" 3.0, "dt 20" 5.0, "dt 50" 6.0, "dt 100" 7.0, "dt 200" 8.0, "dt 20" 10.0, "dt 50" 11.0, "dt 100" 12.0, "dt 200" 13.0, "dt 20" 15.0, "dt 50" 16.0, "dt 100" 17.0, "dt 200" 18.0)
set title "Indirect sensing"
plot newhistogram  fs pattern 1 lt 1, 'dora1-cat.time' using 2 t "continual planner", '' using 3  t "dt planner", \
     newhistogram  fs pattern 1 lt 1, 'dora3-cat.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora2-cat.time' using 2 notitle, '' using 3  notitle, \
     newhistogram  fs pattern 1 lt 1, 'dora4-cat.time' using 2 notitle, '' using 3  notitle


set output "dora56-time.eps"
set multiplot
set title "Problem 5 "
set xtics ("cp" 0.0, "dt 20" 1.0, "dt 50" 2.0, "dt 100" 3.0, "dt 200" 4.0)
set origin 0, 0
set size 0.45, 1.0
plot 'dora5.time' using 2 t "continual planner" fs pattern 1 lt 1, '' using 3  t "dt planner"

set origin 0.55, 0
set size 0.45, 1.0
set title "Problem 6 "
#set output "dora6-time.eps"
plot 'dora6.time' using 2 notitle fs pattern 1 lt 1, '' using 3  notitle
     # newhistogram  fs pattern 1 lt 1, 'dora1-hard.time' using 2 notitle, '' using 3  notitle
unset multiplot



#     '' using ($4*2.0833333333333335) t "success ratio" axes x1y2 with lines lw 3

# set lmargin 0.5
# unset ylabel
# unset ytics
# set key off

# # set output "dora1-medium.tex"
# set title "Dora 1 - medium" offset 0,-1
# plot 'dora1-med.time' using 2 t "continual planner", '' using 3  t "dt planner", \
#      '' using ($4*2.0833333333333335) t "success ratio" axes x1y2 with lines lw 3

# set rmargin 2.0
# set y2label "success ratio" offset -4,0
# set y2tics offset -0.5,0

# # set output "dora1-hard.tex"
# set title "Dora 1 - hard" offset 0,-1
# plot 'dora1-hard.time' using 2 t "continual planner", '' using 3  t "dt planner", \
#      '' using ($4*1.6666666666666667) t "success ratio" axes x1y2 with lines lw 3


# # set output "dora2-easy.tex"
# # set title "Dora 2 - easy"
# # plot 'dora2-easy.time' using 2 t "continual planner", '' using 3  t "dt planner"
# # replot '' using ($4*1.6666666666666667) t "success ratio" axes x1y2 with lines lw 3

# # set output "dora2-medium.tex"
# # set title "Dora 2 - medium"
# # plot 'dora2-med.time' using 2 t "continual planner", '' using 3  t "dt planner"
# # replot '' using ($4*1.6666666666666667) t "success ratio" axes x1y2 with lines lw 3

# # set output "dora2-hard.tex"
# # set title "Dora 2 - hard"
# # plot 'dora2-hard.time' using 2 t "continual planner", '' using 3  t "dt planner"
# # replot '' using ($4*1.6666666666666667) t "success ratio" axes x1y2 with lines lw 3

# unset multiplot