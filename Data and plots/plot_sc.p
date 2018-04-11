set xlabel "Time"
set ylabel "Theta Error"
plot "./v3.dat" using 1:2 title "Left Eye" with linespoints,\
   "./v3.dat" using 1:3 title "Right Eye" with linespoints
