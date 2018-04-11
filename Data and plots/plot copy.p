set xlabel "Time"
set ylabel "Cartesian Error"
plot "./ag.dat" u ($1):($2 - 0.51) t "X Error" w linespoints,\
"./ag.dat" u ($1):($3 - 1.021) t "Y Error" w linespoints
