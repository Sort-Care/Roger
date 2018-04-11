set xlabel "Time"
set ylabel "Errors"
plot "./simu.dat" u 1:2 t 'Eye Error' w linespoints,\
	 "./simu.dat" u 1:3 t 'Shoulder Error' w linespoints,\
	 "./simu.dat" u 1:4 t 'Elbow Error' w linespoints,\
	 "./simu.dat" u 1:5 t 'Trans Error' w linespoints,\
	 "./simu.dat" u 1:6 t 'Theta Error' w linespoints
