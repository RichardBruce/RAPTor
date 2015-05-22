# Set input data format
set datafile separator ","

# Set output
set term png size 1920, 1080
set output outfile

# plot column chart
set auto x
set logscale y
set boxwidth 1.0
set style data histogram
set style histogram cluster gap 0
set style fill solid border -1
set xtics rotate 90
plot infile every ::1 using 4: xtic(1) ti col, '' u 5 ti col, '' u 6 ti col, '' u 7 ti col, '' u 8 ti col, '' u 9 ti col, '' u 10 ti col