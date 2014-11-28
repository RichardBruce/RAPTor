# Set input data format
set datafile separator ","

# Set output
set term png size 1920, 1080
set output outfile

# plot column chart
set key autotitle columnheader
set logscale y
set boxwidth 1.0
set style data histogram
set style fill solid border -1
set xtics rotate 90
plot infile using datacolumn: xtic(1) with histogram