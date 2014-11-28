#!/usr/bin/gnuplot

# Set input data format
set datafile separator ","

# Set output
#set size 1,2
set term png size 1920, 1080
set output "chart.png"

# plot column chart
set key off
set logscale y
set boxwidth 0.95
set style fill solid
set xtics rotate 90# offset 0, -10
plot filename using 3: xtic(1) with histogram