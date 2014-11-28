#!/usr/bin/env bash
gnuplot -e "infile='$1'" -e "outfile='$2'" -e datacolumn=4 $RAPTOR_TOOLS/box_graph.gp