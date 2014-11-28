#!/usr/bin/env bash
gnuplot -e "infile='$1'" -e "outfile='$2'" $RAPTOR_TOOLS/stacked_box_graph.gp