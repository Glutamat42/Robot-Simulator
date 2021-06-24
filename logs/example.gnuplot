set datafile separator ','

set key invert box center right reverse Left
set xtics nomirror
set ytics nomirror
set border 3

samples(x) = $0 > 9 ? 10 : ($0+1)
avg10(x) = (shift10(x), (back1+back2+back3+back4+back5+back6+back7+back8+back9+back10)/samples($0))
shift10(x) = (back10 = back9, back9 = back8, back8 = back7, back7 = back6, back6 = back5, back5 = back4, back4 = back3, back3 = back2, back2 = back1, back1 = x)

#
# Initialize a running sum
#
init(x) = (back1 = back2 = back3 = back4 = back5 = back6 = back7 = back8 = back9 = back10 = sum = 0)

#
# Plot data, running average and cumulative average
#

datafile = '24-06-2021_13-02-19_q-learning.csv'

set style data linespoints

plot sum = init(0), \
     datafile using 1:2 title 'data' lw 2 lc rgb 'forest-green' with points, \
     '' using 0:(avg10($2)) title "running mean over previous 10 points" pt 7 ps 0.5 lw 1 lc rgb "blue", \
     '' using 0:(sum = sum + $2, sum/($0+1)) title "cumulative mean" pt 1 lw 1 lc rgb "dark-red"
