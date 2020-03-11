In the phase 1, I use grid sample method to divide the whole scene into 34*34 grids, and then do the ramdom sampling in these grids individually. I've also tested the mid-point method but I found it's not very suitable in this map. 

In the phase 2, I just use the A star algorithm to search the solution and build a dict to record the child-father nodes. If there is a way to goal, we can use the record dict to trace back and plot the path. 

Sometimes, the roadmap may have a gap between some areas, do the generating again may solve it. 