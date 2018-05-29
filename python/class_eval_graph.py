#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import class_analyse_tools as tools


if __name__ == '__main__':

	iteration = list()
	perf_rand = list()
	perf = list()
	perf_rand_sort = list()
	perf_sort = list()
	diff= list() 

	iteration, perf, perf_rand = tools.load_class_eval(sys.argv[1])
	iteration, tabs = tools.sort_data(iteration, perf, perf_rand)
	perf_sort = tabs[0]
	perf_rand_sort = tabs[1]


	perf_expert = list(1 for p in perf_rand_sort)
	diff = tools.performance(perf_rand_sort,perf_expert,perf_sort)

	diff_aver = list()
	diff_aver.append(diff[0])
	for i in range(1,len(diff)):
		diff_aver.append((diff[i-1]+diff[i])/2.)

	# plt.plot(iteration,perf_rand_sort,'o-',iteration,perf_sort,'o-')
	plt.plot(iteration,diff,'o-')
	plt.plot(iteration,diff_aver,'r-')
	plt.fill_between(iteration,diff,0)
	plt.ylim(-1,1)
	plt.show()
