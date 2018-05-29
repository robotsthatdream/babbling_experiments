#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import class_analyse_tools as tools
import scipy.stats as stats
import math

if __name__ == '__main__':

	iteration = list()
	nbr_pos = list()
	nbr_neg = list()

	diff = list()

	path = sys.argv[1];

	iteration, nbr_pos, nbr_neg = tools.load_pos_neg_data(path)

	iteration, tabs = tools.sort_data(iteration, nbr_pos, nbr_neg)


	nbr_pos = tabs[0]
	nbr_neg = tabs[1]

	log_iter = list()
	for it in iteration :
		log_iter.append(math.log(it))

	log_nbr_pos = list()
	for n in nbr_pos:
		if(n > 0):
			log_nbr_pos.append(math.log(n))
		else :
			log_nbr_pos.append(0)

	slope, intercept, r_value, p_value, std_err = stats.linregress(log_iter,nbr_neg)

	print("correlation between nbr_neg and log")
	print("	correlation coefficent: " + str(r_value**2))
	print("	standard error : " + str(std_err))
	print("	p_value : " + str(p_value))


	slope, intercept, r_value, p_value, std_err = stats.linregress(iteration,log_nbr_pos)

	print("correlation between nbr_pos and exp")
	print("	correlation coefficent: " + str(r_value**2))
	print("	standard error : " + str(std_err))
	print("	p_value : " + str(p_value))




	diff = tools.absolute_difference(nbr_pos,nbr_neg)

	plt.plot(iteration,nbr_neg,'r--',label='number of negative samples')
	plt.plot(iteration,nbr_pos,'g--',label='number of positive samples')
	plt.plot(iteration,diff,'b-',label='difference between number of negative and positive samples')

	plt.legend()

	plt.show()