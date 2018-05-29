#!/usr/bin/env python

import sys
import os
import matplotlib.pyplot as plt
import class_analyse_tools as tools



iteration = list()
nbr_pos_vect = list()
nbr_neg_vect = list()

if len(sys.argv) != 4 :
    print("Usage : \narg1 : archive path")
    print("arg2 : name of file with the scores")
    print("arg3 : number of iteration") 
    sys.exit(1)




for arch_exp in os.listdir(sys.argv[1]) :
    archive_folder = sys.argv[1] + arch_exp + "/"

    iteration, nbr_pos, nbr_neg \
     = tools.load_nbr_comp(archive_folder + sys.argv[2])
    iteration, tabs = tools.sort_data(iteration, nbr_pos, nbr_neg)

    nbr_pos = tabs[0]
    nbr_neg = tabs[1]
    
    nbr_pos_vect.append(nbr_pos)
    nbr_neg_vect.append(nbr_neg)


aver_pos, min_pos, max_pos = tools.average_vector(nbr_pos_vect)
aver_neg, min_neg, max_neg = tools.average_vector(nbr_neg_vect)

iteration = iteration[:len(aver_pos)]



fig, ax1 = plt.subplots(1,sharex=True)



ax1.plot(iteration,aver_pos,'g-',label='number of positive components',linewidth=2)
ax1.plot(iteration,aver_neg,'r-',label='number of negative components',linewidth=2)
ax1.plot(iteration,min_pos,'g-',iteration,max_pos,'g-',linewidth=.5)
ax1.plot(iteration,min_neg,'r-',iteration,max_neg,'r-',linewidth=.5)
ax1.fill_between(iteration,min_pos,aver_pos,facecolor='green',alpha=.5)
ax1.fill_between(iteration,max_pos,aver_pos,facecolor='green',alpha=.5)
ax1.fill_between(iteration,min_neg,aver_neg,facecolor='red',alpha=.5)
ax1.fill_between(iteration,max_neg,aver_neg,facecolor='red',alpha=.5)



# ax1.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=2, borderaxespad=0.,fontsize=25)
#


ax1.set_ylabel('number of components',fontsize=25)
ax1.set_xlabel('number of iteration',fontsize=25)
ax1.set_aspect('auto')
ax1.tick_params(labelsize=20)
ax1.set_xlim([0,int(sys.argv[3])])
# ax1.set_ylim([0,int(sys.argv[3])])




plt.show()


