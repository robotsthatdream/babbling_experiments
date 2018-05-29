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
     = tools.load_pos_neg(archive_folder + sys.argv[2])
    iteration, tabs = tools.sort_data(iteration, nbr_pos, nbr_neg)

    nbr_pos = tabs[0]
    nbr_neg = tabs[1]
    
    nbr_pos_vect.append(nbr_pos)
    nbr_neg_vect.append(nbr_neg)


aver_pos, min_pos, max_pos = tools.average_vector(nbr_pos_vect)
aver_neg, min_neg, max_neg = tools.average_vector(nbr_neg_vect)
diff_neg_pos = tools.absolute_difference(aver_pos,aver_neg)
min_diff_neg_pos = tools.absolute_difference(min_pos,min_neg)
max_diff_neg_pos = tools.absolute_difference(max_pos,max_neg)



iteration = iteration[:len(aver_pos)]


# r_value_pos, p_value_pos, std_err_pos, r_value_neg, p_value_neg, std_err_neg = tools.neg_pos_fitting(iteration,aver_pos,aver_neg)

# text = "correlation between number_of_positives curves and exponentiel function\n" \
# +"  correlation coefficent: " + str(r_value_pos) + "\n" \
# +"  standard error : " + str(std_err_pos) + "\n" \
# +"  p_value : " + str(p_value_pos) + "\n"
# text_2 = "correlation between number of negatives curve and logarithm function" + "\n" \
# +"  correlation coefficent: " + str(r_value_neg) + "\n" \
# +"  standard error : " + str(std_err_neg) + "\n" \
# +"  p_value : " + str(p_value_neg) + "\n"


fig, ax1 = plt.subplots(1,sharex=True)



ax1.plot(iteration,aver_pos,'g--',label='number of positive samples',linewidth=2)
ax1.plot(iteration,aver_neg,'r--',label='number of negative samples',linewidth=2)
ax1.plot(iteration,min_pos,'g-',iteration,max_pos,'g-',linewidth=.5)
ax1.plot(iteration,min_neg,'r-',iteration,max_neg,'r-',linewidth=.5)
ax1.fill_between(iteration,min_pos,aver_pos,facecolor='green',alpha=.5)
ax1.fill_between(iteration,max_pos,aver_pos,facecolor='green',alpha=.5)
ax1.fill_between(iteration,min_neg,aver_neg,facecolor='red',alpha=.5)
ax1.fill_between(iteration,max_neg,aver_neg,facecolor='red',alpha=.5)

ax1.plot(iteration,diff_neg_pos,'b-',linewidth=2,label='absolute difference')
# ax1.plot(iteration,min_diff_neg_pos,'b-',iteration,max_diff_neg_pos,'b-')
# ax1.fill_between(min_diff_neg_pos,diff_neg_pos,facecolor='blue',alpha=.5)
# ax1.fill_between(max_diff_neg_pos,diff_neg_pos,facecolor='blue',alpha=.5)

# ax1.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=2, borderaxespad=0.,fontsize=25)



ax1.set_ylabel('accumulated number of samples',fontsize=25)
ax1.set_xlabel('number of iteration',fontsize=25)
ax1.set_aspect('equal')
ax1.tick_params(labelsize=20)
ax1.set_xlim([0,int(sys.argv[3])])
ax1.set_ylim([0,int(sys.argv[3])])


# plt.figtext(0.2,0,text,bbox=dict(facecolor='white'))
# plt.figtext(0.7,0,text_2,bbox=dict(facecolor='white'))

plt.show()


