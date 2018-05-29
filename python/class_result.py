#!/usr/bin/env python

import sys
import os
import matplotlib.pyplot as plt
import class_analyse_tools as tools

def average_vector(vects) :
    min_size = len(vects[0])

    for vect in vects :
        if(len(vect) < min_size):
            min_size = len(vect)

    aver_vect = list(0 for i in range(0,min_size))
    min_vect = list(vects[0][i] for i in range(0,min_size))
    max_vect = list(vects[0][i] for i in range(0,min_size))

    for vect in vects :
        for i in range(0,min_size) :
            if(vect[i] < min_vect[i]):
                min_vect[i] = vect[i]
            if(vect[i] > max_vect[i]):
                max_vect[i] = vect[i]
            aver_vect[i] += vect[i]

    for i in range(0,len(aver_vect)):
        aver_vect[i] = float(aver_vect[i])/float(len(vects))

    return aver_vect, min_vect, max_vect

iteration_perf = list()
iteration_perf_2 = list()
iteration_neg_pos = list()
perfs = list()
perfs_rand = list()
perfs_2 = list()
precisions = list()
recalls = list()
accuracys = list()
# perfs_rand_2= list()
nbr_pos_vect = list()
nbr_neg_vect = list()

if len(sys.argv) != 4 :
    print("Usage : \narg1 : archive path")
    print("arg2 : name of file with the scores")
    print("arg3 : number of iteration") 
    sys.exit(1)




for arch_exp in os.listdir(sys.argv[1]) :
    archive_folder = sys.argv[1] + arch_exp + "/"

    iteration_perf, perf, perf_rand, precision, recall, accuracy \
     = tools.load_pra(archive_folder + sys.argv[2])
    iteration_perf, tabs = tools.sort_data(iteration_perf,precision,recall,accuracy)

    # perf = tabs[0]
    # perf_rand = tabs[1]
    precision = tabs[0]
    recall = tabs[1]
    accuracy = tabs[2]
    
    # perf = tools.performance(perf_rand,list(1 for i in perf),perf)

    # iteration_perf_2, perf_2, perf_rand_2 = tools.load_class_eval(archive_folder + "classifier_eval_2.yml")
    # iteration_perf_2, tabs = tools.sort_data(iteration_perf_2,perf_2,perf_rand_2)

    # perf_2 = tabs[0]
    # perf_rand_2 = tabs[1]

    # perf_2 = tools.performance(perf_rand_2,list(1 for i in perf_2),perf_2)

    # iteration_neg_pos, nbr_pos, nbr_neg = tools.load_pos_neg_data(archive_folder)

    # iteration_neg_pos, tabs = tools.sort_data(iteration_neg_pos,nbr_pos,nbr_neg)
    
    # nbr_pos = tabs[0]
    # nbr_neg = tabs[1]

    # for i in range(1,len(perf)) :
    #     perf[i-1] = (perf[i] + perf[i-1])/2.
    #     perf_rand[i-1] = (perf_rand[i] + perf_rand[i-1])/2.

    # for i in range(1,len(perf_2)) :
    #   perf_2[i-1] = (perf_2[i] + perf_2[i-1])/2.
    #   perf_rand_2[i-1] = (perf_rand_2[i] + perf_rand_2[i-1])/2.

    # perfs.append(perf)
    # perfs_rand.append(perf_rand)
    precisions.append(precision)
    recalls.append(recall)
    accuracys.append(accuracy)

    # perfs_2.append(perf_2)
    # perfs_rand_2.append(perf_rand_2)

    # nbr_pos_vect.append(nbr_pos)
    # nbr_neg_vect.append(nbr_neg)

# aver_perf, min_perf, max_perf = average_vector(perfs)
# aver_perf_rand, min_perf_rand, max_perf_rand = average_vector(perfs_rand)
aver_prec, min_prec, max_prec = average_vector(precisions)
aver_rec, min_rec, max_rec = average_vector(recalls)
aver_acc, min_acc, max_acc = average_vector(accuracys)


# print(aver_perf)

# aver_perf_2, min_perf_2, max_perf_2 = average_vector(perfs_2)
# aver_perf_rand_2, min_perf_rand_2, max_perf_rand_2 = average_vector(perfs_rand_2)

# aver_pos, min_pos, max_pos = average_vector(nbr_pos_vect)
# aver_neg, min_neg, max_neg = average_vector(nbr_neg_vect)
# diff_neg_pos = tools.absolute_difference(aver_pos,aver_neg)
# min_diff_neg_pos = tools.absolute_difference(min_pos,min_neg)
# max_diff_neg_pos = tools.absolute_difference(max_pos,max_neg)

# final_perf = tools.performance(aver_perf_rand,list(1 for i in aver_perf),aver_perf)
# min_final_perf = tools.performance(min_perf_rand,list(1 for i in min_perf),min_perf)
# max_final_perf = tools.performance(max_perf_rand,list(1 for i in max_perf),max_perf)

# final_perf_2 = tools.performance(aver_perf_rand_2,list(1 for i in aver_perf_2),aver_perf_2)
# min_final_perf_2 = tools.performance(min_perf_rand_2,list(1 for i in min_perf_2),min_perf_2)
# max_final_perf_2 = tools.performance(max_perf_rand_2,list(1 for i in max_perf_2),max_perf_2)


iteration_perf = iteration_perf[:len(aver_prec)]

# iteration_perf_2 = iteration_perf[:len(aver_perf_2)]

# iteration_neg_pos = iteration_neg_pos[:len(diff_neg_pos)]

# r_value_pos, p_value_pos, std_err_pos, r_value_neg, p_value_neg, std_err_neg = tools.neg_pos_fitting(iteration_neg_pos,aver_pos,aver_neg)

# text = "correlation between number_of_positives curves and exponentiel function\n" \
# +"  correlation coefficent: " + str(r_value_pos) + "\n" \
# +"  standard error : " + str(std_err_pos) + "\n" \
# +"  p_value : " + str(p_value_pos) + "\n"
# text_2 = "correlation between number of negatives curve and logarithm function" + "\n" \
# +"  correlation coefficent: " + str(r_value_neg) + "\n" \
# +"  standard error : " + str(std_err_neg) + "\n" \
# +"  p_value : " + str(p_value_neg) + "\n"

# print(text)
# print(text_2)

fig, ax1 = plt.subplots(1,sharex=True)

# ax1.plot(iteration_perf,aver_perf,'k-', linewidth=2,label='performance computed on same set-up as training')
ax1.plot(iteration_perf,aver_prec,'k-', linewidth=2,label='precision')
ax1.plot(iteration_perf,aver_rec,'r-', linewidth=2,label='recall')
ax1.plot(iteration_perf,aver_acc,'g-', linewidth=2,label='accuracy')

# ax1.fill_between(iteration_perf,final_perf,facecolor='black',alpha=0.5)
# ax1.plot(iteration_perf,min_perf,'k--',iteration_perf,max_perf,'k--')
# ax1.fill_between(iteration_perf,min_perf,aver_perf,facecolor='black',alpha=0.2)
# ax1.fill_between(iteration_perf,max_perf,aver_perf,facecolor='black',alpha=0.2)

ax1.plot(iteration_perf,min_prec,'k--',iteration_perf,max_prec,'k--')
ax1.fill_between(iteration_perf,min_prec,aver_prec,facecolor='black',alpha=0.2)
ax1.fill_between(iteration_perf,max_prec,aver_prec,facecolor='black',alpha=0.2)

ax1.plot(iteration_perf,min_rec,'r--',iteration_perf,max_rec,'r--')
ax1.fill_between(iteration_perf,min_rec,aver_rec,facecolor='red',alpha=0.2)
ax1.fill_between(iteration_perf,max_rec,aver_rec,facecolor='red',alpha=0.2)

ax1.plot(iteration_perf,min_acc,'g--',iteration_perf,max_acc,'g--')
ax1.fill_between(iteration_perf,min_acc,aver_acc,facecolor='green',alpha=0.2)
ax1.fill_between(iteration_perf,max_acc,aver_acc,facecolor='green',alpha=0.2)

# ax1.plot(iteration_perf_2,aver_perf_2,'b-', linewidth=2,label='performance computed on different set-up')
# ax1.plot(iteration_perf_2,min_perf_2,'b--',iteration_perf_2,max_perf_2,'b--')
# ax1.fill_between(iteration_perf_2,min_perf_2,aver_perf_2,facecolor='blue',alpha=0.2)
# ax1.fill_between(iteration_perf_2,max_perf_2,aver_perf_2,facecolor='blue',alpha=0.2)


# ax1.legend(bbox_to_anchor=(1., 1.), loc=2, borderaxespad=0.,fontsize=25)
# ax1.ylim(-1,1)
ax1.set_ylabel('performance score',fontsize=25)
ax1.set_xlabel('number of iteration',fontsize=25)

ax1.tick_params(labelsize=20)

ax1.set_xlim([0,int(sys.argv[3])])
ax1.set_ylim([0,1])

# ax2.plot(iteration_neg_pos,aver_pos,'g--',label='number of positive samples',linewidth=2)
# ax2.plot(iteration_neg_pos,aver_neg,'r--',label='number of negative samples',linewidth=2)
# ax2.plot(iteration_neg_pos,min_pos,'g-',iteration_neg_pos,max_pos,'g-',linewidth=.5)
# ax2.plot(iteration_neg_pos,min_neg,'r-',iteration_neg_pos,max_neg,'r-',linewidth=.5)
# ax2.fill_between(iteration_neg_pos,min_pos,aver_pos,facecolor='green',alpha=.5)
# ax2.fill_between(iteration_neg_pos,max_pos,aver_pos,facecolor='green',alpha=.5)
# ax2.fill_between(iteration_neg_pos,min_neg,aver_neg,facecolor='red',alpha=.5)
# ax2.fill_between(iteration_neg_pos,max_neg,aver_neg,facecolor='red',alpha=.5)

# ax2.plot(iteration_neg_pos,diff_neg_pos,'b-',linewidth=2,label='absolute difference')
# # ax2.plot(iteration_neg_pos,min_diff_neg_pos,'b-',iteration_neg_pos,max_diff_neg_pos,'b-')
# # ax2.fill_between(min_diff_neg_pos,diff_neg_pos,facecolor='blue',alpha=.5)
# # ax2.fill_between(max_diff_neg_pos,diff_neg_pos,facecolor='blue',alpha=.5)

# ax2.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=2, borderaxespad=0.,fontsize=25)



# ax2.set_ylabel('accumulated number of samples',fontsize=25)
# ax2.set_xlabel('number of iteration',fontsize=25)
# ax2.set_aspect('auto')
# ax2.tick_params(labelsize=20)
# ax2.set_xlim([0,150])

# plt.figtext(0.2,0,text,bbox=dict(facecolor='white'))
# plt.figtext(0.7,0,text_2,bbox=dict(facecolor='white'))

plt.show()


