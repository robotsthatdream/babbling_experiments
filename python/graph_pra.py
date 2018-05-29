#!/usr/bin/env python

import sys
import os
import matplotlib.pyplot as plt
import class_analyse_tools as tools



iteration = list()
precisions = list()
recalls = list()
accuracys = list()


if len(sys.argv) != 4 :
    print("Usage : \narg1 : archive path")
    print("arg2 : name of file with the scores")
    print("arg3 : number of iteration") 
    sys.exit(1)




for arch_exp in os.listdir(sys.argv[1]) :
    archive_folder = sys.argv[1] + arch_exp + "/"

    iteration, precision, recall, accuracy \
     = tools.load_pra(archive_folder + sys.argv[2])
    iteration, tabs = tools.sort_data(iteration,precision,recall,accuracy)

    precision = tabs[0]
    recall = tabs[1]
    accuracy = tabs[2]
    

    precisions.append(precision)
    recalls.append(recall)
    accuracys.append(accuracy)

aver_prec, min_prec, max_prec = tools.average_vector(precisions)
aver_rec, min_rec, max_rec = tools.average_vector(recalls)
aver_acc, min_acc, max_acc = tools.average_vector(accuracys)




iteration = iteration[:len(aver_prec)]


fig, ax1 = plt.subplots(1,sharex=True)

ax1.plot(iteration,aver_prec,'k-', linewidth=2,label='precision')
ax1.plot(iteration,aver_rec,'r-', linewidth=2,label='recall')
ax1.plot(iteration,aver_acc,'g-', linewidth=2,label='accuracy')


ax1.plot(iteration,min_prec,'k--',iteration,max_prec,'k--')
ax1.fill_between(iteration,min_prec,aver_prec,facecolor='black',alpha=0.2)
ax1.fill_between(iteration,max_prec,aver_prec,facecolor='black',alpha=0.2)

ax1.plot(iteration,min_rec,'r--',iteration,max_rec,'r--')
ax1.fill_between(iteration,min_rec,aver_rec,facecolor='red',alpha=0.2)
ax1.fill_between(iteration,max_rec,aver_rec,facecolor='red',alpha=0.2)

ax1.plot(iteration,min_acc,'g--',iteration,max_acc,'g--')
ax1.fill_between(iteration,min_acc,aver_acc,facecolor='green',alpha=0.2)
ax1.fill_between(iteration,max_acc,aver_acc,facecolor='green',alpha=0.2)


ax1.set_ylabel('performance score',fontsize=25)
ax1.set_xlabel('number of iteration',fontsize=25)

ax1.tick_params(labelsize=20)

ax1.set_xlim([0,int(sys.argv[3])])
ax1.set_ylim([0,1])

plt.show()


