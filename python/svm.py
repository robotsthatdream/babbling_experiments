#!/usr/bin/env python

from sklearn import svm
import class_analyse_tools as tools
import sys
import random

random.seed()

if len(sys.argv) != 4 :
  print("Usage : \narg1 : file containing data from a babbling")
  print("arg2 : number of samples in training")
  print("arg3 : number of evaluation")
  sys.exit(1)


sum_perc = 0

for i in range(0,int(sys.argv[3])):
  data, labels = tools.load_dataset(sys.argv[1])

  test_data = data
  test_labels = labels
  train_data = list()
  train_labels = list()
  np = 0
  nn = 0
  length = len(test_data)
  pos = 0
  neg = 0

  for label in labels :
    if(label == 1) :
      pos+=1
    elif (label == 0) :
      neg+=1


  print(str(pos) + " positives samples and")
  print(str(neg) + " negatives samples from " + str(len(test_data)))

  it = -1
  nb_s = 0
  while(nb_s < int(sys.argv[2])  and it < length) :
    it+=1
    rnb = random.randint(0,len(test_data)-1)
    if(labels[rnb] == 1) :
      if(np > int(sys.argv[2])/2.) :
        continue
      np+=1
    elif(labels[rnb] == 0) :
      if(nn > int(sys.argv[2])/2.) :
        continue
      nn+=1
    train_data.append(test_data[rnb])
    test_data.pop(rnb)
    train_labels.append(test_labels[rnb])
    test_labels.pop(rnb)
    nb_s+=1

  if(it == length):
    if(np < int(sys.argv[2])/2.) :
      print("not enough positive data")
    if(nn < int(sys.argv[2])/2.) :
      nn < int(sys.argv[2])/2.
    sys.exit(1)

  classifier = svm.SVC()

  print(classifier.fit(train_data,train_labels))

  good_class = 0
  for i in range(0, len(test_data)) :
    pred = classifier.predict(test_data[i])
    if(pred == test_labels[i]) :
      good_class+=1

  percentage = float(good_class)/float(len(data))*100.
  sum_perc+=percentage

  print(str(percentage) + " % good classification")

sum_perc/=float(sys.argv[3])
print(str(sum_perc)+" % average good classification")