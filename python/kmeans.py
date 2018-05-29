#!/usr/bin/env python

from sklearn.cluster import KMeans
import class_analyse_tools as tools
import sys
import random


if len(sys.argv) != 2 :
  print("Usage : \narg1 : file containing data from a babbling")
  sys.exit(1)


sum_perc = 0
data, labels = tools.load_dataset(sys.argv[1])


pos = 0
neg = 0

for label in labels :
  if(label == 1) :
    pos+=1
  elif (label == 0) :
    neg+=1


print(str(pos) + " positives samples and")
print(str(neg) + " negatives samples from " + str(len(data)))



classifier = KMeans(n_clusters=2,random_state=0)

print(classifier.fit(data))

good_class = 0
for i in range(0, len(data)) :
  pred = classifier.predict(data[i])
  if(pred == labels[i]) :
    good_class+=1

percentage = float(good_class)/float(len(data))*100.
sum_perc+=percentage

print(str(percentage) + " % good classification")

