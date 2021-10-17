#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import os
import time

os.system('python train.py --gpu 0 --tensorboard-dir "" --epoch 50 --batch-size 1 --step 48 --snapshot-path \'./snapshots1\' csv train1.txt classes --val-annotations test1.txt > output1.txt')

time.sleep(2)

os.system('python train.py --gpu 0 --tensorboard-dir "" --epoch 50 --batch-size 1 --step 48 --snapshot-path \'./snapshots2\' csv train2.txt classes --val-annotations test2.txt > output2.txt')

time.sleep(2)

os.system('python train.py --gpu 0 --tensorboard-dir "" --epoch 50 --batch-size 1 --step 48 --snapshot-path \'./snapshots3\' csv train3.txt classes --val-annotations test3.txt > output3.txt')

time.sleep(2)

os.system('python train.py --gpu 0 --tensorboard-dir "" --epoch 50 --batch-size 1 --step 48 --snapshot-path \'./snapshots4\' csv train4.txt classes --val-annotations test4.txt > output4.txt')

time.sleep(2)

os.system('python train.py --gpu 0 --tensorboard-dir "" --epoch 50 --batch-size 1 --step 48 --snapshot-path \'./snapshots5\' csv train5.txt classes --val-annotations test5.txt > output5.txt')
