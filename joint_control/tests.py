#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 13 12:49:06 2020

@author: duc-ubuntu
"""
from collections import deque
import numpy as np

test = deque([0.3, 1.5, 4.2], maxlen=5 + 1)
print (test)
test2 = deque(test, 2)
print(test2)
test3 = deque(np.zeros(25), maxlen=5 + 1)
print(test3)