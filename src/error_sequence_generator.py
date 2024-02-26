import numpy as np
import random


error_rate=0.30

sequence=[]
x=[1]*6+[0]*12
for cnt in range(2):
    random.shuffle(x)
    sequence+=x
print(sequence)

error_set=[]
error_sequence=[]

x=list(range(1,7))
for i in range(3):
    random.shuffle(x)
    error_set+=x
for i in range(len(sequence)):
    if sequence[i]==1:
        error_sequence+=[error_set.pop(0)]
    else:
        error_sequence+=[0]
print(len(error_sequence))
print(error_sequence)



