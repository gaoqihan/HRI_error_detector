import numpy as np
import random


error_dict={
                        0: "No Error",
                        1:"Delayed response", 
                        2:"Random movement",
                        3:"Abnormal suggestions",
                        4:"Moving too slow",
                        5:"Inappropriate placement",
                        6:"Not release",
                        7:"Hesitation",
                        8:"Stutter motion",
                        9:"Freeze in motion",
                        10:"Non-optimal motion path"}
error_rate=0.33333

sequence=[]
x=[1]*10+[0]*20
for cnt in range(3):
    random.shuffle(x)
    sequence+=x
print(sequence)
cnt=0
while cnt<5:
    cnt+=1
    error_set=[]
    error_sequence=[]

    x=list(range(1,11))
    for i in range(3):
        random.shuffle(x)
        error_set+=x
    for i in range(len(sequence)):
        if sequence[i]==1:
            error_sequence+=[error_set.pop(0)]
        else:
            error_sequence+=[0]
    print(error_sequence)



