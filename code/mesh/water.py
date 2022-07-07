
import random
import numpy as np

lis = [i / 25 for i in range(26)]
print(lis)

#+-0.015

vlist = []

for k in lis:
    for j in lis:
        x = k
        y = j
        if 0.01 < k < 0.99:
            x += random.uniform(-0.015, 0.015)
        if 0.01 < j < 0.99:
            y += random.uniform(-0.015, 0.015)
        li = [str(x) , str(np.random.normal(loc=0, scale=0.01)),str(y)]
        vlist.append(li)

#一行26个点，26行

flist = []

for i in range(25):
    for j in range(25):
        t = i * 26 + j + 1
        flist.append([str(t), str(t + 26), str(t + 27)])
        flist.append([str(t), str(t + 1), str(t + 27)])

with open("water.obj", "w+") as f:
    for l in vlist:
        v1 = ['v']
        v1.extend(l)
        x1 = " ".join(v1)
        x1 += "\n"
        f.write(x1)
    for l in flist:
        v1 = ['f']
        v1.extend(l)
        x1 = " ".join(v1)
        x1 += "\n"
        f.write(x1)
