from posixpath import split

from matplotlib.pyplot import vlines


lis = []
jia_lis = []
v_lis = []
up_lis = []

with open("Viking.obj") as f:
    lines = f.readlines()
    for line in lines:
        if line[0] != 'f':
            lis.append(line)
        else:
            cont = line.replace('\n', '').split(' ')
            v = []
            for i in range(1, len(cont)):
                v.append(int(cont[i].split('/')[0]))
            v_lis.append(v)

            if len(cont) == 4 or len(cont) == 6:
                lis.append(line)
            else:
                jia_lis.append(cont)
                up_lis.append(len(v_lis) - 1)

jjj = 0
for ii in up_lis:
    v_num = len(v_lis[ii])
    v_disable = []
    v_able = []
    for i in range(0, len(v_lis)):
        if i == ii:
            continue
        for j in range(1, v_num):
            if v_lis[ii][0] in v_lis[i]:
                if v_lis[ii][j] in v_lis[i]:
                    if not j in v_disable:
                        v_disable.append(j)
                        if(len(v_disable) >= 2):
                            break
    for j in range(0, v_num):
        if not j in v_disable:
            v_able.append(j)
    print(v_able)
    print(v_disable)
    x1 = ['f']
    x2 = ['f']

    for i in range(2):
        x1.append(jia_lis[jjj][v_able[i] + 1])
        x2.append(jia_lis[jjj][v_able[i] + 1])
    x1.append(jia_lis[jjj][v_disable[0] + 1])
    x2.append(jia_lis[jjj][v_disable[1] + 1])
    x1 = " ".join(x1)
    if not x1.endswith('\n'):
        x1 += "\n"
    x2 = " ".join(x2)
    if not x2.endswith('\n'):
        x2 += "\n"
    lis.append(x1)
    lis.append(x2)
    jjj += 1




with open("Viking2.obj", "w+") as f:
    for co in lis:
        f.write(co)

print("wancj")
