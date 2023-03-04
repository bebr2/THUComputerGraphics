
lis = []

with open("mybunny.obj") as f:
    lines = f.readlines()

    for line in lines:
        if line[0] != 'f':
            lis.append(line)
        else:
            newline = line.replace('//', '/0/')
            lis.append(newline)

with open("mybunny2.obj", "w+") as f:
    for co in lis:
        f.write(co)

