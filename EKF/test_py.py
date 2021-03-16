global a
a = [1,2]

def update(x,y):
    x[0] = y
    return x

a = update(a,2)
print(a)