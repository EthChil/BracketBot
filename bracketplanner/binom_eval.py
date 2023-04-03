import time
import random
import matplotlib.pyplot as plt

def t2s(z1, z2, zd1, zd2, t):
    return (1/5)*zd2*pow(t,5)+(z1/2-z2/2+zd1/4-zd2/4)*pow(t,4)+(z2-z1-(2/3)*zd1)*pow(t,3)+(1/2)*zd1*pow(t,2)+z1*t

def s2t(z1, z2, zd1, zd2, s, taken):
    curr_time = time.time()
    t0 = 0
    t1 = 1
    while(t1-t0>0.1):
        if s > t2s(z1, z2, zd1, zd2, t0):
            if s > t2s(z1, z2, zd1, zd2, (t1 + t0)/2):
                t0 = (t1 + t0)/2
            else:
                t1 = (t1 + t0)/2
    taken.append(time.time() - curr_time)
    return (t0+t1)/2

ts = []
t1s = []
error = []
taken = []
for i in range(100):
    z1 = random.randint(0,500)
    z2 = random.randint(0,500)
    zd1 = random.randint(0,500)
    zd2 = random.randint(0,500)
    t = random.uniform(0,1)
    s1 = t2s(z1, z2, zd1, zd2, t)
    t1 = s2t(z1, z2, zd1, zd2, s1, taken)
    ts.append(t)
    t1s.append(t1)
    error.append((t1-t)/t)

print("Average Error: ",sum(error)/100)
print("Average Time: ",sum(taken)/100)

fig,ax = plt.subplots()
ax.plot(range(100),ts,label="Actual t")
ax.plot(range(100),t1s,label="Approx t")
ax.plot(range(100),error,label="Error")
ax2=ax.twinx()
ax2.plot(range(100),taken,label="Time Taken", color="red")
fig.legend()


plt.show()
