import sys
sys.path.append('../')
import moteusDriver
import time
import matplotlib.pyplot as plt

moteus1 = moteusDriver.MoteusController(device_id=1, direction=1)
moteus1.stop()

moteus2 = moteusDriver.MoteusController(device_id=2, direction=1)
moteus2.stop()


command = 0
start_time = time.time()


swap = True
t1_real = []
t2_real = []
t1 = []
t2 = []
t = time.time()
dts = []

start_time = time.time()
cur_time = 0
prev_time = 0

while time.time() < t+3:
    cur_time = time.time()-start_time
    dt = cur_time - prev_time
    
    if swap:
        moteus1.set_torque(0.75)
        # moteus2.set_torque(0.5)
        t1_real.append(0.5)
        t2_real.append(0.5)
        
    else:
        moteus1.set_torque(-0.75)
        # moteus2.set_torque(-0.5)
        t1_real.append(-0.5)
        t2_real.append(-0.5)
    
    dts.append(dt)

    
    
    t1.append(moteus1.get_velocity())
    t2.append(moteus2.get_velocity())
    
    prev_time = cur_time
    
    swap = not swap
    
plt.figure(figsize=(20, 4))
# plt.plot(t1, label='t1 measured')
# plt.plot(t2, label='t2 measured')
# plt.plot(t1_real, label='t1 commanded')
# plt.plot(t2_real, label='t2 commanded')

plt.plot(dts, label='t2 commanded')

plt.legend()
plt.savefig('torques.png')

