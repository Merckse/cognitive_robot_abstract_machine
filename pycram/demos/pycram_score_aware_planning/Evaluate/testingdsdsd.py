import time

t1 = time.time()
time.sleep(1.555)
t2 = time.time()
t3 = t2 - t1
print(round(t3,3))