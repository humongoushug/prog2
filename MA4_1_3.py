
"""
Solutions to module 4
Review date:
"""

student = "Hugo Sundmark"
reviewer = ""

import math as m
import random as r
from time import perf_counter as pc
from concurrent.futures import ProcessPoolExecutor

def sphere_volume(n, d):
    in_sphere = 0
    for i in range(n):
        lst = [r.uniform(-1,1) for ii in range(d)]

        if(sum(map(lambda x:x**2, lst))) <=1:
            in_sphere +=1
    aprox = (2**d) * in_sphere/n # punkter i / totala antalet * 2^d, percis som 1.1 men med hyper cube och hyper sphere

    return aprox 

def hypersphere_exact(n,d):
    return (m.pi ** (d / 2)) / m.gamma((d / 2) + 1)

    

# parallel code - parallelize for loop
def sphere_volume_parallel1(n,d,np):
    #using multiprocessor to perform 10 iterations of volume function 
    with ProcessPoolExecutor(max_workers=np) as executor:#np är max antsl samtidiga processer
        f = [executor.submit(sphere_volume, n, d) for _ in range(10)]
        results = [future.result() for future in f]
        avg_volume = sum(results) / len(results)
        print(avg_volume)
    return avg_volume

def sphere_volume_parallel2(n,d,np):
    points_per = n // np
    
    with ProcessPoolExecutor(max_workers=np) as executor:
        futures = [executor.submit(sphere_volume, points_per, d) for _ in range(np)]
        partial_volumes = [future.result() for future in futures]
    
    avg_volume = sum(partial_volumes) / len(partial_volumes)
    print(avg_volume)
    return avg_volume

def main():
    # part 1 -- parallelization of a for loop among 10 processes 
    n = 100000
    d = 11
    np = 10

    start = pc()
    for y in range (10):
        sphere_volume(n,d)
    stop = pc()
    print('sequential time: ')
    print(stop-start)

    start = pc()
    sphere_volume_parallel1(n,d,np)
    stop = pc()
    print(stop-start)

    # start = pc()
    # sphere_volume_parallel2()
    # stop = pc()
    # print('sequential time: ')
    # print(stop-start)


if __name__ == '__main__':
	main()
