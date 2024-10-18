
"""
Solutions to module 4
Review date:
"""

student = "Hugo Sundmark"
reviewer = ""

import math as m
import random as r

def sphere_volume(n, d):
    in_sphere = 0
    # n is a list of set of coordinates
    # d is the number of dimensions of the sphere 
    for i in range(n):
        lst = [r.uniform(-1,1) for ii in range(d)]

        if(sum(map(lambda x:x**2, lst))) <=1:
            in_sphere +=1
    aprox = (2**d) * in_sphere/n # punkter i / totala antalet * 2^d, percis som 1.1 men med hyper cube och hyper sphere

    return aprox

def hypersphere_exact(n,d):#Vd(1) =(π^(d/2)) / I(d/2 + 1
    true_volume = (m.pi**(d/2)) / (m.gamma((d/2)+1))
    return true_volume
     
def main():
    n = 100000
    d = 2
    sphere_volume(n,d)


if __name__ == '__main__':
	main()
