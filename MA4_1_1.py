
"""
Solutions to module 4
Review date:
"""

student = "Hugo Sundmark"
reviewer = ""


import random as r
import matplotlib.pyplot as plt 

def approximate_pi(n):
    # Write your code here
    x_in, y_in, x_out, y_out = [], [], [], []
    nc = 0
    for i in range(n):
        x = r.uniform(-1,1)
        y = r.uniform(-1,1)
        if x**2 + y**2 <=1:
            x_in.append(x)
            y_in.append(y)
            nc +=1
        else:
            x_out.append(x)
            y_out.append(y)

        pi = 4 * nc / n
    #plt.figure(figsize=(7,7))
    plt.scatter(x_in, y_in, color='red', s=1, label='Inside')
    plt.scatter(x_out, y_out, color='blue', s=1, label='Outside ')
    plt.title(f'Pi  n={n}: π : {pi}')
    plt.legend()
    #plt.savefig(f'pi{n}.png')
    #plt.show()

    return pi
    
def main():
    dots = [1000, 10000, 100000]
    for n in dots:
        approximate_pi(n)

if __name__ == '__main__':
	main()
