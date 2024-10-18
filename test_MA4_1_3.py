# https://docs.python.org/3/library/unittest.html
import unittest

from HighOrderFunctionChecker import check_higher_order_functions
from MA4_1_3 import *


class Test(unittest.TestCase):

    def test_sphere_volume_parallel1(self):
        self.assertIsInstance(student, str, "Variable 'student' is not set")
        self.assertIsInstance(reviewer, str, "Variable 'reviewer' is not set" )
        self.assertNotEqual(student,'Your name', 'Your name is missing!')
        
        print(f"\nTests the method 'sphere_volume' in MA4 written by {student}. Reviewer: {reviewer}") 

        # test PART 1 if the sphere volume is within the interval [3.10, 3.18]
        n = 100000
        d = 11
        np = 8
        start = pc()
        for y in range (np):
            sphere_volume(n,d)
        stop = pc()
        seq = stop - start

        start = pc()
        app_vol = sphere_volume_parallel1(n, d, np)
        print(app_vol)
        stop = pc()
        par = stop - start

        self.assertLess(1.6, app_vol)
        self.assertLess(app_vol, 2.2)
        self.assertLess(par, seq/2.)
        
        # test PART 2 if the sphere volume is within the interval [3.10, 3.18]
        n = 100000
        d = 11
        np = 10
        start = pc()
        sphere_volume(n, d)
        stop = pc()
        seq = stop - start

        start = pc()
        app_vol = sphere_volume_parallel2(n, d, np)
        print(app_vol)
        stop = pc()
        par = stop - start

        self.assertLess(1.6, app_vol)
        self.assertLess(app_vol, 2.2)
        self.assertLess(par, seq)

        
if __name__ == "__main__":
    unittest.main()
