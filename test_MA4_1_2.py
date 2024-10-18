# https://docs.python.org/3/library/unittest.html
import unittest

from HighOrderFunctionChecker import check_higher_order_functions
from MA4_1_2 import *


class Test(unittest.TestCase):

    def test_sphere_volume(self):
        self.assertIsInstance(student, str, "Variable 'student' is not set")
        self.assertIsInstance(reviewer, str, "Variable 'reviewer' is not set" )
        self.assertNotEqual(student,'Your name', 'Your name is missing!')
        
        print(f"\nTests the method 'sphere_volume' in MA4 written by {student}. Reviewer: {reviewer}") 

        # test if the sphere volume is within the interval [3.02, 3.25]
        n = 100000
        d = 2
        app_vol = sphere_volume(n, d)
        self.assertLess(3.13, app_vol)
        self.assertLess(app_vol, 3.16)
        act_vol = hypersphere_exact(n, d)
        self.assertAlmostEqual(act_vol, 3.141592653589793)

        # test if the sphere volume is within the interval [3.10, 3.18]
        d = 11
        app_vol = sphere_volume(n, 11)
        self.assertLess(1.2, app_vol)
        self.assertLess(app_vol, 2.6)
        act_vol = hypersphere_exact(n, d)
        self.assertAlmostEqual(act_vol, 1.8841038793898994)

        file_path = "MA4_1_2.py"  # Replace with your actual file path
        found_higher_order = check_higher_order_functions(file_path)
        self.assertEqual(found_higher_order, True)


if __name__ == "__main__":
    unittest.main()
