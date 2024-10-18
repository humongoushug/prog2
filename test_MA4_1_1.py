# https://docs.python.org/3/library/unittest.html
import unittest

from MA4_1_1 import *
from MemoryAnalyzer import run_test_for_ma4_1


class Test(unittest.TestCase):

    def test_approximate_pi(self):
        self.assertIsInstance(student, str, "Variable 'student' is not set")
        self.assertIsInstance(reviewer, str, "Variable 'reviewer' is not set" )
        self.assertNotEqual(student,'Your name', 'Your name is missing!')
        
        print(f"\nTests the method 'approximate_pi' in MA4 written by {student}. Reviewer: {reviewer}") 

        # test if the approximated pi is within the interval [3.02, 3.25]
        pi_a = approximate_pi(1000)
        self.assertLess(3.02, pi_a)
        self.assertLess(pi_a, 3.5) #### changed from 3.25 to 3.5

        # test if the approximated pi is within the interval [3.10, 3.18]
        pi_a = approximate_pi(10000)
        self.assertLess(3.10, pi_a)
        self.assertLess(pi_a, 3.18)

        # test if the approximated pi is within the interval [3.12, 3.15]
        pi_a = approximate_pi(100000)
        self.assertLess(3.13, pi_a)
        self.assertLess(pi_a, 3.15)

        lst = run_test_for_ma4_1()
        self.assertEqual(lst, sorted(lst))

if __name__ == "__main__":
    unittest.main()
