import unittest
import gtools # importing the module we want to test (the test file should be in the same directory as module file)

class TestGtools(): # inherit from unittest.TestCase 
	def test_alt_calc (self):  # this is a method so it should start with test_ and then the function we want to test
		alt_return_dict = gtools.alt_calc()  # file name. function name