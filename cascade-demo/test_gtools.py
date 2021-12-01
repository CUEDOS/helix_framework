import unittest
import gtools # importing the module we want to test its function (the test file should be in the same directory as module file)

class TestGtools(unittest.TestCase): # inherit from unittest.TestCase 
	def test_alt_calc (self):  # this is a method so it should start with test_ and then the function we want to test
		alt_dict={"P101":25, "P102":12.5, "P103":11, "P104":21, "P105":18, "P106":14}
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict=", alt_return_dict)

if __name__ == '__main__':  # to be able to run the code directly
	unittest.main()