import unittest
import gtools # importing the module we want to test its function (the test file should be in the same directory as module file)

class TestGtools(unittest.TestCase): # inherit from unittest.TestCase 
	def test_alt_calc (self):  # this is a method so it should start with test_ and then the function we want to test
		alt_dict={"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18} # altitude set 1
		alt_return_dict_1={"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5} # correct dictionary for input set 1
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict=", alt_return_dict)
		self.assertEqual(alt_return_dict_1,alt_return_dict)

if __name__ == '__main__':  # to be able to run the code directly
	unittest.main()