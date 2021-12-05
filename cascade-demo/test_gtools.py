import unittest
import gtools # importing the module we want to test its function (the test file should be in the same directory as module file)

class TestGtools(unittest.TestCase): # inherit from unittest.TestCase 
	def test_alt_calc (self):  # this is a method so it should start with test_ and then the function we want to test
    # input output set 1-------------------------------------------
		alt_dict={"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18} # altitude set 1 (even number of drones)
		alt_return_dict_1={"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5} # output dictionary for input set 1
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict for set 1=", alt_return_dict)
		self.assertEqual(alt_return_dict_1,alt_return_dict) # asserting outputs of set 1
	# input output set 2 ---------------------------------------------
		alt_dict={"P101":12, "P102":10, "P103":15, "P104":15, "P105":20, "P106":18} # altitude set 2 (even number of drones, two same altitudes)
		alt_return_dict_2={"P101":13.5, "P102":12.5, "P103":14.5, "P104":15.5, "P105":17.5, "P106":16.5} # output dictionary for input set 2
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict for set 2=", alt_return_dict)
		self.assertEqual(alt_return_dict_2,alt_return_dict) # asserting outputs of set 2
	# input output set 3 -----------------------------------------------
		alt_dict={"P101":17, "P102":15, "P103":24, "P104":20, "P105":14} # altitude set 3 (odd number of drones)
		alt_return_dict_3={"P101":18, "P102":17, "P103":20, "P104":19, "P105":16} # output dictionary for input set 3
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict for set 3=", alt_return_dict)
		self.assertEqual(alt_return_dict_3,alt_return_dict) # asserting outputs of set 3
	# input output set 4 -----------------------------------------------
		alt_dict={"P101":20, "P102":15, "P103":21, "P104":20, "P105":14} # altitude set 4 (odd number of drones, two same altitudes)
		alt_return_dict_4={"P101":18, "P102":17, "P103":20, "P104":19, "P105":16} # output dictionary for input set 4
		alt_return_dict = gtools.alt_calc(alt_dict)  # module name. function name (input)
		print ("alt_return_dict for set 4=", alt_return_dict)
		self.assertEqual(alt_return_dict_4,alt_return_dict) # asserting outputs of set 4

if __name__ == '__main__':  # to be able to run the code directly
	unittest.main()