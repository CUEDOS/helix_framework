from helixio.gtools import alt_calc # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest

def test_alt_calc_1 ():
	alt_dict={"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18} 
	alt_return_dict_1={"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5} # output dictionary for input set 1
	assert alt_calc(alt_dict)==alt_return_dict_1
