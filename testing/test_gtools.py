from helixio.gtools import alt_calc # importing the module we want to test its function (the test file should be in the same directory as module file)


def test_alt_calc_1 (self):  # this is a method so it should start with test_ and then the function we want to test
    # input output set 1-------------------------------------------
	# print ("Set 1:")
	# altitude set 1 (even number of drones)
	alt_dict={"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18} 
	alt_return_dict_1={"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5} # output dictionary for input set 1
	assert alt_calc(alt_dict)==alt_return_dict_1  # module name. function name (input)
	# print ("alt_return_dict for set 1=", alt_return_dict, "\n")

def test_alt_calc_2 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 2 ---------------------------------------------
	# print ("Set 2:")
	# altitude set 2 (even number of drones, two same altitudes)
	alt_dict={"P101":12, "P102":10, "P103":15, "P104":15, "P105":20, "P106":18} 
	alt_return_dict_2={"P101":13.5, "P102":12.5, "P103":14.5, "P104":15.5, "P105":17.5, "P106":16.5} # output dictionary for input set 2
	assert alt_calc(alt_dict)==alt_return_dict_2  # module name. function name (input)
	# print ("alt_return_dict for set 2=", alt_return_dict, "\n")

def test_alt_calc_3 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 3 -----------------------------------------------
	#print ("Set 3:")
	# altitude set 3 (odd number of drones)
	alt_dict={"P101":17, "P102":15, "P103":24, "P104":20, "P105":14} 
	alt_return_dict_3={"P101":18, "P102":17, "P103":20, "P104":19, "P105":16} # output dictionary for input set 3
	assert alt_calc(alt_dict)==alt_return_dict_3  # module name. function name (input)
	# print ("alt_return_dict for set 3=", alt_return_dict, "\n")

def test_alt_calc_4 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 4 -----------------------------------------------
	# print ("Set 4:")
	# altitude set 4 (odd number of drones, two same altitudes)
	alt_dict={"P101":20, "P102":15, "P103":21, "P104":20, "P105":14} 
	alt_return_dict_4={"P101":18, "P102":17, "P103":20, "P104":19, "P105":16} # output dictionary for input set 4
	assert alt_calc(alt_dict)==alt_return_dict_4  # module name. function name (input)
	# print ("alt_return_dict for set 4=", alt_return_dict)
	
def test_alt_calc_5 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 5 -----------------------------------------------
	# print ("Set 5:")
	# altitude set 5 (8 drones, all same altitudes)
	alt_dict={"P101":20, "P102":20, "P103":20, "P104":20, "P105":20, "P106":20, "P107":20, "P108":20}
	alt_return_dict_5={"P101":16.5, "P102":17.5, "P103":18.5, "P104":19.5, "P105":20.5, "P106":21.5, "P107":22.5, "P108":23.5} # output dictionary for input set 5
	assert alt_calc(alt_dict)==alt_return_dict_5  # module name. function name (input)
	# print ("alt_return_dict for set 5=", alt_return_dict)
	
def test_alt_calc_6 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 6 -----------------------------------------------
	# print ("Set 6:")
	# altitude set 6 (7 drones, three & two same altitudes, huge differences)
	alt_dict={"P101":1000, "P102":0, "P103":1, "P104":1, "P105":1, "P106":20000, "P107":0}
	alt_return_dict_6={"P101":99, "P102":94, "P103":96, "P104":97, "P105":98, "P106":100, "P107":95} # output dictionary for input set 6
	assert alt_calc(alt_dict)==alt_return_dict_6  # module name. function name (input)
	# print ("alt_return_dict for set 6=", alt_return_dict)

def test_alt_calc_7 (self):  # this is a method so it should start with test_ and then the function we want to test
	# input output set 7 -----------------------------------------------
	# print ("Set 7:")
	# altitude set 7 (10 drones, four & two same altitudes)
	alt_dict={"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2}
	alt_return_dict_7={"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11} # output dictionary for input set 7
	assert alt_calc(alt_dict)==alt_return_dict_7  # module name. function name (input)
	# print ("alt_return_dict for set 7=", alt_return_dict)
