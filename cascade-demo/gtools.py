from math import floor
def alt_calc(alt_dict):

    min_alt = 10  # in meters - min return altitude above launch altitude
    alt_step = 2  # in meters - the alt difference between return alts

    alts=[0]*CONST_SWARM_SIZE                # Saving current altitudes in alts array ----------------------------------
    for i in range(0, CONST_SWARM_SIZE - 1):
        alts[i] = alt_dict["P" + str(i + 101)]

    alts.sort()    # Sorting alts
    
    mean=0          # Calculating the mean of current alts --------------------------------------------------------------
    for i in range(0, CONST_SWARM_SIZE - 1): 
        mean=mean+alts[i]
    mean=mean/CONST_SWARM_SIZE
    
    #Creating sorted return alts ------------------------------------------------------------------------------------------
    middle_element=floor(CONST_SWARM_SIZE/2)
    alt_return_sorted=[0]*CONST_SWARM_SIZE
    alt_return_sorted[middle_element]=mean
    
    if CONST_SWARM_SIZE%2 == 1:
        for i in range (1, (CONST_SWARM_SIZE - 1)/2):
            alt_return_sorted[middle_element-i]=mean-i*alt_step
            alt_return_sorted[middle_element+i]=mean+i*alt_step
    else:
        for i in range (1, (CONST_SWARM_SIZE - 2)/2):
            alt_return_sorted[middle_element-i]=mean-i*alt_step
            alt_return_sorted[middle_element+i]=mean+i*alt_step
        alt_return_sorted[0]=alt_return_sorted[1]-alt_step

    if alt_return_sorted[0]<min_alt:  # Checking minimum alt ----------------------------------------------------------------
        difference=min_alt-alt_return_sorted[0]
        for i in range(0, CONST_SWARM_SIZE - 1):
            alt_return_sorted[i]=alt_return_sorted[i]+difference

    for i in range(0, CONST_SWARM_SIZE - 1):  # assigning sorted alts (Now i shows the order of the drones in alts) ----------
        alt_return_dict[alt_dict[alts[i]]] = alt_return_sorted[i]

    return alt_return_dict
