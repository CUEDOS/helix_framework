def alt_calc(alt_dict):

    min_alt = 10  # meters - min return altitude above launch altitude

    for i in range(0, CONST_SWARM_SIZE - 1):  # Saving current altitudes in alts array
        alts[i] = alt_dict["P" + str(i + 101)]

    alts.sort()    # Sorting alts
    
    mean=0
    for i in range(0, CONST_SWARM_SIZE - 1): # Calculating the mean of current alts
        mean=mean+alts[i]
    mean=mean/CONST_SWARM_SIZE
    
    alt_return_sorted

    if alt_return_sorted[0]<min_alt:  # Checking minimum alt
        difference=min_alt-alt_return_sorted[0]
        for i in range(0, CONST_SWARM_SIZE - 1):
            alt_return_sorted[i]=alt_return_sorted[i]+difference

    for i in range(0, CONST_SWARM_SIZE - 1):  # assigning sorted alts (Now i shows the order of the drones in alts)
        alt_return_dict[alt_dict[alts[i]]] = alt_return_sorted[i]

    return alt_return_dict
