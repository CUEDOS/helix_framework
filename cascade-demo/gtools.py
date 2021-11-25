def alt_calc(alt_dict):
    """
    Parameters
    ------------
    alt_dict: Dict(key drone_index (string): value altitude (float), ...)

    Example:
    alt_dict =
    {
    "P101": 20,
    "P102": 12,
    ...
    }
    x = alt_dict["P101"]
    print(x)
    # 20

    Returns:
    --------
    output_dict: Dict(key:drone_index (string), value: altitude (float))
    """

    min_alt = 10  # meters - min return altitude above launch altitude

    for i in range(0, CONST_SWARM_SIZE - 1):  # Saving current altitudes in alts array
        alts[i] = alt_dict["P" + str(i + 101)]

    alts.sort()                              # Sorting alts
    mean=0
    for i in range(0, CONST_SWARM_SIZE - 1): # Calculating the mean of current alts
        mean=mean+alts[i]
    mean=mean/CONST_SWARM_SIZE
    
    alt_return_sorted
    for i in range(0, CONST_SWARM_SIZE - 1):  # Now i shows the order of the drones in alts
        alt_return_dict[alt_dict[alts[i]]] = alt_return_sorted[i]

    return alt_return_dict
