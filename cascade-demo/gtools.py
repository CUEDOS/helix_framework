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

    for i in drone_ids:  # Saving current altitudes in alts array
        alts = alt_dict["P" + str(i)]

    alts.sort()  # Sorting alts

    # TODO find a way to sort the altitudes, and therefore sorting the drones

    # TODO Make new dictionary with return altitudes

    # TODO Ensure all altitudes are higher than min altitude

    return output_dict
