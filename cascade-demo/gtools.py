import numpy as np


def alt_calc(alt_dict):
    """
    Parameters
    ------------
    alt_dict: Dict(key drone_index (string): value altitude (float), ...)

    Returns
    -----------
    output_dict: Dict(key:drone_index (string), value: altitude (float))
    """
    print("alt_dict=", alt_dict)
    site_elevation = 488
    alt_lims = np.array(
        [10, 100]
    )  # in meters - min return altitude above launch altitude
    alt_lims += site_elevation
    alt_step = 1  # in meters - the alt difference between return alts

    size = len(alt_dict)
    # Sorting dictionary by values and returning list of keys -----------------------------------------------------------------------------
    sorted_idx = sorted(alt_dict, key=alt_dict.get)

    # Calculating the mean of current alts -------------------------------------------------------------------------------
    mean = sum(alt_dict.values()) / size

    # Creating sorted return alts ------------------------------------------------------------------------------------------
    alts = np.arange(size) - (size - 1) / 2  # Makes an array centered around 0
    alts *= alt_step  # Increases to stepsize
    alts += mean  # Adds to mean

    # Checking minimum and maximum alt -----------------------------------------------------------------------------------------------
    # if alts[0] < alt_lims[0]:
    #     difference = alt_lims[0] - alts[0]
    #     alts += difference  # Increase altitude so greater than min
    # if alts[-1] > alt_lims[1]:
    #     difference = alts[-1] - alt_lims[1]
    #     alts -= difference  # Reduce altitude so less than max

    # Assigning alts to ordered index  -----------------------------------------------
    alt_return_dict = {}
    for i, idx in enumerate(sorted_idx):
        alt_return_dict[idx] = alts[i]

    return alt_return_dict
