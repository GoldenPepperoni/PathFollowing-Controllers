from importlib import import_module
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathfollowingcontrollers.PF_utils.abstractions import *

# Initialise Array
algoName_arr = []
desired_path_arr = []
actual_path_arr = []
IAE_arr = []
ISE_arr = []
ITAE_arr = []
dict = {}

# Initialise Plot
fig, ax = plt.subplots(layout='constrained')
width = 0.25  # the width of the bars
multiplier = 0

if __name__ == "__main__":

    # Handle situation where lesser than 2 algorithms were put to compared
    if len(sys.argv) <= 2:
        exit("Comparison need more than 2 algorithms!")

    # Run each algorithm script
    for i in range(1, len(sys.argv)):
        algoScript = sys.argv[i] # Extract individual script
        algoPath = algoScript.replace("/", ".")[:-3] # Formatting
        algoName = algoPath[1:].replace(".", "_")
        algoName_arr.append(algoName)

        try:
            # Run the script by importing
            print(f"Now running {algoName} !!!")
            algoModule = import_module(algoPath, "pathfollowingcontrollers.algorithms")
            print(f"Finished running {algoName} !!!")
        except:
            exit(f"Unable to run {sys.argv[i+1]} !!!")
        
        
        # Collect trajectory into an array
        desired_path_arr.append(algoModule.desiredPath)
        actual_path_arr.append(algoModule.actualPath)


        IAE, ISE, ITAE = getPerformanceData(algoModule.cross_track_err, algoModule.actualPath, algoModule.tArray)

        # Collect metrics into array
        IAE_arr.append(IAE)
        ISE_arr.append(ISE)
        ITAE_arr.append(ITAE)


    # Initialise plot title
    title = algoName_arr[0]
    # Assemble plot title
    for i in range(1, len(IAE_arr)):
        # Update title
        title += " VS " + algoName_arr[i]


    # Plot desired vs actual trajectory
    plot3D(desired_path_arr[0], actual_path_arr, "3D Trajectory "+title, algoName_arr)


    # Collect metrics into a dictionary
    dict["IAE"] = (IAE_arr)
    dict["ISE"] = (ISE_arr)
    dict["ITAE"] = (ITAE_arr)

    # Plot bar chart comparing errors
    for metric, values in dict.items():
        offset = width * multiplier
        rects = ax.bar(np.arange(len(IAE_arr)) + offset, values, width, label=metric)
        ax.bar_label(rects, padding=3)
        multiplier += 1

        ax.legend(loc='upper center')

    ax.set_xticks(np.arange(len(IAE_arr)) + width, algoName_arr)
    ax.set_title("Metrics of "+title)
    plt.savefig("plots/"+"Metrics of "+title)

    plt.show()
