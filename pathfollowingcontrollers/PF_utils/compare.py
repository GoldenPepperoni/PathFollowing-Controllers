from importlib import import_module
import sys
import numpy as np
import matplotlib.pyplot as plt

data = np.array([])
itseArray = []
iseArray = []
iaeArray = []

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
        algoName = algoPath[1:]

        try:
            # Run the script by importing
            algoModule = import_module(algoPath, "pathfollowingcontrollers.algorithms")
            print(f"Finished running {algoName} !!!")
        except:
            exit(f"Unable to run {sys.argv[i+1]} !!!")
        
        
        desiredPath, actualPath, ctrlTraces, tArray = algoModule.getPerformanceData()

        # Calculate errors
        absErr = np.linalg.norm([actualPath])
        iae = np.sum(absErr)
        ise = np.sum(absErr**2)
        itse = np.matmul(absErr, tArray)

        # Plot bar chart comparing errors


        offset = width * multiplier
        rects = ax.bar(i + offset, iae, width, label=algoName)
        ax.bar_label(rects, padding=3)
        multiplier += 1

        plt.show()
