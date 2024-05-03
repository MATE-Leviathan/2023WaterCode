'''
Stephen Zhou
Makes a line plot from a CSV file
CSV file should have the same format as the table in the manual
hello
'''

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

originalData = pd.read_csv('sturgeon.csv')

#transposing dataframe
updatedData = originalData.transpose() 
updatedData = updatedData.to_numpy().tolist() #converting to list to remove an element (the column headers)
updatedData.pop(0)

updatedData = np.asarray(updatedData).transpose() #converted into an array with three arrays for each receiver's data

#creating new dataframe with has been transposed 
updatedDataFrame = pd.DataFrame({'Day':np.arange(1, len(updatedData[0])+1), 'Receiver 1':updatedData[0], 'Receiver 2':updatedData[1], 'Receiver 3':updatedData[2]})

plt.rcParams['figure.figsize'] = [10, 10]

#plotting
plt.plot(updatedDataFrame['Day'], updatedDataFrame['Receiver 1'], color = 'green', label = 'Receiver 1')
plt.plot(updatedDataFrame['Day'], updatedDataFrame['Receiver 2'], color = 'red', label = 'Receiver 2')
plt.plot(updatedDataFrame['Day'], updatedDataFrame['Receiver 3'], color = 'blue', label = 'Receiver 3')

#customizing the graph
plt.legend(loc = 'best')
plt.xticks(np.arange(1, 16))

maximumVal = max([np.max(updatedData[0]), np.max(updatedData[1]), np.max(updatedData[2])]) #find max sturgeons seen, useful for setting yticks
plt.yticks(np.arange(0, maximumVal + 1, 2))
plt.grid(axis = 'y')

#adding descriptive labels 
plt.suptitle('Number of Sturgeons Detected by Receivers Over Time')
plt.xlabel('Day')
plt.ylabel('Number of Sturgeons')

plt.savefig('Sturgeon Visualization.png')
plt.show()