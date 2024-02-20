'''
Stephen Zhou
Makes a line plot from a CSV file
CSV file should have the same format as the table in the manual

I know what the table is supposed to look like, but I tried not to hard code it
'''

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

originalData = pd.read_csv('sturgeon.csv')

#creating a dataframe where columns are days and receivers
updatedData = pd.DataFrame()

tempDictionary = {'Receiver 1':[], 'Receiver 2':[], 'Receiver 3':[]} #dictionary to keep track of columns

for row in range(len(originalData)): #adds values from the OG dataframe to dictionary
    for column in originalData:
        if column.isnumeric(): #prevents column names (receiver #) from getting added
            tempDictionary[originalData['Day'][row]].append(originalData[column][row])

updatedData['Day'] = np.arange(1, len(tempDictionary['Receiver 1']) + 1) #makes column 'Day' using length of 'Receiver 1' in case more/less days are needed

for key, value in tempDictionary.items(): #adds columns to updated dataframe
    updatedData[key] = value

maxSturgeonsSeen = [] #finds the highest number of sturgeons seen by a receiver in one day; useful for setting y-ticks
for column in updatedData:
    if column != 'Day':
        maxSturgeonsSeen.append(updatedData[column].max())

#plotting the updated data frame; this is where things get hard coded
plt.rcParams['figure.figsize'] = [10, 10]

plt.plot(updatedData['Day'], updatedData['Receiver 1'], color = 'green', label = 'Receiver 1')
plt.plot(updatedData['Day'], updatedData['Receiver 2'], color = 'red', label = 'Receiver 2')
plt.plot(updatedData['Day'], updatedData['Receiver 3'], color = 'blue', label = 'Receiver 3')

#customizing the graph
plt.legend(loc = 'best')
plt.xticks(np.arange(1, 16))
plt.yticks(np.arange(0, max(maxSturgeonsSeen) + 1, 2))
plt.grid(axis = 'y')

#adding descriptive labels 
plt.suptitle('Number of Sturgeons Detected by Receivers Over Time')
plt.xlabel('Day')
plt.ylabel('Number of Sturgeons')

plt.savefig('Sturgeon Visualization.png')
plt.show()