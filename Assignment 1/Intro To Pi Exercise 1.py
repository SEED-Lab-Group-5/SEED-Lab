#######################################################################################
# NAME: David Long
# CLASS: EENG-350
# TITLE: Introduction to Pi Challenge Activity:
#        Reads in a text file containing a list of numbers, then finds
#        various properties of the list.
#
# RESOURCE: https://www.w3schools.com/python/python_ref_list.asp
# PURPOSE:  Learn about array methods in Python
#
# RESOURCE: https://stackoverflow.com/questions/522563/accessing-the-index-in-for-loops
# PURPOSE:  Remember how to use the enumerate function to track the index in a for loop
#
# RESOURCE: https://www.w3schools.com/python/ref_func_print.asp
# PURPOSE:  Remember print function parameters (sep)
#
# RESOURCE: https://stackoverflow.com/questions/67563547/how-to-install-numpy-using-official-python-idle
# PURPOSE:  Learn how to install the numPy module
#
# RESOURCE: https://numpy.org/doc/stable/reference/generated/numpy.array.html
# PURPOSE:  Learn how to use numpy arrays
#######################################################################################
import numpy as np

# Read in the file and convert to a list
with open('datafile.txt', 'r') as inFile:
    txtList = eval(inFile.read())


# Find and print the max and min numbers in the txtList list
print("The maximum number in the list is", max(txtList))
print("The maximum number in the list is", min(txtList))


# Find the index of the number 38
print("The number 38 is at index", txtList.index(38))


# Find all unique elements in the txtList list
uniqueElements = []
for element in txtList:
    if element not in uniqueElements:
        uniqueElements.append(element)


# Find number of occurrences of each unique element
elementCounts = []
for element in uniqueElements:
    elementCounts.append(txtList.count(element))


# Run through elementCounts list once to find the maximum element count
maxCount = elementCounts[0]
for element in elementCounts:
    if element > maxCount:
        maxCount = elementCounts[element]

        
# Run through elementCounts list a second time to find all numbers in the uniqueElements
# list with number of occurrences == maxCount
maxCountNumbers = []
for index, element in enumerate(elementCounts):
    if element == maxCount:        
        maxCountNumbers.append(uniqueElements[index])


# Print the number with maximum occurences and how many time they appear
listSize = len(maxCountNumbers)
if(listSize > 1): # If there is more than one number with max occurrences
    print("The numbers ", maxCountNumbers[0:listSize-1], " and [", maxCountNumbers[listSize-1],
          "] have the most occurrences. They appear ", maxCount, " times each.", sep = '')
    
else:   # If there is only one number with max occurrences
     print("The number", maxCountNumbers[0], "Has the most occurrences. It appears", maxCount, "times.")


# Convert txtList to an array then sort numerically
txtArray = np.array(txtList)    # Convert to a numpy array
sortedArray = np.sort(txtArray) # Sort the array
print("\nThe array in ascending order is:")
print(sortedArray, "\n")


# Find all even numbers in the sorted array
evenNumbers = []
for element in sortedArray:
    if element % 2 == 0:        # Determine if number is even
        evenNumbers.append(element)
print("All even numbers in the array are shown below in ascending order:")
print(np.array(evenNumbers))    # Numpy array used to match formatting
