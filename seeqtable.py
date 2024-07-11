import pickle

# Open the pickle file in binary read mode
with open('q_table.pkl', 'rb') as f:
    # Load the data from the file
    data = pickle.load(f)

# Print the unpickled data
print(data)