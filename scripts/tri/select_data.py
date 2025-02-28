import numpy as np

# Function to read data from file and convert to required format
def read_data(file_path):
    data_sets = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Skip empty lines
                # Parse the line to extract data
                parts = line.split('; ')
                current_data = {}
                current_data['Pose'] = eval(parts[0].split('Pose: ')[1])
                current_data['theta_Rho'] = eval(parts[1].split('theta_Rho: ')[1])
                current_data['s_p'] = eval(parts[2].split('s_p: ')[1])
                current_data['w_p'] = eval(parts[3].split('w_p: ')[1])
                data_sets.append(current_data)
    return data_sets

# Function to calculate difference between two data sets
def calculate_difference(data1, data2):
    pose_diff = np.sum((np.array(data1['Pose']) - np.array(data2['Pose'])) ** 2)
    theta_Rho_diff = np.sum((np.array(data1['theta_Rho']) - np.array(data2['theta_Rho'])) ** 2)
    s_p_diff = np.sum((np.array(data1['s_p']) - np.array(data2['s_p'])) ** 2)
    w_p_diff = np.sum((np.array(data1['w_p']) - np.array(data2['w_p'])) ** 2)
    total_diff = pose_diff + theta_Rho_diff + s_p_diff + w_p_diff
    return total_diff

def read_data2(file_path):
    data_sets = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # Skip empty lines
                # Parse the line to extract data
                parts = line.split('; ')
                Pose = eval(parts[0].split('Pose: ')[1])
                theta_Rho = eval(parts[1].split('theta_Rho: ')[1])[0]
                P = eval(parts[2].split('s_p: ')[1])[0]
                data_sets.append([Pose, theta_Rho, P])
    return data_sets



if __name__ == "__main__":
    # Finding the two data sets with the maximum difference
    file_path = 'record.txt'
    data_sets = read_data(file_path)

    max_diff = 0
    max_pair = (0, 0)

    for i in range(len(data_sets)):
        for j in range(i + 1, len(data_sets)):
            diff = calculate_difference(data_sets[i], data_sets[j])
            if diff > max_diff:
                max_diff = diff
                max_pair = (i, j)

    # print(data_sets[max_pair[0]])
    # print(data_sets[max_pair[1]])
    
    data_sets = read_data2(file_path)
    print(data_sets[0][2])
    