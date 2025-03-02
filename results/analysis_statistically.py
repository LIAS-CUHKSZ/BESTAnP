import numpy as np

methods = ['ToCAnP', 'CombineCIO', 'BESTAnPCIO', 'Nonapp', 'App']
trajectory_shape = ['square', 'circle', 'eight']
def print_header(shape):
    print("{:<10} {:<8} {:<8} {:<8} {:<8}".format(
        shape, "ATE_t", "ATE_R", "RPE_t", "RPE_R"))
        
def print_separator():
    print("=" * 50)

def print_avg_res(temp):
    print_separator()
    average_matrix = np.mean(temp, axis=0)
    # Print three groups of 4x4 data
    for group, shape in enumerate(trajectory_shape):
        print_header(shape)
        # Print 4 methods for each group
        for method_idx, method in enumerate(methods):
            metrics = average_matrix[group*5 + method_idx]
            print("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}".format(
                method,
                metrics[0],  # ATE_t
                metrics[1],  # ATE_R
                metrics[2],  # RPE_t
                metrics[3]   # RPE_R
            ))
        print()  # Empty line between groups
    print_separator()



data = np.load("results_5000.npy")
valid_indices = ~np.any((data[:, :, 0] > 4) | (data[:, :, 1] > 35), axis=1)  # we need to filter our outliers
filtered_data = data[valid_indices]
print("The unit for t is meter and unit for R is degree, cooresponding to Table II (see our paper)")
print_avg_res(filtered_data)    
