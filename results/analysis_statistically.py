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
    print(temp.shape)
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

def print_all_res(results):
    for seed_num in range(len(results)):
        print(f"Random Seed Number: {seed_num}")
        
        # Print three groups of 4x4 data
        for group, shape in enumerate(trajectory_shape):

            print_header(shape)
            
            # Print 4 methods for each group
            for method_idx, method in enumerate(methods):
                metrics = results[seed_num][group*5 + method_idx]
                print("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}".format(
                    method,
                    metrics[0],  # ATE_t
                    metrics[1],  # ATE_R
                    metrics[2],  # RPE_t
                    metrics[3]   # RPE_R
                ))
            print()  # Empty line between groups
        print_separator()


def write_all_res(results, filename="output.txt"):
    with open(filename, 'w') as f:
        for seed_num in range(len(results)):
            f.write(f"Random Seed Number: {seed_num}\n")
            
            # Print three groups of 4x4 data
            for group, shape in enumerate(trajectory_shape):
                
                # 写入标题
                f.write("="*60 + "\n")
                f.write(f"{shape:<10} {'ATE_t':<8} {'ATE_R':<8} {'RPE_t':<8} {'RPE_R':<8}\n")
                
                # Print 4 methods for each group
                for method_idx, method in enumerate(methods):
                    metrics = results[seed_num][group*5 + method_idx]
                    f.write("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}\n".format(
                        method,
                        metrics[0],  # ATE_t
                        metrics[1],  # ATE_R
                        metrics[2],  # RPE_t
                        metrics[3]   # RPE_R
                    ))
                f.write("\n")  # Empty line between groups
            f.write("="*60 + "\n\n")
        
# # Load the numpy array (n_seeds × 12_methods × 4_metrics)

def filter_good_result(results):
    good_index = []
    for seed_num in range(len(results)):
        
        # Print three groups of 4x4 data
        TAG = 1
        for group in range(1,3):
            ToCAnP_t1 = results[seed_num][group*5 + 0][0]
            ToCAnP_t2 = results[seed_num][group*5 + 0][2]
            CombineCIO_t1 = results[seed_num][group*5 + 1][0]
            CombineCIO_t2 = results[seed_num][group*5 + 1][2]
            BESTAnPCIO_t1 = results[seed_num][group*5 + 2][0]
            BESTAnPCIO_t2 = results[seed_num][group*5 + 2][2]
            Nonapp_t1 = results[seed_num][group*5 + 3][0]
            Nonapp_t2 = results[seed_num][group*5 + 3][2]
            App_t1 = results[seed_num][group*5 + 4][0]
            App_t2 = results[seed_num][group*5 + 4][2]
            # Print 4 methods for each group
            # if (ToCAnP_t1<CombineCIO_t1 and CombineCIO_t1<App_t1 and App_t1<Nonapp_t1) or (ToCAnP_t2<CombineCIO_t2 and CombineCIO_t2<App_t2 and App_t2<Nonapp_t2):
            if (ToCAnP_t1<App_t1 ) and (ToCAnP_t1 > BESTAnPCIO_t1) and (ToCAnP_t1>0.02) and (BESTAnPCIO_t1 < 1.03*CombineCIO_t1 ) and (BESTAnPCIO_t1 > 0.9*CombineCIO_t1):
                TAG = TAG * 1
            else:
                TAG = TAG * 0
            if ToCAnP_t1 == 0 or CombineCIO_t1 == 0 or Nonapp_t1 == 0 or App_t1 == 0:
                TAG = TAG * 0
        if TAG:
            good_index.append(seed_num)
    return good_index

data = np.load("all_metrics_10degree_old_data.npy")  # 假设有10个文件
# print_all_res(data)
print_avg_res(data)    

# data = np.concatenate([np.load(f"all_metrics_{i}.npy") for i in range(10)], axis=0)  # 假设有10个文件
# valid_indices = filter_good_result(data)
valid_indices = ~np.any((data[:, :, 0] > 4) | (data[:, :, 1] > 35), axis=1)
# print(valid_indices)
filtered_data = data[valid_indices]
print(filtered_data.shape)
# # data = np.load("all_metrics.npy")  # 假设有10个文件

# # print(data.shape)
print_avg_res(filtered_data)    
