# README  

## Project Overview  
This project implements a newly proposed algorithm, **BESTAnP**, and compares its performance with three baseline algorithms: **App Algorithm**, **Non-app Algorithm**, and **Combine+CIO Algorithm**. The goal is to solve the acoustic n-point problem (AnP), which involves estimating the pose (position and orientation) of an underwater robot using sonar measurements and the known positions of several feature points in the environment.  

Additionally, the project integrates MATLAB with Python to leverage the SLSQP optimization method, which is not natively supported in MATLAB.  

## File Structure  

### **Experiments**  
This folder contains subfolders for different experiments, each evaluating the algorithms under specific conditions. Every folder includes the required algorithms and a `.mlx` script to run the experiments.  

#### Subfolders:  
1. **compare_different_noise_adding_scheme**  
   Evaluates algorithm performance under two different noise addition schemes.  
2. **compare_GN_with_different_times**  
   Compares the performance of the Gauss-Newton optimization algorithm with varying iteration numbers.  
3. **compare_points_distribution**  
   Analyzes the effect of different feature point distributions on localization accuracy.  
4. **reprojection_experiments**  
   Tests the reprojection error for evaluating the algorithms' accuracy.  
5. **test_noise_intensity_influence**  
   Investigates the impact of different noise intensities on the estimation results.  
6. **test_numberpoint_influence**  
   Studies the influence of the number of feature points on the algorithms' performance.  
7. **test_running_time**  
   Compares the runtime of the algorithms.  

Each subfolder includes:  
- The required algorithms from the **Algorithm** folder.  
- An experiment script (`*.mlx`) that runs the corresponding test and outputs detailed results.  

## Quick Start  

### Environment Setup  

1. Install MATLAB and ensure all related scripts are placed in the same working directory.  

2. Install Python and set up the following dependencies:  

   - `numpy`  
   - `scipy`  
   - `matplotlib` (optional, for visualization in Python)  

3. Install **CPython** to enable integration between MATLAB and Python.  

4. Ensure MATLAB can call Python via `pyenv`. Test the connection:  

   ```matlab
   pyInfo = pyenv;
   disp(pyInfo)
   py.numpy.array([1, 2, 3]); % Check if Python calls are successful
   ```

5. When using the baseline algorithms (**App Algorithm**, **Non-app Algorithm**, and **Combine+CIO Algorithm**), ensure the paths to the required `.py` files ( `calculate_tz.py` and `CIO.py`) are correctly specified in the variable `py_path` in your MATLAB scripts.

## Contact Information

If you have any questions or suggestions, please contact the project authors.

