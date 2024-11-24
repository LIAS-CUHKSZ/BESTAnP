# README  

## Project Overview  
This project implements a newly proposed algorithm, **BESTAnP**, and compares its performance with three baseline algorithms: **App Algorithm**, **Non-app Algorithm**, and **Combine+CIO Algorithm**. The goal is to solve the acoustic n-point problem (AnP), which involves estimating the pose (position and orientation) of an underwater robot using sonar measurements and the known positions of several feature points in the environment.  

Additionally, the project integrates MATLAB with Python to leverage the SLSQP optimization method, which is not natively supported in MATLAB.  

## File Structure  

### Core Algorithms  
- **BESTAnP.m**  
  The core algorithm proposed in this project.  

- **App_Algorithm.m**  
  An approximate algorithm. 

- **Nonapp_Algorithm.m**  
  A non-approximate algorithm.  

- **Combine_CIO.m**  
  Combines the results of App_Algorithm and Nonapp_Algorithm, refining them with Consistent Iterative Optimization (CIO).  

### Utility Functions  
- **isRotationMatrix.m**  
  Validates whether a given matrix is a valid rotation matrix.  

- **ToRotationMatrix.m**  
  Converts an input matrix or vector into a valid rotation matrix.  

### Test Script  
- **ALgorithm_test.mlx**  
  A test script that evaluates all algorithms by testing their ability to estimate the **rotation matrix \( R \)** and the **translation vector \( t \)**. 

### Python Script  
The Python scripts that use the SLSQP optimization method to solve optimization problems. It is only utilized by the three baseline algorithms (**App Algorithm**, **Non-app Algorithm**, and **Combine+CIO Algorithm** ). 

- **calculate_tz.py**    

  Computes the third dimension of the translation vector \( t_z \).  

- **CIO.py**   

  Combines the estimation results from **App Algorithm** and **Non-app Algorithm**, and further refines them using Consistent Iterative Optimization (CIO).  

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

