
# Structure-from-Sherds 
 <p align="center">
 <img src="https://github.com/SeongJong-Yoo/structure-from-sherds/blob/main/etc/Pot%20reconstruction.gif">
 </p>
 
 This repository contains code for restoring pottery of unordered and mixed fragments. The code is provided without any warranty. If using this code, please cite our work as shown below. 

	@inproceedings{Hong_2021_ICCV,
    	author    = {Hong, Je Hyeong and Yoo, Seong Jong and Zeeshan, Muhammad Arshad and Kim, Young Min and Kim, Jinwook},
    	title     = {Structure-From-Sherds: Incremental 3D Reassembly of Axially Symmetric Pots From Unordered and Mixed Fragment Collections},
    	booktitle = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
    	month     = {October},
    	year      = {2021},
    	pages     = {5443-5451}
	}

## News
[2025/02/06] Extended version of structure-from-sherds is released! Check it [here](https://sj-yoo.info/sfs/) 

## How to run
1. Install requirements using [VCPKG](https://vcpkg.io/en/).
    * Classic mode
    ```
    vcpkg install ceres[*]
    vcpkg install pcl[*] 
    ```
    or
    ``` 
    vcpkg install pcl[tools]
    vcpkg install pcl[vtk] --recurse
    vcpkg install pcl[visualization] --recurse
    ```
	* Manifest mode
        - Create 'vcpkg.json'
        ```
        {
            "$schema": "https://raw.githubusercontent.com/microsoft/vcpkg-tool/main/docs/vcpkg.schema.json",
            "name": "base-agnostic",
            "version": "0.1.0",
            "dependencies": [
                {"name": "ceres", "features":["cxsparse", "eigensparse", "lapack", "suitesparse", "tools"]},
                {"name": "pcl", "features": ["visualization", "vtk", "tools"]}
            ]
        }
        ```
2. Build with CMAKE file.
3. Download pottery data : https://drive.google.com/file/d/1sqUlQXsW9lMhPkcDwKUF6-5vau1YPhhR/view?usp=sharing
5. Choose one of the experiments commented out at *data_path.h*. 
6. Run main.cpp file.

### Parameters
Data path : Path of the input data set  

	data_path.h
	string path = "../ICCV Data/"; 

Beam-search parameters : TOP_k indicates the *k*-top ranked beams being brought to the next iteration and BRANCHE_b indicates *b* branches expanding to search for more possible paths in each iteration. (Combinations of (k, b) as (5, 3), (10,5), and (20,10) are recommended)

	main.cpp
	TOP_k : 5
	BRANCH_b : 3

Number of CPU Thread : The number of CPU thread utilizing in*OpenMP* and *ceres solver*.
	
	data_structure.h
	NUMBER_OF_THREAD : 16

Ceres solver function tolerance : ceres hyper-parameter for convergence. We recommend to use the same or a lower value than 1.0e-6. For more details check [here](http://ceres-solver.org/nnls_solving.html). 

	reconstruction.h
	CERES_FUNC_TOL : 1.0e-6

MINIMUM_NUMBER : The minimum number of points on the edge line  to be considered in the calculation.

	reconstruction.h
	MINIMUM_NUMBER : 7

Counting inliers threshold : In order to evaluate ranking, we counted the number of inliers of that distance is less than this value. (Range 1-2 is recommended)

	reconstruction.h
	INLIER_THRESHOLD : 1.5

Command line arguments : Before you run the program, you can determine the number of reconstructed fragments by setting argv value. If you leave it empty, program will run until reconstructing all fragments.

	argv = empty 

### Keyboard events
To visualize the results, several keyboard events are defined below. 

|Keyboard|Description|
|:---:|:---|
|'spacebar'| To show first ranked result|
|'->'| Move on lower ranked result|
|'<-'|Move on higer ranked result|
|'o' |Turn on/off OBJ
|'s'|Save result

Pressing 's' button will save current state of data at "../ICCV Data/Result/" including *edge-line* and *obj*. 

### Save Log file
As visualizing result, two log files automatically are saved at "../ICCV Data/Graph Log/".

1. RESULT_Top_x_history_year_day_hour_min.txt
2. RESULT_Top_x_Graph_x_year_day_hour_min.csv

First 'txt' file includes overall history of reconstructed sherd's order. Second 'csv' file includes transformation matrix information. 


## Bug Report
Please raise an issue on Github for issues related to this code. If you have any questions related about the code feel free to send an email to here (yoosj@umd.edu). 
