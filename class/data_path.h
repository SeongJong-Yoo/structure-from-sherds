#pragma once
#ifndef _DATA_PATH_H_
#define _DATA_PATH_H_

#define ICCV_POT_A
//#define ICCV_POT_B
//#define ICCV_POT_C
//#define ICCV_POT_D
//#define ICCV_POT_E
//#define ICCV_POT_A_B
//#define ICCV_POT_A_B_C
//#define ICCV_POT_D_E
//#define ICCV_POT_All
using namespace std;

string path = "../ICCV Data/";

//############################################ ICCV Pottery A ############################################//
#ifdef ICCV_POT_A
#define SHARD_NUMBER 8

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_A_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_08_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_A_Piece_01_Mesh.obj",
	path + "Mesh/Pot_A_Piece_02_Mesh.obj",
	path + "Mesh/Pot_A_Piece_03_Mesh.obj",
	path + "Mesh/Pot_A_Piece_04_Mesh.obj",
	path + "Mesh/Pot_A_Piece_05_Mesh.obj",
	path + "Mesh/Pot_A_Piece_06_Mesh.obj",
	path + "Mesh/Pot_A_Piece_07_Mesh.obj",
	path + "Mesh/Pot_A_Piece_08_Mesh.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_A_Piece_01_Axis.xyz",
	path + "Axes/Pot_A_Piece_02_Axis.xyz",
	path + "Axes/Pot_A_Piece_03_Axis.xyz",
	path + "Axes/Pot_A_Piece_04_Axis.xyz",
	path + "Axes/Pot_A_Piece_05_Axis.xyz",
	path + "Axes/Pot_A_Piece_06_Axis.xyz",
	path + "Axes/Pot_A_Piece_07_Axis.xyz",
	path + "Axes/Pot_A_Piece_08_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_1.xyz"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1 
	true,   // 2 
	true,	// 3 
	true,   // 4 
	true,   // 5 
	true,   // 6 
	true,   // 7 
	true,   // 8 
};
#endif

//############################################ ICCV Pottery B ############################################//
#ifdef ICCV_POT_B
#define SHARD_NUMBER 9

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_B_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_09_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_B_Piece_01_Mesh.obj",
	path + "Mesh/Pot_B_Piece_02_Mesh.obj",
	path + "Mesh/Pot_B_Piece_03_Mesh.obj",
	path + "Mesh/Pot_B_Piece_04_Mesh.obj",
	path + "Mesh/Pot_B_Piece_05_Mesh.obj",
	path + "Mesh/Pot_B_Piece_06_Mesh.obj",
	path + "Mesh/Pot_B_Piece_07_Mesh.obj",
	path + "Mesh/Pot_B_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_09_Mesh.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_B_Piece_01_Axis.xyz",
	path + "Axes/Pot_B_Piece_02_Axis.xyz",
	path + "Axes/Pot_B_Piece_03_Axis.xyz",
	path + "Axes/Pot_B_Piece_04_Axis.xyz",
	path + "Axes/Pot_B_Piece_05_Axis.xyz",
	path + "Axes/Pot_B_Piece_06_Axis.xyz",
	path + "Axes/Pot_B_Piece_07_Axis.xyz",
	path + "Axes/Pot_B_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_09_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_B_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_B_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_1.xyz"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1
	true,	// 2
	true,	// 3
	true,	// 4
	true,	// 5
	true,	// 6
	true,	// 7
	true,	// 8
	true	// 9
};
#endif

//############################################ ICCV Pottery C ############################################//
#ifdef ICCV_POT_C
#define SHARD_NUMBER 4

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_C_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_04_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_C_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_04_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_C_Piece_01_Axis.xyz",
	path + "Axes/Pot_C_Piece_02_Axis.xyz",
	path + "Axes/Pot_C_Piece_03_Axis.xyz",
	path + "Axes/Pot_C_Piece_04_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_C_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_C_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_1.xyz"
};

string surface_fr[SHARD_NUMBER] = {
	path + "Surfaces/Pot_C_Piece_01_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_02_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_03_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_04_Surface_0_FracturedSurfacePts.pcd"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1	
	true,   // 2	
	true,	// 3		
	true	// 4		
};
#endif

//############################################ ICCV Pottery D ############################################//
#ifdef ICCV_POT_D
#define SHARD_NUMBER 29

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_D_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_29_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_D_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_29_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_D_Piece_01_Axis.xyz",
	path + "Axes/Pot_D_Piece_02_Axis.xyz",
	path + "Axes/Pot_D_Piece_03_Axis.xyz",
	path + "Axes/Pot_D_Piece_04_Axis.xyz",
	path + "Axes/Pot_D_Piece_05_Axis.xyz",
	path + "Axes/Pot_D_Piece_06_Axis.xyz",
	path + "Axes/Pot_D_Piece_07_Axis.xyz",
	path + "Axes/Pot_D_Piece_08_Axis.xyz",
	path + "Axes/Pot_D_Piece_09_Axis.xyz",
	path + "Axes/Pot_D_Piece_10_Axis.xyz",
	path + "Axes/Pot_D_Piece_11_Axis.xyz",
	path + "Axes/Pot_D_Piece_12_Axis.xyz",
	path + "Axes/Pot_D_Piece_13_Axis.xyz",
	path + "Axes/Pot_D_Piece_14_Axis.xyz",
	path + "Axes/Pot_D_Piece_15_Axis.xyz",
	path + "Axes/Pot_D_Piece_16_Axis.xyz",
	path + "Axes/Pot_D_Piece_17_Axis.xyz",
	path + "Axes/Pot_D_Piece_18_Axis.xyz",
	path + "Axes/Pot_D_Piece_19_Axis.xyz",
	path + "Axes/Pot_D_Piece_20_Axis.xyz",
	path + "Axes/Pot_D_Piece_21_Axis.xyz",
	path + "Axes/Pot_D_Piece_22_Axis.xyz",
	path + "Axes/Pot_D_Piece_23_Axis.xyz",
	path + "Axes/Pot_D_Piece_24_Axis.xyz",
	path + "Axes/Pot_D_Piece_25_Axis.xyz",
	path + "Axes/Pot_D_Piece_26_Axis.xyz",
	path + "Axes/Pot_D_Piece_27_Axis.xyz",
	path + "Axes/Pot_D_Piece_28_Axis.xyz",
	path + "Axes/Pot_D_Piece_29_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_D_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_D_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_1.xyz"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1 
	true,   // 2 
	true,	// 3 
	true,   // 4 
	true,   // 5 
	true,   // 6 
	true,   // 7 
	true,   // 8 
	true,   // 9 
	true,   // 10
	true,   // 11
	true,	// 12 
	true,   // 13
	true,	// 14 
	true,   // 15
	true,   // 16
	true,   // 17
	true,   // 18
	true,   // 19
	true,   // 20
	true,   // 21
	true,   // 22
	true,   // 23
	true,   // 24
	true,   // 25
	true,   // 26
	true,   // 27
	false,   // 28
	true   // 29
};
#endif

//############################################ ICCV Pottery E ############################################//
#ifdef ICCV_POT_E
#define SHARD_NUMBER 31

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_E_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_29_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_30_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_31_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_E_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_29_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_30_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_31_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_E_Piece_01_Axis.xyz",
	path + "Axes/Pot_E_Piece_02_Axis.xyz",
	path + "Axes/Pot_E_Piece_03_Axis.xyz",
	path + "Axes/Pot_E_Piece_04_Axis.xyz",
	path + "Axes/Pot_E_Piece_05_Axis.xyz",
	path + "Axes/Pot_E_Piece_06_Axis.xyz",
	path + "Axes/Pot_E_Piece_07_Axis.xyz",
	path + "Axes/Pot_E_Piece_08_Axis.xyz",
	path + "Axes/Pot_E_Piece_09_Axis.xyz",
	path + "Axes/Pot_E_Piece_10_Axis.xyz",
	path + "Axes/Pot_E_Piece_11_Axis.xyz",
	path + "Axes/Pot_E_Piece_12_Axis.xyz",
	path + "Axes/Pot_E_Piece_13_Axis.xyz",
	path + "Axes/Pot_E_Piece_14_Axis.xyz",
	path + "Axes/Pot_E_Piece_15_Axis.xyz",
	path + "Axes/Pot_E_Piece_16_Axis.xyz",
	path + "Axes/Pot_E_Piece_17_Axis.xyz",
	path + "Axes/Pot_E_Piece_18_Axis.xyz",
	path + "Axes/Pot_E_Piece_19_Axis.xyz",
	path + "Axes/Pot_E_Piece_20_Axis.xyz",
	path + "Axes/Pot_E_Piece_21_Axis.xyz",
	path + "Axes/Pot_E_Piece_22_Axis.xyz",
	path + "Axes/Pot_E_Piece_23_Axis.xyz",
	path + "Axes/Pot_E_Piece_24_Axis.xyz",
	path + "Axes/Pot_E_Piece_25_Axis.xyz",
	path + "Axes/Pot_E_Piece_26_Axis.xyz",
	path + "Axes/Pot_E_Piece_27_Axis.xyz",
	path + "Axes/Pot_E_Piece_28_Axis.xyz",
	path + "Axes/Pot_E_Piece_29_Axis.xyz",
	path + "Axes/Pot_E_Piece_30_Axis.xyz",
	path + "Axes/Pot_E_Piece_31_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_E_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_E_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_1.xyz"
};

string surface_fr[SHARD_NUMBER] = {
	path + "Surfaces/Pot_E_Piece_01_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_02_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_03_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_04_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_05_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_06_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_07_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_08_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_09_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_10_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_11_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_12_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_13_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_14_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_15_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_16_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_17_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_18_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_19_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_20_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_21_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_22_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_23_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_24_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_25_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_26_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_27_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_28_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_29_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_30_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_31_Surface_0_FracturedSurfacePts.pcd"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1	Pot E		
	true,   // 2						
	true,	// 3				
	true,   // 4				
	true,   // 5				
	true,   // 6					
	true,   // 7				
	true,   // 8				
	true,   // 9				
	true,   // 10				
	true,   // 11				
	true,	// 12				
	true,   // 13				
	true,	// 14				
	true,   // 15				
	true,   // 16				
	true,   // 17				
	true,   // 18				
	true,   // 19				
	true,   // 20				
	true,   // 21				
	true,   // 22				
	true,   // 23				
	true,   // 24				
	true,   // 25				
	true,   // 26				
	true,   // 27				
	true,	// 28				
	true,	// 29
	true,	// 30				
	true	// 31				
};

#endif

//############################################ ICCV Pottery A + B ############################################//
#ifdef ICCV_POT_A_B
#define SHARD_NUMBER 17

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_A_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_09_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_A_Piece_01_Mesh.obj",
	path + "Mesh/Pot_A_Piece_02_Mesh.obj",
	path + "Mesh/Pot_A_Piece_03_Mesh.obj",
	path + "Mesh/Pot_A_Piece_04_Mesh.obj",
	path + "Mesh/Pot_A_Piece_05_Mesh.obj",
	path + "Mesh/Pot_A_Piece_06_Mesh.obj",
	path + "Mesh/Pot_A_Piece_07_Mesh.obj",
	path + "Mesh/Pot_A_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_01_Mesh.obj",
	path + "Mesh/Pot_B_Piece_02_Mesh.obj",
	path + "Mesh/Pot_B_Piece_03_Mesh.obj",
	path + "Mesh/Pot_B_Piece_04_Mesh.obj",
	path + "Mesh/Pot_B_Piece_05_Mesh.obj",
	path + "Mesh/Pot_B_Piece_06_Mesh.obj",
	path + "Mesh/Pot_B_Piece_07_Mesh.obj",
	path + "Mesh/Pot_B_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_09_Mesh.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_A_Piece_01_Axis.xyz",
	path + "Axes/Pot_A_Piece_02_Axis.xyz",
	path + "Axes/Pot_A_Piece_03_Axis.xyz",
	path + "Axes/Pot_A_Piece_04_Axis.xyz",
	path + "Axes/Pot_A_Piece_05_Axis.xyz",
	path + "Axes/Pot_A_Piece_06_Axis.xyz",
	path + "Axes/Pot_A_Piece_07_Axis.xyz",
	path + "Axes/Pot_A_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_01_Axis.xyz",
	path + "Axes/Pot_B_Piece_02_Axis.xyz",
	path + "Axes/Pot_B_Piece_03_Axis.xyz",
	path + "Axes/Pot_B_Piece_04_Axis.xyz",
	path + "Axes/Pot_B_Piece_05_Axis.xyz",
	path + "Axes/Pot_B_Piece_06_Axis.xyz",
	path + "Axes/Pot_B_Piece_07_Axis.xyz",
	path + "Axes/Pot_B_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_09_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_1.xyz"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1  Pot A
	true,   // 2 
	true,	// 3 
	true,   // 4 
	true,   // 5 
	true,   // 6 
	true,   // 7 
	true,   // 8 
	true,	// 1  Pot B
	true,	// 2
	true,	// 3
	true,	// 4
	true,	// 5
	true,	// 6
	true,	// 7
	true,	// 8
	true	// 9
};
#endif

//############################################ ICCV Pottery D + E ############################################//
#ifdef ICCV_POT_D_E
#define SHARD_NUMBER 60

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_D_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_29_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_29_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_30_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_31_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_D_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_29_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_29_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_30_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_31_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_D_Piece_01_Axis.xyz",
	path + "Axes/Pot_D_Piece_02_Axis.xyz",
	path + "Axes/Pot_D_Piece_03_Axis.xyz",
	path + "Axes/Pot_D_Piece_04_Axis.xyz",
	path + "Axes/Pot_D_Piece_05_Axis.xyz",
	path + "Axes/Pot_D_Piece_06_Axis.xyz",
	path + "Axes/Pot_D_Piece_07_Axis.xyz",
	path + "Axes/Pot_D_Piece_08_Axis.xyz",
	path + "Axes/Pot_D_Piece_09_Axis.xyz",
	path + "Axes/Pot_D_Piece_10_Axis.xyz",
	path + "Axes/Pot_D_Piece_11_Axis.xyz",
	path + "Axes/Pot_D_Piece_12_Axis.xyz",
	path + "Axes/Pot_D_Piece_13_Axis.xyz",
	path + "Axes/Pot_D_Piece_14_Axis.xyz",
	path + "Axes/Pot_D_Piece_15_Axis.xyz",
	path + "Axes/Pot_D_Piece_16_Axis.xyz",
	path + "Axes/Pot_D_Piece_17_Axis.xyz",
	path + "Axes/Pot_D_Piece_18_Axis.xyz",
	path + "Axes/Pot_D_Piece_19_Axis.xyz",
	path + "Axes/Pot_D_Piece_20_Axis.xyz",
	path + "Axes/Pot_D_Piece_21_Axis.xyz",
	path + "Axes/Pot_D_Piece_22_Axis.xyz",
	path + "Axes/Pot_D_Piece_23_Axis.xyz",
	path + "Axes/Pot_D_Piece_24_Axis.xyz",
	path + "Axes/Pot_D_Piece_25_Axis.xyz",
	path + "Axes/Pot_D_Piece_26_Axis.xyz",
	path + "Axes/Pot_D_Piece_27_Axis.xyz",
	path + "Axes/Pot_D_Piece_28_Axis.xyz",
	path + "Axes/Pot_D_Piece_29_Axis.xyz",
	path + "Axes/Pot_E_Piece_01_Axis.xyz",
	path + "Axes/Pot_E_Piece_02_Axis.xyz",
	path + "Axes/Pot_E_Piece_03_Axis.xyz",
	path + "Axes/Pot_E_Piece_04_Axis.xyz",
	path + "Axes/Pot_E_Piece_05_Axis.xyz",
	path + "Axes/Pot_E_Piece_06_Axis.xyz",
	path + "Axes/Pot_E_Piece_07_Axis.xyz",
	path + "Axes/Pot_E_Piece_08_Axis.xyz",
	path + "Axes/Pot_E_Piece_09_Axis.xyz",
	path + "Axes/Pot_E_Piece_10_Axis.xyz",
	path + "Axes/Pot_E_Piece_11_Axis.xyz",
	path + "Axes/Pot_E_Piece_12_Axis.xyz",
	path + "Axes/Pot_E_Piece_13_Axis.xyz",
	path + "Axes/Pot_E_Piece_14_Axis.xyz",
	path + "Axes/Pot_E_Piece_15_Axis.xyz",
	path + "Axes/Pot_E_Piece_16_Axis.xyz",
	path + "Axes/Pot_E_Piece_17_Axis.xyz",
	path + "Axes/Pot_E_Piece_18_Axis.xyz",
	path + "Axes/Pot_E_Piece_19_Axis.xyz",
	path + "Axes/Pot_E_Piece_20_Axis.xyz",
	path + "Axes/Pot_E_Piece_21_Axis.xyz",
	path + "Axes/Pot_E_Piece_22_Axis.xyz",
	path + "Axes/Pot_E_Piece_23_Axis.xyz",
	path + "Axes/Pot_E_Piece_24_Axis.xyz",
	path + "Axes/Pot_E_Piece_25_Axis.xyz",
	path + "Axes/Pot_E_Piece_26_Axis.xyz",
	path + "Axes/Pot_E_Piece_27_Axis.xyz",
	path + "Axes/Pot_E_Piece_28_Axis.xyz",
	path + "Axes/Pot_E_Piece_29_Axis.xyz",
	path + "Axes/Pot_E_Piece_30_Axis.xyz",
	path + "Axes/Pot_E_Piece_31_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_D_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_D_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_1.xyz"
};

string surface_fr[SHARD_NUMBER] = {
	path + "Surfaces/Pot_D_Piece_01_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_02_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_03_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_04_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_05_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_06_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_07_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_08_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_09_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_10_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_11_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_12_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_13_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_14_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_15_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_16_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_17_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_18_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_19_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_20_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_21_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_22_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_23_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_24_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_25_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_26_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_27_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_28_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_D_Piece_29_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_01_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_02_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_03_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_04_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_05_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_06_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_07_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_08_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_09_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_10_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_11_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_12_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_13_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_14_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_15_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_16_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_17_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_18_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_19_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_20_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_21_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_22_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_23_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_24_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_25_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_26_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_27_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_28_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_29_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_30_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_E_Piece_31_Surface_0_FracturedSurfacePts.pcd"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1 Pot D
	true,   // 2 
	true,	// 3 			
	true,   // 4	
	true,   // 5 
	true,   // 6 
	true,   // 7 	
	true,   // 8 
	true,   // 9	
	true,   // 10 
	true,   // 11 	
	true,	// 12 	
	true,   // 13 
	true,	// 14 		
	true,   // 15
	true,   // 16
	true,   // 17	
	true,   // 18 	
	true,   // 19 
	true,   // 20 		
	true,   // 21 		
	true,   // 22 		
	true,   // 23 
	true,   // 24
	true,   // 25 
	true,   // 26 
	true,   // 27
	false,   // 28 	
	true,   // 29				
	true,	// 1	Pot E	30	
	true,   // 2			31	
	true,	// 3			32	
	true,   // 4			33	
	true,   // 5			34	
	true,   // 6			35	
	true,   // 7			36	
	true,   // 8			37	
	true,   // 9			38	
	true,   // 10			39	
	true,   // 11			40		
	true,	// 12			41	
	true,   // 13			42	
	true,	// 14			43	
	true,   // 15			44	
	true,   // 16			45	
	true,   // 17			46	
	true,   // 18			47	
	true,   // 19			48	
	true,   // 20			49	
	true,   // 21			50	
	true,   // 22			51	
	true,   // 23			52	
	true,   // 24			53	
	true,   // 25			54	
	true,   // 26			55	
	true,   // 27			56	
	true,	// 28			57	
	true,   // 29			58	
	true,	// 30			59	
	true   // 31			60		
};

#endif

//############################################ ICCV Pottery A + B + C ############################################//
#ifdef ICCV_POT_A_B_C
#define SHARD_NUMBER 21

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_A_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_04_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_A_Piece_01_Mesh.obj",
	path + "Mesh/Pot_A_Piece_02_Mesh.obj",
	path + "Mesh/Pot_A_Piece_03_Mesh.obj",
	path + "Mesh/Pot_A_Piece_04_Mesh.obj",
	path + "Mesh/Pot_A_Piece_05_Mesh.obj",
	path + "Mesh/Pot_A_Piece_06_Mesh.obj",
	path + "Mesh/Pot_A_Piece_07_Mesh.obj",
	path + "Mesh/Pot_A_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_01_Mesh.obj",
	path + "Mesh/Pot_B_Piece_02_Mesh.obj",
	path + "Mesh/Pot_B_Piece_03_Mesh.obj",
	path + "Mesh/Pot_B_Piece_04_Mesh.obj",
	path + "Mesh/Pot_B_Piece_05_Mesh.obj",
	path + "Mesh/Pot_B_Piece_06_Mesh.obj",
	path + "Mesh/Pot_B_Piece_07_Mesh.obj",
	path + "Mesh/Pot_B_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_09_Mesh.obj",
	path + "Mesh/Pot_C_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_04_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_A_Piece_01_Axis.xyz",
	path + "Axes/Pot_A_Piece_02_Axis.xyz",
	path + "Axes/Pot_A_Piece_03_Axis.xyz",
	path + "Axes/Pot_A_Piece_04_Axis.xyz",
	path + "Axes/Pot_A_Piece_05_Axis.xyz",
	path + "Axes/Pot_A_Piece_06_Axis.xyz",
	path + "Axes/Pot_A_Piece_07_Axis.xyz",
	path + "Axes/Pot_A_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_01_Axis.xyz",
	path + "Axes/Pot_B_Piece_02_Axis.xyz",
	path + "Axes/Pot_B_Piece_03_Axis.xyz",
	path + "Axes/Pot_B_Piece_04_Axis.xyz",
	path + "Axes/Pot_B_Piece_05_Axis.xyz",
	path + "Axes/Pot_B_Piece_06_Axis.xyz",
	path + "Axes/Pot_B_Piece_07_Axis.xyz",
	path + "Axes/Pot_B_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_09_Axis.xyz",
	path + "Axes/Pot_C_Piece_01_Axis.xyz",
	path + "Axes/Pot_C_Piece_02_Axis.xyz",
	path + "Axes/Pot_C_Piece_03_Axis.xyz",
	path + "Axes/Pot_C_Piece_04_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_1.xyz"
};

string surface_fr[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_02.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_03.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_04.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_05.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_06.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_07.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_A_Piece_08.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_01.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_02.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_03.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_04.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_05.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_06.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_07.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_08.obj_AllPts_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_B_Piece_09.obj_samples_Int_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_01_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_02_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_03_Surface_0_FracturedSurfacePts.pcd",
	path + "Surfaces/Pot_C_Piece_04_Surface_0_FracturedSurfacePts.pcd"
};


bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1	PotA
	true,   // 2 
	true,	// 3 
	true,   // 4 
	true,   // 5 
	true,   // 6 
	true,   // 7	
	true,   // 8	
	true,	// 1	PotB
	true,	// 2
	true,	// 3
	true,	// 4
	true,	// 5
	true,	// 6
	true,	// 7
	true,	// 8
	true,	// 9
	true,	// 1	PotC		
	true,   // 2					
	true,	// 3					
	true	// 4					
};
#endif

//############################################ ICCV Pottery All ############################################//
#ifdef ICCV_POT_All
#define SHARD_NUMBER 81

string file_path[SHARD_NUMBER] = {
	path + "Breaklines/Pot_A_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_A_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_B_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_D_Piece_29_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_04_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_05_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_06_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_07_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_08_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_09_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_10_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_11_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_12_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_13_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_14_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_15_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_16_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_17_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_18_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_19_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_20_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_21_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_22_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_23_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_24_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_25_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_26_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_27_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_28_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_29_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_30_Breakline_0.pcd",
	path + "Breaklines/Pot_E_Piece_31_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_01_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_02_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_03_Breakline_0.pcd",
	path + "Breaklines/Pot_C_Piece_04_Breakline_0.pcd"
};

string obj_path[SHARD_NUMBER] = {
	path + "Mesh/Pot_A_Piece_01_Mesh.obj",
	path + "Mesh/Pot_A_Piece_02_Mesh.obj",
	path + "Mesh/Pot_A_Piece_03_Mesh.obj",
	path + "Mesh/Pot_A_Piece_04_Mesh.obj",
	path + "Mesh/Pot_A_Piece_05_Mesh.obj",
	path + "Mesh/Pot_A_Piece_06_Mesh.obj",
	path + "Mesh/Pot_A_Piece_07_Mesh.obj",
	path + "Mesh/Pot_A_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_01_Mesh.obj",
	path + "Mesh/Pot_B_Piece_02_Mesh.obj",
	path + "Mesh/Pot_B_Piece_03_Mesh.obj",
	path + "Mesh/Pot_B_Piece_04_Mesh.obj",
	path + "Mesh/Pot_B_Piece_05_Mesh.obj",
	path + "Mesh/Pot_B_Piece_06_Mesh.obj",
	path + "Mesh/Pot_B_Piece_07_Mesh.obj",
	path + "Mesh/Pot_B_Piece_08_Mesh.obj",
	path + "Mesh/Pot_B_Piece_09_Mesh.obj",
	path + "Mesh/Pot_D_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_D_Piece_29_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_04_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_05_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_06_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_07_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_08_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_09_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_10_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_11_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_12_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_13_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_14_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_15_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_16_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_17_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_18_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_19_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_20_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_21_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_22_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_23_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_24_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_25_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_26_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_27_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_28_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_29_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_30_Mesh_DS.obj",
	path + "Mesh/Pot_E_Piece_31_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_01_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_02_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_03_Mesh_DS.obj",
	path + "Mesh/Pot_C_Piece_04_Mesh_DS.obj"
};

string axis_path[SHARD_NUMBER] = {
	path + "Axes/Pot_A_Piece_01_Axis.xyz",
	path + "Axes/Pot_A_Piece_02_Axis.xyz",
	path + "Axes/Pot_A_Piece_03_Axis.xyz",
	path + "Axes/Pot_A_Piece_04_Axis.xyz",
	path + "Axes/Pot_A_Piece_05_Axis.xyz",
	path + "Axes/Pot_A_Piece_06_Axis.xyz",
	path + "Axes/Pot_A_Piece_07_Axis.xyz",
	path + "Axes/Pot_A_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_01_Axis.xyz",
	path + "Axes/Pot_B_Piece_02_Axis.xyz",
	path + "Axes/Pot_B_Piece_03_Axis.xyz",
	path + "Axes/Pot_B_Piece_04_Axis.xyz",
	path + "Axes/Pot_B_Piece_05_Axis.xyz",
	path + "Axes/Pot_B_Piece_06_Axis.xyz",
	path + "Axes/Pot_B_Piece_07_Axis.xyz",
	path + "Axes/Pot_B_Piece_08_Axis.xyz",
	path + "Axes/Pot_B_Piece_09_Axis.xyz",
	path + "Axes/Pot_D_Piece_01_Axis.xyz",
	path + "Axes/Pot_D_Piece_02_Axis.xyz",
	path + "Axes/Pot_D_Piece_03_Axis.xyz",
	path + "Axes/Pot_D_Piece_04_Axis.xyz",
	path + "Axes/Pot_D_Piece_05_Axis.xyz",
	path + "Axes/Pot_D_Piece_06_Axis.xyz",
	path + "Axes/Pot_D_Piece_07_Axis.xyz",
	path + "Axes/Pot_D_Piece_08_Axis.xyz",
	path + "Axes/Pot_D_Piece_09_Axis.xyz",
	path + "Axes/Pot_D_Piece_10_Axis.xyz",
	path + "Axes/Pot_D_Piece_11_Axis.xyz",
	path + "Axes/Pot_D_Piece_12_Axis.xyz",
	path + "Axes/Pot_D_Piece_13_Axis.xyz",
	path + "Axes/Pot_D_Piece_14_Axis.xyz",
	path + "Axes/Pot_D_Piece_15_Axis.xyz",
	path + "Axes/Pot_D_Piece_16_Axis.xyz",
	path + "Axes/Pot_D_Piece_17_Axis.xyz",
	path + "Axes/Pot_D_Piece_18_Axis.xyz",
	path + "Axes/Pot_D_Piece_19_Axis.xyz",
	path + "Axes/Pot_D_Piece_20_Axis.xyz",
	path + "Axes/Pot_D_Piece_21_Axis.xyz",
	path + "Axes/Pot_D_Piece_22_Axis.xyz",
	path + "Axes/Pot_D_Piece_23_Axis.xyz",
	path + "Axes/Pot_D_Piece_24_Axis.xyz",
	path + "Axes/Pot_D_Piece_25_Axis.xyz",
	path + "Axes/Pot_D_Piece_26_Axis.xyz",
	path + "Axes/Pot_D_Piece_27_Axis.xyz",
	path + "Axes/Pot_D_Piece_28_Axis.xyz",
	path + "Axes/Pot_D_Piece_29_Axis.xyz",
	path + "Axes/Pot_E_Piece_01_Axis.xyz",
	path + "Axes/Pot_E_Piece_02_Axis.xyz",
	path + "Axes/Pot_E_Piece_03_Axis.xyz",
	path + "Axes/Pot_E_Piece_04_Axis.xyz",
	path + "Axes/Pot_E_Piece_05_Axis.xyz",
	path + "Axes/Pot_E_Piece_06_Axis.xyz",
	path + "Axes/Pot_E_Piece_07_Axis.xyz",
	path + "Axes/Pot_E_Piece_08_Axis.xyz",
	path + "Axes/Pot_E_Piece_09_Axis.xyz",
	path + "Axes/Pot_E_Piece_10_Axis.xyz",
	path + "Axes/Pot_E_Piece_11_Axis.xyz",
	path + "Axes/Pot_E_Piece_12_Axis.xyz",
	path + "Axes/Pot_E_Piece_13_Axis.xyz",
	path + "Axes/Pot_E_Piece_14_Axis.xyz",
	path + "Axes/Pot_E_Piece_15_Axis.xyz",
	path + "Axes/Pot_E_Piece_16_Axis.xyz",
	path + "Axes/Pot_E_Piece_17_Axis.xyz",
	path + "Axes/Pot_E_Piece_18_Axis.xyz",
	path + "Axes/Pot_E_Piece_19_Axis.xyz",
	path + "Axes/Pot_E_Piece_20_Axis.xyz",
	path + "Axes/Pot_E_Piece_21_Axis.xyz",
	path + "Axes/Pot_E_Piece_22_Axis.xyz",
	path + "Axes/Pot_E_Piece_23_Axis.xyz",
	path + "Axes/Pot_E_Piece_24_Axis.xyz",
	path + "Axes/Pot_E_Piece_25_Axis.xyz",
	path + "Axes/Pot_E_Piece_26_Axis.xyz",
	path + "Axes/Pot_E_Piece_27_Axis.xyz",
	path + "Axes/Pot_E_Piece_28_Axis.xyz",
	path + "Axes/Pot_E_Piece_29_Axis.xyz",
	path + "Axes/Pot_E_Piece_30_Axis.xyz",
	path + "Axes/Pot_E_Piece_31_Axis.xyz",
	path + "Axes/Pot_C_Piece_01_Axis.xyz",
	path + "Axes/Pot_C_Piece_02_Axis.xyz",
	path + "Axes/Pot_C_Piece_03_Axis.xyz",
	path + "Axes/Pot_C_Piece_04_Axis.xyz"
};

string surface_in[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_0.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_01_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_0.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_0.xyz"
};

string surface_out[SHARD_NUMBER] = {
	path + "Surfaces/Pot_A_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_A_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_B_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_D_Piece_29_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_04_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_05_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_06_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_07_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_08_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_09_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_10_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_11_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_12_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_13_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_14_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_15_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_16_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_17_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_18_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_19_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_20_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_21_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_22_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_23_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_24_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_25_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_26_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_27_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_28_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_29_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_30_Surface_1.xyz",
	path + "Surfaces/Pot_E_Piece_31_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_01_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_02_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_03_Surface_1.xyz",
	path + "Surfaces/Pot_C_Piece_04_Surface_1.xyz"
};

bool shard_on_off[SHARD_NUMBER] = {
	true,	// 1	PotA		1
	true,   // 2				2
	true,	// 3 				3
	true,   // 4				4
	true,   // 5				5
	true,   // 6				6
	true,   // 7 				7
	true,   // 8				8
	true,	// 1	PotB		9
	true,	// 2				10
	true,	// 3				11
	true,	// 4				12
	true,	// 5				13
	true,	// 6				14		
	true,	// 7				15
	true,	// 8				16
	true,	// 9				17
	true,	// 1	PotD		18
	true,   // 2				19
	true,	// 3				20
	true,   // 4				21
	true,   // 5				22
	true,   // 6				23
	true,   // 7				24
	true,   // 8				25
	true,   // 9				26
	true,   // 10				27
	true,   // 11				28
	true,	// 12				29
	true,   // 13				30
	true,	// 14				31
	true,   // 15				32
	true,   // 16				33
	true,   // 17 				34
	true,   // 18				35
	true,   // 19				36
	true,   // 20				37	
	true,   // 21				38
	true,   // 22				39
	true,   // 23				40
	true,   // 24				41
	true,   // 25				42
	true,   // 26				43
	true,   // 27				44
	false,   // 28				45
	true,   // 29				46
	true,	// 1	PotE		47
	true,   // 2				48		
	true,	// 3				49
	true,   // 4				50
	true,   // 5				51
	true,   // 6				52	
	true,   // 7				53
	true,   // 8				54
	true,   // 9				55
	true,   // 10				56
	true,   // 11				57
	true,	// 12				58
	true,   // 13				59
	true,	// 14				60
	true,   // 15				61
	true,   // 16				62
	true,   // 17				63
	true,   // 18				64
	true,   // 19				65
	true,   // 20				66
	true,   // 21				67
	true,   // 22				68
	true,   // 23				69
	true,   // 24				70
	true,   // 25				71
	true,   // 26				72
	true,   // 27				73
	true,	// 28				74
	true,   // 29				75
	true,	// 30				76
	true,   // 31				77
	true,	// 1	PotC		78
	true,   // 2				79	
	true,	// 3				80
	true,   // 4				81
};

#endif

#endif
