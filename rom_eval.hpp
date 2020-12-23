#ifndef ROM_BOUNDARY_HPP
#define ROM_BOUNDARY_HPP

#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>
#include<cmath>
#include<cstring>
#include"csv.hpp"
#include"Vector2D.hpp"
#include"dhArmature.h"
#include"dhMath.h"
#include"dhBone.h"

struct alphashape{
    QString path;
    vector<Vector2D> vertices;
};

double cal_res(int age);

void rest_ashape(vector<alphashape>& ashape_all, double rst);

void prepare_romeval(vector<vector<QString>>& joints_list, vector<vector<QString>>& joint_bone,
                     vector<vector<QString>>& DF, vector<alphashape>& ashape_all, int age);

double calc_distance(std::vector<std::vector<float>> xy, Vector2D pos_pos);

void in_or_out(std::vector<std::vector<float>> xy, Vector2D pos_pos,int& inside_num,int& in_out);

double rom_eval(dhArmature* arm, vector<vector<QString>> joints_list,
                vector<vector<QString>> joint_bone, vector<vector<QString>> DF,
                vector<alphashape> ashape_all);


#endif
