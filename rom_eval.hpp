#ifndef ROM_BOUNDARY_HPP
#define ROM_BOUNDARY_HPP

#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>
#include<cmath>
#include<cstring>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include"pgmlib.h"
#include"csv.hpp"
#include"Vector2D.hpp"
#include"dhArmature.h"
#include"dhMath.h"
#include"dhBone.h"

const int WIDTH = 500;      //PGM画像の横画素数
const int HEIGHT = 500;     //PGM画像の縦画素数
const int MARGIN = 5;       //PGM画像の縁のマージン
const int vertex_number = 30;   //csvにROM再構成した際の頂点数


struct alphashape{
    QString path;
    vector<Vector2D> vertices;
};

double cal_res(int age);

void restrict_ROM(vector<vector<QString>> &DF, double rst);

void csv_to_pgm(vector<vector<int>> &conv, const vector<Vector2D> ashape,
                double minx, double miny, double maxx, double maxy);

void pgm_to_csv(vector<array<int,2>> edge, vector<Vector2D> &csvize,
                double minx, double miny, double maxx, double maxy, int split_csv);

void ConnectingPoints(vector<vector<int>> conv, PGM *rom);

vector<alphashape> restrict_ashape(vector<alphashape>& ashape_all, double rst);

void prepare_romeval(vector<vector<QString>>& joints_list, vector<vector<QString>>& joint_bone,
                     vector<vector<QString>>& DF, vector<alphashape>& ashape_rst_all, int age);

double calc_distance(std::vector<std::vector<float>> xy, Vector2D pos_pos);

void in_or_out(std::vector<std::vector<float>> xy, Vector2D pos_pos,int& inside_num,int& in_out);

double rom_eval(dhArmature* arm, vector<vector<QString>> joints_list,
                vector<vector<QString>> joint_bone, vector<vector<QString>> DF,
                vector<alphashape> ashape_all);


#endif
