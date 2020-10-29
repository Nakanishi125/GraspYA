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


double calc_distance(std::vector<std::vector<float>> xy, Vector2D pos_pos);

void in_or_out(std::vector<std::vector<float>> xy, Vector2D pos_pos,int& inside_num,int& in_out);

double rom_eval(dhArmature* arm);


#endif
