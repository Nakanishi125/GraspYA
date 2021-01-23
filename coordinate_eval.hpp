#ifndef COORDINATE_EVAL_HPP
#define COORDINATE_EVAL_HPP

#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>
#include<cstring>
#include<QString>
#include"csv.hpp"
#include"dhMath.h"
#include"dhFeaturePoint.h"


void prepare_coordeval(dhFeaturePoints* Fp, vector<vector<QString>>& ObjPs,
                       vector<vector<QString>>& ObjPs_normal, QStringList& fpname);

float coord_eval(dhFeaturePoints* Fp, const vector<vector<QString>>& ObjPs,
                 const vector<vector<QString>>& ObjPs_normal, const QStringList& fpname);


#endif // COORDINATE_EVAL_HPP
