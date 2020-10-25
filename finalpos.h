#ifndef FINALPOS_H
#define FINALPOS_H

#include "dhMath.h"
#include "dhArmature.h"
#include "dhFeaturePoint.h"
#include "csv.hpp"
#include "Vector2D.hpp"
#include "dhBone.h"

//#include <gsl/gsl_math.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>

#include <math.h>
#include <vector>
#include <cmath>
#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>
#include<cstring>


struct Parameter{
    double par1;    double par2;    double par3;
    dhArmature* arm;    dhFeaturePoints* Fp;
    dhSkeletalSubspaceDeformation* mesh1;    dhMesh* mesh2;
};

vector<double> Rot2Euler(const dhMath::dhMat33 Mat);

double func_estimate(const gsl_vector *v,void *params);

void estimate_armature_change(const gsl_vector *v, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2);

#endif // FINALPOS_H
