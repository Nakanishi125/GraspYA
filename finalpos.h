#ifndef FINALPOS_H
#define FINALPOS_H

#include "dhMath.h"
#include "dhArmature.h"
#include "dhFeaturePoint.h"
#include "dhPointCloud.h"
#include "csv.hpp"
#include "Vector2D.hpp"
#include "dhBone.h"
#include "rom_eval.hpp"

//#include <gsl/gsl_math.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <cfloat>
#include <array>

const double pi = 3.14159265;
const double e = 2.718281828;

const int number_of_particles = 50;
const int dimensions = 33;
const int repeat_times = 70;

typedef array<array<double,dimensions>,number_of_particles> particles;

template<class T>
array<T,dimensions> operator+(const array<T,dimensions>& v1, const array<T,dimensions>& v2){
    array<T,dimensions> ans = v1;
    for(size_t i=0; i<ans.size(); i++){
        ans[i] += v2[i];
    }
    return ans;
}

template<class T>
array<T,dimensions> operator-(const array<T,dimensions>& v1, const array<T,dimensions>& v2){
    array<T,dimensions> ans = v1;
    for(size_t i=0; i<ans.size(); i++){
        ans[i] -= v2[i];
    }
    return ans;
}

template<class T>
array<T,dimensions> operator*(const double w, const array<T,dimensions>& v2){
    array<T,dimensions> ans = v2;
    for(size_t i=0; i<ans.size(); i++){
        ans[i] *= w;
    }
    return ans;
}


struct Parameter{
    double par1;    double par2;    double par3;    double par4;
    dhArmature* arm;    dhFeaturePoints* Fp;
    dhSkeletalSubspaceDeformation* ssd;    dhMesh* handMesh;    dhMesh* objMesh;
    //ROM_eval関連
    vector<vector<QString>> jl;
    vector<vector<QString>> jb;
    vector<vector<QString>> DF;
    vector<alphashape> as;
    //Coord_eval関連
    vector<vector<QString>> ObjPs;
    vector<vector<QString>> ObjPs_normal;
    QStringList fpname;
    //Collision_eval関連
    dhPointCloudAsVertexRef* bodyPoints;
    dhPointCloudAsVertexRef* objectPoints;
    dhPointCloud* internal;
    double hand_size;
    //ForceClosure関連
    vector<vector<QString>> MP;
    vector<vector<QString>> color_def;
    vector<vector<QString>> area_to_bone;
    double coef;
};

vector<double> Rot2Euler(const dhMath::dhMat33 Mat);

double func_estimate(const gsl_vector *v,void *params);

void estimate_armature_change(const gsl_vector *v, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2);

int FinalPostureCreate(dhArmature* arm,dhFeaturePoints* Fp,dhSkeletalSubspaceDeformation* ssd,
                       dhMesh* handMesh, dhMesh* objMesh, int age);

double func_estimate_PSO(const array<double,dimensions> para, Parameter pp);

void estimate_armature_change_PSO(const array<double,dimensions> para, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* ssd, dhMesh* objMesh);

void update_positions(particles& positions, particles velocities);

void update_velocities( particles positions, particles& velocities,
                        particles PersonalBest, array<double,dimensions> GlobalBest, int t,
                        const double wmax=1.3, const double wmin = 0.5,
                        const double ro1=1.5, const double ro2=1.5);

void FinalPostureCreate_PSO(dhArmature* arm,dhFeaturePoints* Fp, dhSkeletalSubspaceDeformation* ssd,
                            dhMesh* handMesh, dhMesh* objMesh, int age);

#endif // FINALPOS_H
