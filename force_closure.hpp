#ifndef FORCE_CLOSURE_HPP
#define FORCE_CLOSURE_HPP

#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "dhMesh.h"
#include "dhSkeletalSubspaceDeformation.h"


void prepare_forceClosure(vector<vector<QString>>& MP,  vector<vector<QString>>& color_def,
                          vector<vector<QString>>& area_to_bone, double& coef, int age);

dhMat44 RotateAroundAxis(dhVec3 axis, double theta);

void GetJacobianBones(dhArmature* arm, vector<QString> bones, vector<QString> contact_bones,
                      vector<QString>& JacobianBones);

vector<vector<double>> ComputeFrictionMatrix(segment* segm, double friction_coefficient,vector<int> force_areas,
                                             dhSkeletalSubspaceDeformation *bodySSD);

vector<vector<double>> ComputeGraspMatrix(segment* segm, dhVec3 object_center, vector<int> force_areas,
                                          dhSkeletalSubspaceDeformation* bodySSD);

vector<double> GetBoundMatrix(segment* segm, vector<int> contact_areas, dhSkeletalSubspaceDeformation* bodySSD,
                              vector<vector<QString>> ObjPs_normal);

vector<vector<double>> ComputeContactJacobian(dhArmature* arm, map<QString, int> bone_index ,
                                              vector<QString> JacobianBones, vector<vector<int>> DoFs,
                                              vector<int> force_areas, segment* segm, map<int,QString> atb,
                                              dhSkeletalSubspaceDeformation *bodySSD);


vector<vector<double>> GetMomentArm_Force(vector<vector<QString>> MP, vector<QString> JacobianBones,
                                          vector<vector<int>> DoFs, map<QString, int> bone_index);

void Adapt_to_glpk1(vector<vector<double>> G, vector<vector<double>> F, vector<double> eT,
                   vector<vector<double>>& left, vector<double>& right);

void Adapt_to_glpk2(vector<double> mg, vector<vector<double>> G, vector<vector<double>> J,
                    vector<vector<QString>> MP, vector<vector<double>> MF, vector<vector<double>> F,
                    vector<vector<double>>& left, vector<double>& right, vector<double>& coef);

double GLPK_solve_LP1(vector<vector<double>> left, vector<double> right, vector<vector<double>> G,
                  vector<double> coef);

double GLPK_solve_LP2(vector<vector<double>> left, vector<double> right, vector<double> coef,
                      vector<vector<double>> G, vector<vector<double>> J,vector<vector<double>> MF);

double forceClosure_eval(dhArmature* arm, dhSkeletalSubspaceDeformation* bodySSD,
                         dhMesh* objMesh, vector<vector<QString>> ObjPs_normal,
                         vector<vector<QString>> MP, vector<vector<QString>> color_def,
                         vector<vector<QString>> area_to_bone, dhPointCloudAsVertexRef* &bodyPoints,
                         double coef, vector<vector<QString>> input_set);

#endif // FORCE_CLOSURE_HPP
