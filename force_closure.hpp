#ifndef FORCE_CLOSURE_HPP
#define FORCE_CLOSURE_HPP

#include<vector>
#include"dhMesh.h"
#include"dhSkeletalSubspaceDeformation.h"


void prepare_forceClosure(vector<vector<QString>>& MP, vector<vector<QString>>& color_def,
                          vector<vector<QString>>& area_to_bone, double& coef, int age);

dhMat44 RotateAroundAxis(dhVec3 axis, double theta);

void GetJacobianBones(dhArmature* arm, vector<QString> bones, vector<QString> contact_bones,
                      vector<QString>& JacobianBones);

vector<vector<double>> ComputeFrictionMatrix(segment* segm, vector<int> contact_areas, double friction_coefficient);

vector<vector<double>> ComputeGraspMatrix(segment* segm, vector<int> contact_areas, dhVec3 object_center);

vector<vector<double>> ComputeContactJacobian(dhArmature* arm, map<QString, int> bone_index ,
                                              vector<QString> JacobianBones, vector<vector<int>> DoFs,
                                              vector<QString> contact_bones, vector<int> contact_areas,
                                              segment* segm);


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
                      vector<vector<double>> G, vector<vector<double>> J);

double forceClosure_eval(dhArmature* arm, dhSkeletalSubspaceDeformation* bodySSD,
                         dhMesh* bodyMesh, dhMesh* objMesh,
                         vector<vector<QString>> MP, vector<vector<QString>> color_def,
                         vector<vector<QString>> area_to_bone, dhPointCloudAsVertexRef* &bodyPoints,
                         double coef);

#endif // FORCE_CLOSURE_HPP
