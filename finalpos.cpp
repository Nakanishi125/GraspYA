#include "finalpos.h"
#include "dhMath.h"
#include "rom_eval.hpp"
#include "coordinate_eval.hpp"
#include "collision_eval.hpp"

//#include <gsl/gsl_math.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>

#include <math.h>
#include <vector>
#include <cmath>

using namespace std;

const double Pi = 3.14159265;

vector<double> Rot2Euler(const dhMath::dhMat33 Mat){

    double thetaX_0,thetaY_0,thetaZ_0;
    double deg_X,deg_Y,deg_Z;
    vector<double> angle;

    thetaY_0 = -asin(Mat[6]);
    thetaX_0 = atan2(Mat[7],Mat[8]);
    thetaZ_0 = atan2(Mat[3],Mat[0]);

    deg_X = thetaX_0/Pi*180;
    deg_Y = thetaY_0/Pi*180;
    deg_Z = thetaZ_0/Pi*180;

    angle.push_back(deg_X);
    angle.push_back(deg_Y);
    angle.push_back(deg_Z);

    return angle;
}

dhMath::dhMat33 Euler2Rot(const vector<double> angle){

    dhMat33 RotX(1,            0,             0,
                 0,cos(angle[0]),-sin(angle[0]),
                 0,sin(angle[0]), cos(angle[0]));

    dhMat33 RotY( cos(angle[1]),0,sin(angle[1]),
                  0,            1,            0,
                 -sin(angle[1]),0,cos(angle[1]));

    dhMat33 RotZ(cos(angle[2]),-sin(angle[2]),0,
                 sin(angle[2]), cos(angle[2]),0,
                 0            ,0             ,1);

    dhMat33 Rot = RotZ*RotY;
    Rot = Rot*RotX;

    return Rot;
}

double func_estimate(const gsl_vector *v,void *params){
    Parameter *dp = (Parameter*)params;
    estimate_armature_change(v,dp->arm, dp->Fp, dp->mesh1, dp->mesh2);
    //　評価関数定義
    double estimate_func = dp->par1*coord_eval(dp->Fp) + dp->par2*rom_evaluation(dp->arm)
                            + dp->par3*handcollision_eval(dp->mesh1, dp->mesh2, dp->arm);

    return estimate_func;
}

void estimate_armature_change(const gsl_vector *v, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2){

    map<int,QString> bone_list;
    bone_list[0] = "ROOT";     bone_list[1] = "TMCP";   bone_list[2] = "TPP";
    bone_list[3] = "TDP";      bone_list[4] = "IPP";    bone_list[5] = "IMP";
    bone_list[6] = "IDP";      bone_list[7] = "MPP";    bone_list[8] = "MMP";
    bone_list[9] = "MDP";      bone_list[10] = "RMCP";  bone_list[11] = "RPP";
    bone_list[12] = "RMP";     bone_list[13] = "RDP";   bone_list[14] = "PMCP";
    bone_list[15] = "PPP";     bone_list[16] = "PMP";   bone_list[17] = "PDP";

    vector<double> angle_deg;
    vector<double> vec_t;
    for(int l=0; l<bone_list.size(); l++){
        if(bone_list[l] == "ROOT"){
            angle_deg = { gsl_vector_get(v,0), gsl_vector_get(v,1), gsl_vector_get(v,2) };
            vec_t = { gsl_vector_get(v,3), gsl_vector_get(v,4), gsl_vector_get(v,5) };
        }
        else if(bone_list[l] == "TMCP"){
            angle_deg = { gsl_vector_get(v,6), gsl_vector_get(v,7), gsl_vector_get(v,8) };
        }
        else if(bone_list[l] == "TPP"){
            angle_deg = { gsl_vector_get(v,9), 0, 0 };
        }
        else if(bone_list[l] == "TDP"){
            angle_deg = { gsl_vector_get(v,10), 0, 0 };
        }
        else if(bone_list[l] == "IPP"){
            angle_deg = { gsl_vector_get(v,11), gsl_vector_get(v,12), gsl_vector_get(v,13) };
        }
        else if(bone_list[l] == "IMP"){
            angle_deg = { gsl_vector_get(v,14), 0, 0 };
        }
        else if(bone_list[l] == "IDP"){
            angle_deg = { gsl_vector_get(v,15), 0, 0 };
        }
        else if(bone_list[l] == "MPP"){
            angle_deg = { gsl_vector_get(v,16), gsl_vector_get(v,17), gsl_vector_get(v,18) };
        }
        else if(bone_list[l] == "MMP"){
            angle_deg = { gsl_vector_get(v,19), 0, 0 };
        }
        else if(bone_list[l] == "MDP"){
            angle_deg = { gsl_vector_get(v,20), 0, 0 };
        }
        else if(bone_list[l] == "RMCP"){
            angle_deg = { gsl_vector_get(v,21), 0, 0 };
        }
        else if(bone_list[l] == "RPP"){
            angle_deg = { gsl_vector_get(v,22), gsl_vector_get(v,23), gsl_vector_get(v,24) };
        }
        else if(bone_list[l] == "RMP"){
            angle_deg = { gsl_vector_get(v,25), 0, 0 };
        }
        else if(bone_list[l] == "RDP"){
            angle_deg = { gsl_vector_get(v,26), 0, 0 };
        }
        else if(bone_list[l] == "PMCP"){
            angle_deg = { gsl_vector_get(v,27), 0, 0 };
        }
        else if(bone_list[l] == "PPP"){
            angle_deg = { gsl_vector_get(v,28), gsl_vector_get(v,29), gsl_vector_get(v,30) };
        }
        else if(bone_list[l] == "PMP"){
            angle_deg = { gsl_vector_get(v,31), 0, 0 };
        }
        else if(bone_list[l] == "PDP"){
            angle_deg = { gsl_vector_get(v,32), 0, 0 };
        }

        vector<double> theta = { angle_deg[0]/180*Pi, angle_deg[1]/180*Pi, angle_deg[2]/180*Pi };
        dhMat33 RotMat33 = Euler2Rot(theta);
        dhMat44 RotMat44(RotMat33);
        if(bone_list[l] == "ROOT"){
            RotMat44.p[12] = vec_t[0];
            RotMat44.p[13] = vec_t[1];
            RotMat44.p[14] = vec_t[2];
        }

        arm->bone(bone_list[l])->R = RotMat44;

        angle_deg.clear();
        theta.clear();
    }

    arm->Update();
    Fp->Update();
    mesh1->Update();
    mesh2->Update();

}
