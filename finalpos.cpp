#include "finalpos.h"
#include "dhMath.h"
#include "rom_eval.hpp"
#include "coordinate_eval.hpp"
#include "collision_eval.hpp"
#include "force_closure.hpp"

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
#include<cstdlib>
#include<ctime>
#include<cfloat>
#include<array>

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
    estimate_armature_change(v,dp->arm, dp->Fp, dp->ssd, dp->objMesh);
    extract_contactPoints(dp->ssd, dp->internal, dp->bodyPoints, dp->objectPoints);

    //　評価関数定義
    double estimate_func =    dp->par1*coord_eval(dp->Fp, dp->ObjPs, dp->ObjPs_normal, dp->fpname)
                            + dp->par2*rom_eval(dp->arm, dp->jl, dp->jb, dp->DF, dp->as)
                            + dp->par3*collision_eval(dp->arm, dp->bodyPoints, dp->objectPoints, dp->hand_size)
                            + dp->par4*forceClosure_eval(dp->arm, dp->ssd, dp->handMesh, dp->objMesh, dp->ObjPs_normal,
                                                         dp->MP, dp->color_def, dp->area_to_bone, dp->bodyPoints,
                                                         dp->coef, dp->input_set);

    dp->bodyPoints->clearAllPoints();
    dp->objectPoints->clearAllPoints();

    return estimate_func;
}

void estimate_armature_change(const gsl_vector *v, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* ssd, dhMesh* objMesh){

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
    ssd->Update();
    objMesh->Update();

}

int FinalPostureCreate(dhArmature* arm,dhFeaturePoints* Fp, dhSkeletalSubspaceDeformation* ssd,
                       dhMesh* handMesh, dhMesh* objMesh, int age)
{
    string fn = "C:\\Users\\ynunakanishi\\Desktop\\LOG\\nelder-mead.txt";
    ofstream log(fn, ios::app);

    vector<vector<QString>> joints_list;        //ファイル類の読み込み
    vector<vector<QString>> joint_bone;
    vector<vector<QString>> DF;
    vector<alphashape> ashape_all;

    vector<vector<QString>> ObjPs;
    vector<vector<QString>> ObjPs_normal;
    QStringList fpname;

    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloud* internal = dhnew<dhPointCloud>();
    vector<vector<QString>> input_set;
    double hand_size;

    vector<vector<QString>> MP;
    vector<vector<QString>> color_def;
    vector<vector<QString>> area_to_bone;
    double coef;

    prepare_romeval(joints_list, joint_bone, DF, ashape_all, age);
    prepare_coordeval(Fp, ObjPs, ObjPs_normal, fpname);
    prepare_colleval(arm, hand_size, internal, objMesh, input_set);
    prepare_forceClosure(MP, color_def, area_to_bone, coef, age);


    map<int,QString> bone_list;
    bone_list[0] = "ROOT";     bone_list[1] = "TMCP";   bone_list[2] = "TPP";
    bone_list[3] = "TDP";      bone_list[4] = "IPP";    bone_list[5] = "IMP";
    bone_list[6] = "IDP";      bone_list[7] = "MPP";    bone_list[8] = "MMP";
    bone_list[9] = "MDP";      bone_list[10] = "RMCP";  bone_list[11] = "RPP";
    bone_list[12] = "RMP";     bone_list[13] = "RDP";   bone_list[14] = "PMCP";
    bone_list[15] = "PPP";     bone_list[16] = "PMP";   bone_list[17] = "PDP";

//ここから初期値生成
    vector<double> init;
    for(int l=0; l<bone_list.size(); l++){
        if(bone_list[l] == "ROOT"){
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
            init.push_back(angle[1]);
            init.push_back(angle[2]);
            init.push_back(trans_mat[12]);
            init.push_back(trans_mat[13]);
            init.push_back(trans_mat[14]);
        }
        else if(bone_list[l] == "TMCP" || bone_list[l] == "IPP" || bone_list[l] == "MPP"
                || bone_list[l] == "RPP" || bone_list[l] == "PPP"){
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
            init.push_back(angle[1]);
            init.push_back(angle[2]);
        }
        else{
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
        }
    }

//    for(int i=0; init.size(); i++){     //初期値確認
//        DH_LOG(QString::number(i)+": "+QString::number(init[i]),0);
//    }

//=======================
//ここからGSLによる最適化
//=======================
    size_t iter = 0;
    int status;
    double size;

//func_estimateのパラメータ(各評価関数の全変数)

    Parameter p = { 200, 1.5, 30000, 300, arm, Fp, ssd, handMesh, objMesh,
                    joints_list, joint_bone, DF, ashape_all,               //ROM
                    ObjPs, ObjPs_normal, fpname,                           //Coordinate
                    bodyPoints, objectPoints, internal, hand_size,         //Collision
                    MP, color_def, area_to_bone, coef, input_set};                    //ForceClosure

    const gsl_multimin_fminimizer_type *T;      //必要ないろいろ宣言
    gsl_multimin_fminimizer *s = NULL;
    gsl_vector *x, *ss;
    gsl_multimin_function my_func;

    ss = gsl_vector_alloc(init.size());         //初期頂点の大きさのベクトル
    gsl_vector_set_all(ss, 50);                //ステップ幅の初期値は30

    x = gsl_vector_alloc(init.size());
    for(int index=0; index<init.size(); index++){       //initの変数数繰り返して開始点代入
        gsl_vector_set(x,index,init[index]);
    }

    my_func.f = &func_estimate;
    my_func.n = x->size;
    my_func.params = (void *)(&p);

    T = gsl_multimin_fminimizer_nmsimplex;      //Nelder-meadのシンプレックス法を用いる
    s = gsl_multimin_fminimizer_alloc(T,x->size);
    gsl_multimin_fminimizer_set(s, &my_func, x, ss);

    do{
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if(status)  break;

        size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size,0.01);        //重心と各頂点の平均距離が1以内

        if(status == GSL_SUCCESS){
            DH_LOG("iter is "+QString::number(iter),0);
        }

        log << "size is :";     log << size << endl;
    }while(status == GSL_CONTINUE && iter < 30000);


    estimate_armature_change(s->x, arm, Fp, ssd, objMesh);
    dhApp::updateAllWindows();

    //ここから評価値確認部分
    extract_contactPoints(ssd, internal, bodyPoints, objectPoints);
    DH_LOG("ROM evaluation is "+QString::number(rom_eval(arm, joints_list, joint_bone, DF, ashape_all)),0);
    DH_LOG("Coordinate evaluation is "+QString::number(coord_eval(Fp, ObjPs, ObjPs_normal, fpname)),0);
    DH_LOG("Collision evaluation is "+QString::number(collision_eval(arm, bodyPoints, objectPoints, hand_size)),0);
    DH_LOG("ForceClosure evaluation is "+QString::number(forceClosure_eval(arm, ssd, handMesh, objMesh, ObjPs_normal,MP, color_def, area_to_bone, bodyPoints,coef, input_set)),0);

    DH_LOG("FinalEvaluation is "+QString::number(s->fval),0);
    DH_LOG("iter is"+QString::number(iter),0);
    //ここまで

    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free(s);

    dhdelete(bodyPoints);
    dhdelete(objectPoints);
    dhdelete(internal);

    return status;

}


//===============================================================================================ここからPSO





double func_estimate_PSO(const array<double,dimensions> para, Parameter pp){

    estimate_armature_change_PSO(para, pp.arm, pp.Fp, pp.ssd, pp.objMesh);
    extract_contactPoints(pp.ssd, pp.internal, pp.bodyPoints, pp.objectPoints);

    //　評価関数定義
    double estimate_func = pp.par1*coord_eval(pp.Fp, pp.ObjPs, pp.ObjPs_normal, pp.fpname)
                         + pp.par2*rom_eval(pp.arm, pp.jl, pp.jb, pp.DF, pp.as)
                         + pp.par3*collision_eval(pp.arm, pp.bodyPoints, pp.objectPoints, pp.hand_size)
                         + pp.par4*forceClosure_eval(pp.arm, pp.ssd, pp.handMesh, pp.objMesh, pp.ObjPs_normal,
                                                     pp.MP, pp.color_def, pp.area_to_bone, pp.bodyPoints,
                                                     pp.coef, pp.input_set);

    return estimate_func;
}

void estimate_armature_change_PSO(const array<double,dimensions> para, dhArmature* arm, dhFeaturePoints *Fp,
                              dhSkeletalSubspaceDeformation* ssd, dhMesh* objMesh){

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
            angle_deg = { para[0], para[1], para[2] };
            vec_t = { para[3], para[4], para[5] };
        }
        else if(bone_list[l] == "TMCP"){
            angle_deg = { para[6], para[7], para[8] };
        }
        else if(bone_list[l] == "TPP"){
            angle_deg = { para[9], 0, 0 };
        }
        else if(bone_list[l] == "TDP"){
            angle_deg = { para[10], 0, 0 };
        }
        else if(bone_list[l] == "IPP"){
            angle_deg = { para[11], para[12], para[13] };
        }
        else if(bone_list[l] == "IMP"){
            angle_deg = { para[14], 0, 0 };
        }
        else if(bone_list[l] == "IDP"){
            angle_deg = { para[15], 0, 0 };
        }
        else if(bone_list[l] == "MPP"){
            angle_deg = { para[16], para[17], para[18] };
        }
        else if(bone_list[l] == "MMP"){
            angle_deg = { para[19], 0, 0 };
        }
        else if(bone_list[l] == "MDP"){
            angle_deg = { para[20], 0, 0 };
        }
        else if(bone_list[l] == "RMCP"){
            angle_deg = { para[21], 0, 0 };
        }
        else if(bone_list[l] == "RPP"){
            angle_deg = { para[22], para[23], para[24] };
        }
        else if(bone_list[l] == "RMP"){
            angle_deg = { para[25], 0, 0 };
        }
        else if(bone_list[l] == "RDP"){
            angle_deg = { para[26], 0, 0 };
        }
        else if(bone_list[l] == "PMCP"){
            angle_deg = { para[27], 0, 0 };
        }
        else if(bone_list[l] == "PPP"){
            angle_deg = { para[28], para[29], para[30] };
        }
        else if(bone_list[l] == "PMP"){
            angle_deg = { para[31], 0, 0 };
        }
        else if(bone_list[l] == "PDP"){
            angle_deg = { para[32], 0, 0 };
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
    ssd->Update();
    objMesh->Update();

}

//PSOの位置更新関数
void update_positions(particles& positions, particles velocities){
    for(size_t i=0; i<positions.size(); i++){
        positions[i] = positions[i] + velocities[i];
    }
}

//PSOの速度更新関数
void update_velocities(particles positions, particles& velocities,
                        particles PersonalBest, array<double,dimensions> GlobalBest,int t,
                        const double wmax, const double wmin, const double ro1, const double ro2
                        ){

//    double rc1,rc2;
//    rc1 = rand()*(ro_max/RAND_MAX);       //このプログラムでは乱数でなく，確定数を用いる
//    rc2 = rand()*(ro_max/RAND_MAX);

    double w = (wmax-wmin)*( repeat_times - t )/repeat_times + wmin;
    for(size_t i=0; i<velocities.size(); i++){
//        velocities[i] = w*velocities[i] + rc1*(PersonalBest[i] - positions[i]) +
//                            rc2*(GlobalBest - positions[i]);
        velocities[i] = w*velocities[i] + ro1*(PersonalBest[i] - positions[i]) +
                            ro2*(GlobalBest - positions[i]);
    }

}

//粒子群最適化法(PSO)による最終姿勢生成
void FinalPostureCreate_PSO(dhArmature* arm,dhFeaturePoints* Fp, dhSkeletalSubspaceDeformation* ssd,
                            dhMesh* handMesh, dhMesh* objMesh, int age){
    //粒子法の設定部
    double scope_min = -30;
    double scope_max = 30;
    particles positions;
    particles velocities;

    //粒子の位置，速度の初期化
    map<int,QString> bone_list;
    bone_list[0] = "ROOT";     bone_list[1] = "TMCP";   bone_list[2] = "TPP";
    bone_list[3] = "TDP";      bone_list[4] = "IPP";    bone_list[5] = "IMP";
    bone_list[6] = "IDP";      bone_list[7] = "MPP";    bone_list[8] = "MMP";
    bone_list[9] = "MDP";      bone_list[10] = "RMCP";  bone_list[11] = "RPP";
    bone_list[12] = "RMP";     bone_list[13] = "RDP";   bone_list[14] = "PMCP";
    bone_list[15] = "PPP";     bone_list[16] = "PMP";   bone_list[17] = "PDP";

    vector<double> init;
    for(int l=0; l<bone_list.size(); l++){
        if(bone_list[l] == "ROOT"){
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
            init.push_back(angle[1]);
            init.push_back(angle[2]);
            init.push_back(trans_mat[12]);
            init.push_back(trans_mat[13]);
            init.push_back(trans_mat[14]);
        }
        else if(bone_list[l] == "TMCP" || bone_list[l] == "IPP" || bone_list[l] == "MPP"
                || bone_list[l] == "RPP" || bone_list[l] == "PPP"){
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
            init.push_back(angle[1]);
            init.push_back(angle[2]);
        }
        else{
            dhMat44 trans_mat = arm->bone(bone_list[l])->R;
            dhMat33 rot_mat = trans_mat.toMat33();
            vector<double> angle = Rot2Euler(rot_mat);

            init.push_back(angle[0]);
        }
    }

    srand(time(NULL));
    for(int row=0; row<positions.size(); row++){
        for(int col=0; col<positions[0].size(); col++){
            positions[row][col] = init[col] + rand()*(scope_max-scope_min)/RAND_MAX + scope_min;
            velocities[row][col] = 0;
        }
    }

    //ROM評価関数のファイル読み込み
    vector<vector<QString>> joints_list;
    vector<vector<QString>> joint_bone;
    vector<vector<QString>> DF;
    vector<alphashape> ashape_all;

    vector<vector<QString>> ObjPs;
    vector<vector<QString>> ObjPs_normal;
    QStringList fpname;

    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloud* internal = dhnew<dhPointCloud>();
    double hand_size;

    vector<vector<QString>> MP;
    vector<vector<QString>> color_def;
    vector<vector<QString>> area_to_bone;
    double coef;

    prepare_forceClosure(MP, color_def, area_to_bone, coef, age);
    prepare_romeval(joints_list, joint_bone, DF, ashape_all, age);
    prepare_coordeval(Fp, ObjPs, ObjPs_normal, fpname);


    Parameter pp = { 70, 2, 1000, 500, arm, Fp, ssd, handMesh, objMesh,
                     joints_list, joint_bone, DF, ashape_all,
                     ObjPs, ObjPs_normal, fpname,
                     bodyPoints, objectPoints, internal, hand_size,
                     MP, color_def, area_to_bone};

    //パーソナルベスト位置，値，グローバルベスト位置の初期化
    particles PersonalBest = positions;
    array<double,number_of_particles> PersonalBest_value;
    for(int n=0; n<number_of_particles; n++){
        PersonalBest_value[n] = func_estimate_PSO(PersonalBest[n], pp);
    }
    int gb_index;
    double tmpmin = DBL_MAX;
    for(int index=0; index<number_of_particles; index++){
        if(PersonalBest_value[index] < tmpmin){
            tmpmin = PersonalBest_value[index];
            gb_index = index;
        }
    }

    array<double,dimensions> GlobalBest = PersonalBest[gb_index];

    //ここから最適化のための繰り返し計算
    double tmpeval;
    for(int t=0; t<repeat_times; t++){
        //速度更新
        update_velocities(positions, velocities, PersonalBest, GlobalBest, t);
        //位置
        update_positions(positions, velocities);

        //パーソナルベストとグローバルベストの更新
        tmpmin = DBL_MAX;
        for(int i=0; i<number_of_particles; i++){
            tmpeval = func_estimate_PSO(positions[i], pp);
            if(tmpeval < PersonalBest_value[i]){
                PersonalBest_value[i] = tmpeval;
                PersonalBest[i] = positions[i];
            }
            if(PersonalBest_value[i] < tmpmin){
                tmpmin = PersonalBest_value[i];
                gb_index = i;
            }
            GlobalBest = PersonalBest[gb_index];
        }

        DH_LOG("now value is "+QString::number(func_estimate_PSO(GlobalBest, pp)),0);
    }

    DH_LOG("Final estimate value is "+QString::number(func_estimate_PSO(GlobalBest, pp)),0);
}


//double Ackley_function(array<double,dimensions> pos){
//    double t1,t2,t3,t4;
//    t1 = -20*exp(-0.2 * sqrt(0.5 * (pos[0]*pos[0]+pos[1]*pos[1]) ));
//    t2 = -exp(0.5 * ( cos(2*pi*pos[0]) + cos(2*pi*pos[1]) ));
//    t3 = e;
//    t4 = 20;
//    return t1+t2+t3+t4;
//}


