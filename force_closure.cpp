#include<cmath>

#include"csv.hpp"
#include"dhcontact.h"
#include"force_closure.hpp"
#include"glpk.h"


void prepare_forceClosure(vector<vector<QString>>& MP, vector<vector<QString>>& color_def,
                          vector<vector<QString>>& area_to_bone)
{
    QString mppath = "C:\\kenkyu\\ForceClosure\\csv\\MPdata0302.csv";
    QString parapath = "C:\\kenkyu\\ForceClosure\\csv\\max_muscle_force_0302_para.csv";

//MPdata0302.csvの読み込み==========================
    Csv csvmp(mppath);
    if(!csvmp.getCsv(MP)){
        DH_LOG("failed to read the file",0);
        return ;
    }
//================================================

//max_muscle_force_0302_para.csvの読み込み===========
    vector<vector<QString>> MP_para;
    Csv csvpara(parapath);
    if(!csvpara.getCsv(MP_para)){
        DH_LOG("failed to read the file",0);
        return ;
    }
    MP_para.erase(MP_para.begin());

//=================================================

//==ハンドモデルの領域の色情報取得======================================================
    QString list = "C:\\kenkyu\\ForceClosure\\csv\\def_color_area.csv";
    Csv obj(list);
    if(!obj.getCsv(color_def)){
        DH_LOG("failed to read the file",0);
        return ;
    }
    color_def.erase(color_def.begin());
//==============================================================================

//==エリアとboneの対応付け情報取得=====================================================
    QString atb = "C:\\kenkyu\\ForceClosure\\csv\\area_bone.csv";
    Csv obj_atb(atb);
    if(!obj_atb.getCsv(area_to_bone)){
        DH_LOG("failed to read the file",0);
        return ;
    }
    area_to_bone.erase(area_to_bone.begin());
//================================================================================


    //最大筋張力×筋活動度
    for(size_t i=0; i<MP_para.size(); i++){
        for(size_t j=2; j<MP[0].size(); j++){        //最初2列は行名
            if(MP[0][j].contains(MP_para[i][0])){
                for(size_t k=1; k<MP.size(); k++){       //最初1行は筋肉名
                    if(MP[k][j] != "Nan"){
                        double force = MP_para[i][1].toDouble() * MP[k][j].toDouble();
                        MP[k][j] = QString::number(force);
                    }
                }
            }
        }
    }
//    //確認用
//    for(size_t i=0; i<MP.size(); i++){
//        for(size_t j=0; j<MP[0].size(); j++){
//            DH_LOG(MP[i][j],0);
//        }
//    }

}

dhMat44 RotateAroundAxis(dhVec3 axis, double theta)
{
    vector<vector<double>> mat;
    vector<double> row;
    row.push_back(axis[0]*axis[0] + (1.0 - axis[0]*axis[0])*cos(theta) );
    row.push_back(axis[0]*axis[1]*(1.0 - cos(theta)) + axis[2]*sin(theta) );
    row.push_back(axis[2]*axis[0]*(1.0 - cos(theta)) - axis[1]*sin(theta) );
    row.push_back(0.0);
    mat.push_back(row);
    row.clear();
    row.push_back(axis[0]*axis[1]*(1.0 - cos(theta)) - axis[2]*sin(theta) );
    row.push_back(axis[1]*axis[1] + (1.0 - axis[1]*axis[1])*cos(theta) );
    row.push_back(axis[1]*axis[2]*(1.0 - cos(theta)) + axis[0]*sin(theta) );
    row.push_back(0.0);
    mat.push_back(row);
    row.clear();
    row.push_back(axis[2]*axis[0]*(1.0 - cos(theta)) + axis[1]*sin(theta) );
    row.push_back(axis[1]*axis[2]*(1.0 - cos(theta)) - axis[0]*sin(theta) );
    row.push_back(axis[2]*axis[2] + (1.0 - axis[2]*axis[2])*cos(theta) );
    row.push_back(0.0);
    mat.push_back(row);
    row.clear();
    row.push_back(0.0);row.push_back(0.0);row.push_back(0.0);row.push_back(0.0);
    mat.push_back(row);

    dhMat44 matrix(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
                   mat[1][0], mat[1][1], mat[1][2], mat[1][3],
                   mat[2][0], mat[2][1], mat[2][2], mat[2][3],
                   mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
    return matrix;
}

void GetJacobianBones(dhArmature* arm, vector<QString> bones, vector<QString> contact_bones,
                      vector<QString>& JacobianBones)
{
    for(size_t i=1; i<bones.size(); i++){   //don't use 'ROOT'
        int flag = 0;
        for(size_t j=0; j<contact_bones.size(); j++){
            dhBone* tmplink;
            tmplink = dhnew<dhBone>();
            vector<QString> tmpbones;
            vector<int> tmpdepths;
            getArmatureStructure(arm, tmplink, tmpbones, tmpdepths, bones[i]);
            for(size_t k=0; k<tmpbones.size(); k++){
                if(contact_bones[j] == tmpbones[k]){
                    JacobianBones.push_back(bones[i]);
                    flag = 1;
                    break;
                }
            }
            if(flag == 1)   break;
        }
    }
}



vector<vector<double>> ComputeFrictionMatrix(dhArmature* arm, segment* segm, vector<int> contact_areas,
                                             vector<QString> contact_bones, double friction_coefficient)
{
    vector<vector<double>> FrictionMatrix(4*contact_areas.size(),vector<double>(3*contact_areas.size()));
    double theta = atan(friction_coefficient);
    dhVec3 vertical_vector;
    const dhMat33 global_direction(1, 0, 0,
                                   0, 1, 0,
                                   0, 0, 1);


    for(int area=0; area<contact_areas.size(); area++){

        //さらに変更ここから

        dhVec3 fricConeHeight = segm[contact_areas[area]].BodyCoG_Normal;      //摩擦円錐の高さ方向ベクトル
        fricConeHeight.normalize();

        dhVec3 fricCone_x(1, 0, -fricConeHeight[0]/fricConeHeight[2]);      //y方向を0とする．
        fricCone_x.normalize();

        double zz = -1/(fricCone_x[2]);
        double yy = -fricConeHeight[0]/fricConeHeight[1] - zz*fricConeHeight[2]/fricConeHeight[1];

        dhVec3 fricCone_z(1, yy, zz);
        fricCone_z.normalize();

//        DH_LOG("coord1:"+QString::number(fricConeHeight[0])+","+QString::number(fricConeHeight[1])+","+QString::number(fricConeHeight[2]),0);
//        DH_LOG("coord2:"+QString::number(fricCone_x[0])+","+QString::number(fricCone_x[1])+","+QString::number(fricCone_x[2]),0);
//        DH_LOG("coord3:"+QString::number(fricCone_z[0])+","+QString::number(fricCone_z[1])+","+QString::number(fricCone_z[2]),0);

        dhMat33 z1 = RotateAroundAxis(fricCone_z, theta+M_PI/2).toMat33();
        dhMat33 z2 = RotateAroundAxis(fricCone_z,-(theta+M_PI/2)).toMat33();
        dhMat33 z3 = RotateAroundAxis(fricCone_x, theta+M_PI/2).toMat33();
        dhMat33 z4 = RotateAroundAxis(fricCone_x,-(theta+M_PI/2)).toMat33();

        dhVec3 n1 = z1*fricConeHeight;
        dhVec3 n2 = z2*fricConeHeight;
        dhVec3 n3 = z3*fricConeHeight;
        dhVec3 n4 = z4*fricConeHeight;

        FrictionMatrix[4*area][3*area]     = n1[0];
        FrictionMatrix[4*area][3*area+1]   = n1[1];
        FrictionMatrix[4*area][3*area+2]   = n1[2];
        FrictionMatrix[4*area+1][3*area]   = n2[0];
        FrictionMatrix[4*area+1][3*area+1] = n2[1];
        FrictionMatrix[4*area+1][3*area+2] = n2[2];
        FrictionMatrix[4*area+2][3*area]   = n3[0];
        FrictionMatrix[4*area+2][3*area+1] = n3[1];
        FrictionMatrix[4*area+2][3*area+2] = n3[2];
        FrictionMatrix[4*area+3][3*area]   = n4[0];
        FrictionMatrix[4*area+3][3*area+1] = n4[1];
        FrictionMatrix[4*area+3][3*area+2] = n4[2];
        //ここまで




//ここから変更
//        dhMat33 coord = arm->bone(contact_bones[area])->Twj.toMat33();

//                                                //　(同次変換行列)×(グローバル座標系y方向)
//        dhVec3 finger_x = coord.column(0);      //グローバル座標系から見た指のx座標
//        dhVec3 finger_y = coord.column(1);      //グローバル座標系から見た指のy座標(軸方向)
//        dhVec3 finger_z = coord.column(2);      //グローバル座標系から見た指のz座標


//        dhMat33 z1 = RotateAroundAxis(finger_z, theta+M_PI/2).toMat33();
//        dhMat33 z2 = RotateAroundAxis(finger_z,-(theta+M_PI/2)).toMat33();
//        dhMat33 z3 = RotateAroundAxis(finger_x, theta+M_PI/2).toMat33();
//        dhMat33 z4 = RotateAroundAxis(finger_x,-(theta+M_PI/2)).toMat33();

//        dhVec3 n1 = z1*finger_y;
//        dhVec3 n2 = z2*finger_y;
//        dhVec3 n3 = z3*finger_y;
//        dhVec3 n4 = z4*finger_y;

//        FrictionMatrix[4*area][3*area]     = n1[0];
//        FrictionMatrix[4*area][3*area+1]   = n1[1];
//        FrictionMatrix[4*area][3*area+2]   = n1[2];
//        FrictionMatrix[4*area+1][3*area]   = n2[0];
//        FrictionMatrix[4*area+1][3*area+1] = n2[1];
//        FrictionMatrix[4*area+1][3*area+2] = n2[2];
//        FrictionMatrix[4*area+2][3*area]   = n3[0];
//        FrictionMatrix[4*area+2][3*area+1] = n3[1];
//        FrictionMatrix[4*area+2][3*area+2] = n3[2];
//        FrictionMatrix[4*area+3][3*area]   = n4[0];
//        FrictionMatrix[4*area+3][3*area+1] = n4[1];
//        FrictionMatrix[4*area+3][3*area+2] = n4[2];
//ここまで

//        if(segm[area].ObjectCoG[1] > 0){
//            vertical_vector[0] = 0;
//            vertical_vector[1] = -1;
//            vertical_vector[2] = 0;
//        }
//        else{
//            vertical_vector[0] = 0;
//            vertical_vector[1] = 1;
//            vertical_vector[2] = 0;
//        }

//        dhMat33 z1 = RotateAroundAxis(global_direction.row(2), theta+M_PI/2).toMat33();
//        dhMat33 z2 = RotateAroundAxis(global_direction.row(2),-(theta+M_PI/2)).toMat33();
//        dhMat33 z3 = RotateAroundAxis(global_direction.row(0), theta+M_PI/2).toMat33();
//        dhMat33 z4 = RotateAroundAxis(global_direction.row(0),-(theta+M_PI/2)).toMat33();

//        dhVec3 n1 = z1*vertical_vector;
//        dhVec3 n2 = z2*vertical_vector;
//        dhVec3 n3 = z3*vertical_vector;
//        dhVec3 n4 = z4*vertical_vector;

//        FrictionMatrix[4*area][3*area]     = n1[0];
//        FrictionMatrix[4*area][3*area+1]   = n1[1];
//        FrictionMatrix[4*area][3*area+2]   = n1[2];
//        FrictionMatrix[4*area+1][3*area]   = n2[0];
//        FrictionMatrix[4*area+1][3*area+1] = n2[1];
//        FrictionMatrix[4*area+1][3*area+2] = n2[2];
//        FrictionMatrix[4*area+2][3*area]   = n3[0];
//        FrictionMatrix[4*area+2][3*area+1] = n3[1];
//        FrictionMatrix[4*area+2][3*area+2] = n3[2];
//        FrictionMatrix[4*area+3][3*area]   = n4[0];
//        FrictionMatrix[4*area+3][3*area+1] = n4[1];
//        FrictionMatrix[4*area+3][3*area+2] = n4[2];
    }

    return FrictionMatrix;
}


vector<vector<double>> ComputeGraspMatrix(segment* segm, vector<int> contact_areas, vector<double> object_center)
{
    vector<vector<double>> GraspMatrix(6,vector<double>(3*contact_areas.size()));

    for(size_t i=0; i<contact_areas.size(); i++){
        GraspMatrix[0][3*i]   = 1;      // 1 0 0
        GraspMatrix[1][3*i+1] = 1;      // 0 1 0
        GraspMatrix[2][3*i+2] = 1;      // 0 0 1

        double rx,ry,rz;
        rx = segm[contact_areas[i]].ObjectCoG[0] - object_center[0];
        ry = segm[contact_areas[i]].ObjectCoG[1] - object_center[1];
        rz = segm[contact_areas[i]].ObjectCoG[2] - object_center[2];

        GraspMatrix[3][3*i+1] = rz;     GraspMatrix[3][3*i+2] = -ry;
        GraspMatrix[4][3*i] = -rz;      GraspMatrix[4][3*i+2] = rx;
        GraspMatrix[5][3*i] = ry;       GraspMatrix[5][3*i+1] = -rx;
    }
//    for(int i=0; i<GraspMatrix.size(); i++){      // G　確認用
//        for(int j=0; j<GraspMatrix[0].size(); j++){
//            DH_LOG(QString::number(GraspMatrix[i][j]),0);
//        }
//    }

    return GraspMatrix;
}


vector<double> GetBoundMatrix(dhArmature* arm, vector<QString> contact_bones)
{
    vector<double> eT;
    for(int i=0; i<contact_bones.size(); i++){
        dhVec3 depth_vec = arm->bone(contact_bones[i])->Twj.toMat33().column(1);
        double norm = depth_vec[0] + depth_vec[1] + depth_vec[2];
        eT.push_back(depth_vec[0]/norm);
        eT.push_back(depth_vec[1]/norm);
        eT.push_back(depth_vec[2]/norm);
    }

    return eT;
}



vector<vector<double>> ComputeContactJacobian(dhArmature* arm, map<QString, int> bone_index ,
                                              vector<QString> JacobianBones, vector<vector<int>> DoFs,
                                              vector<QString> contact_bones, vector<int> contact_areas,
                                              segment* segm)
{
    vector<vector<double>> ContactJacobian;
    for(int i=0; i<JacobianBones.size(); i++){
        dhBone* tmplink;
        tmplink = dhnew<dhBone>();
        vector<QString> tmpbones;
        vector<int> tmpdepths;
        getArmatureStructure(arm, tmplink, tmpbones, tmpdepths, JacobianBones[i]);

        for(int j=0; j<DoFs[ bone_index[JacobianBones[i]] ].size(); j++){
            dhMat44 Tmat = arm->bone(JacobianBones[i])->Twj;
            dhVec3 axis = Tmat.column(DoFs[ bone_index[JacobianBones[i]] ][j]).toVec3();
            dhVec3 orig = arm->bone(JacobianBones[i])->Porigin().toVec3();

            vector<double> jacobiVec;
            for(int k=0; k<contact_bones.size(); k++){
                int flag = 0;
                for(int l=0; l<tmpbones.size(); l++){
                    if(tmpbones[l] == contact_bones[k]){
                        dhVec3 jacobi_ele = axis.cross( (segm[contact_areas[k]].ObjectCoG - orig) );
                        jacobiVec.push_back(jacobi_ele[0]);
                        jacobiVec.push_back(jacobi_ele[1]);
                        jacobiVec.push_back(jacobi_ele[2]);
                        flag = 1;
                        break;
                    }
                }
                if(flag == 0){
                    jacobiVec.push_back(0.0);
                    jacobiVec.push_back(0.0);
                    jacobiVec.push_back(0.0);
                }
            }

            ContactJacobian.push_back(jacobiVec);
        }
    }

    return ContactJacobian;
}

vector<vector<double>> GetMomentArm_Force(vector<vector<QString>> MP, vector<QString> JacobianBones,
                                          vector<vector<int>> DoFs, map<QString, int> bone_index)
{
    vector<vector<double>> MF;
    for(size_t i=0; i<JacobianBones.size(); i++){       //JacobianBones一つ一つにたいして
        for(size_t j=1; j<MP.size(); j++){              //MPdataの1列目を上から走査して
            vector<double> tmprow, tmprow2;
            if(MP[j][0] == JacobianBones[i]){           //同じboneの時の行を取り出す
                if(DoFs[ bone_index[JacobianBones[i]] ].size() == 1){
                    for(size_t k=2; k<MP[0].size(); k++){
                        tmprow.push_back(MP[j][k].toDouble());
                    }
                    MF.push_back(tmprow);
                    break;
                }
                else if(DoFs[ bone_index[JacobianBones[i]] ].size() == 2){
                    for(size_t k=2; k<MP[0].size(); k++){
                        tmprow.push_back(MP[j][k].toDouble());
                        tmprow2.push_back(MP[j+1][k].toDouble());
                    }
                    MF.push_back(tmprow);
                    MF.push_back(tmprow2);
                    break;
                }
            }
        }
    }

    return MF;
}



void Adapt_to_glpk1(vector<vector<double>> G, vector<vector<double>> F, vector<double> eT,
                    vector<vector<double>>& left, vector<double>& right, vector<double>& coef)
{
//条件式の左辺行列の生成
    //等式部分
    for(size_t i=0; i<G.size(); i++){
        vector<double> row;
        for(size_t j=0; j<G[0].size(); j++){
            row.push_back(G[i][j]);
        }
        row.push_back(0.0);
        left.push_back(row);
    }
    //不等式部分
    for(size_t i=0; i<F.size(); i++){
        vector<double> row;
        for(size_t j=0; j<F[0].size(); j++){
            row.push_back(F[i][j]);
        }
        row.push_back(1.0);
        left.push_back(row);
    }
    vector<double> row;
    for(size_t j=0; j<F[0].size(); j++){
        row.push_back(0.0);
    }
    row.push_back(-1.0);
    left.push_back(row);
    row.clear();
//ここから書き換え
    for(size_t j=0; j<eT.size(); j++){
        row.push_back(eT[j]);
    }
//ここまで

//    for(size_t j=0; j<F[0].size(); j++){
//        if(j%3 == 0){
//            row.push_back(1.0);
//        }
//        else{
//            row.push_back(0.0);
//        }
//    }
    row.push_back(0.0);
    left.push_back(row);
    row.clear();

//条件式の右辺行列の生成
    for(size_t col=0; col<G.size(); col++){
        right.push_back(0.0);
    }
    for(size_t col=0; col<F.size()+1; col++){
        right.push_back(0.0);
    }
    right.push_back(F[0].size());

//係数の行列生成
    for(size_t i=0; i<G[0].size(); i++){
        coef.push_back(0.0);
    }
    coef.push_back(1.0);

}



void Adapt_to_glpk2(vector<double> mg, vector<vector<double>> G, vector<vector<double>> J,
                    vector<vector<QString>> MP, vector<vector<double>> MF, vector<vector<double>> F,
                    vector<vector<double>>& left, vector<double>& right, vector<double>& coef)
{
    //条件式の左辺行列の生成
    //等式部分
    for(size_t i=0; i<G.size(); i++){       //1行目
        vector<double> row;
        for(size_t j=0; j<G[0].size(); j++){
            row.push_back(G[i][j]);
        }
        row.push_back(0.0);
        for(size_t j=0; j<MF[0].size(); j++){
            row.push_back(0.0);
        }
        left.push_back(row);
    }

    for(size_t i=0; i<J.size(); i++){       //2行目
        vector<double> row;
        for(size_t j=0; j<J[0].size(); j++){
            row.push_back(J[i][j]);
        }
        row.push_back(0.0);
        for(size_t j=0; j<MF[0].size(); j++){
            row.push_back(MF[i][j]);
        }
        left.push_back(row);
    }

    //不等式部分
    for(size_t i=0; i<F.size(); i++){       //1行目
        vector<double> row;
        for(size_t j=0; j<F[0].size(); j++){
            row.push_back(F[i][j]);
        }
        row.push_back(1.0);
        for(size_t j=0; j<MP[0].size()-2; j++){
            row.push_back(0.0);
        }
        left.push_back(row);
    }

    vector<double> row;     //2行目
    for(size_t j=0; j<F[0].size(); j++){
        row.push_back(0.0);
    }
    row.push_back(-1.0);
    for(size_t j=0; j<MP[0].size()-2; j++){
        row.push_back(0.0);
    }
    left.push_back(row);

    for(size_t i=0; i<MP[0].size()-2; i++){     //3行目
        vector<double> row;
        for(size_t j=0; j<F[0].size(); j++){
            row.push_back(0.0);
        }
        row.push_back(0.0);
        for(size_t j=0; j<MP[0].size()-2; j++){
            if(j == i){
                row.push_back(1.0);
            }
            else{
                row.push_back(0.0);
            }
        }
        left.push_back(row);
    }

    for(size_t i=0; i<MP[0].size()-2; i++){     //4行目
        vector<double> row;
        for(size_t j=0; j<F[0].size(); j++){
            row.push_back(0.0);
        }
        row.push_back(0.0);
        for(size_t j=0; j<MP[0].size()-2; j++){
            if(j == i){
                row.push_back(-1.0);
            }
            else{
                row.push_back(0.0);
            }
        }
        left.push_back(row);
    }

    //条件式の右辺行列
    for(size_t col=0; col<G.size(); col++){
        if(col < 3){
            right.push_back(mg[col]);
        }
        else{
            right.push_back(0.0);
        }
    }

    for(size_t col=0; col<J.size(); col++){
        right.push_back(0.0);
    }

    for(size_t col=0; col<F.size(); col++){
        right.push_back(0.0);
    }
    right.push_back(0.0);

    for(size_t col=0; col<MP[0].size()-2; col++){
        right.push_back(1.0);
    }

    for(size_t col=0; col<MP[0].size()-2; col++){
        right.push_back(0.0);
    }

    //係数の行列生成
    for(size_t i=0; i<G[0].size(); i++){
        coef.push_back(0.0);
    }
    coef.push_back(1.0);
    for(size_t i=0; i<MP[0].size()-2; i++){
        coef.push_back(0.0);
    }


}



double GLPK_solve_LP1(vector<vector<double>> left, vector<double> right, vector<vector<double>> G,
                  vector<double> coef)
{
    double z;
    vector<double> x;
    int ia[1000], ja[1000];
    double ar[1000];
    glp_prob* lp;
    lp = glp_create_prob();
    glp_set_prob_name(lp, "LP1");
    glp_set_obj_dir(lp, GLP_MAX);
    glp_add_rows(lp, left.size());
    for(size_t i=1; i<=left.size(); i++){
        if(i<=G.size()){
            glp_set_row_bnds(lp, i, GLP_FX, right[i-1], right[i-1]);
        }
        else{
            glp_set_row_bnds(lp, i, GLP_UP, 0, right[i-1]);
        }
    }
    glp_add_cols(lp, coef.size());
    for(size_t j=1; j<=coef.size(); j++){
        glp_set_col_bnds(lp, j, GLP_FR, 0, 0);
        glp_set_obj_coef(lp, j, coef[j-1]);
    }
    int cnt=0;
    for(size_t i=1; i<=left.size(); i++){
        for(size_t j=1; j<=left[0].size(); j++){
            if(left[i-1][j-1] != 0){
                ++cnt;
                ia[cnt] = i, ja[cnt] = j, ar[cnt] = left[i-1][j-1];
            }
        }
    }

    glp_load_matrix(lp, cnt, ia, ja, ar);
    glp_simplex(lp, NULL);
    z = glp_get_obj_val(lp);
//    DH_LOG("LP1 value is "+QString::number(z),0);
//    for(size_t j=1; j<=left[0].size(); j++){
//        x.push_back(glp_get_col_prim(lp,j));
//        DH_LOG(QString::number(glp_get_col_prim(lp, j)),0);
//    }

    return z;

}

double GLPK_solve_LP2(vector<vector<double>> left, vector<double> right, vector<double> coef,
                      vector<vector<double>> G, vector<vector<double>> J)
{
    double z;
    vector<double> x;
    int ia[1000], ja[1000];
    double ar[1000];
    glp_prob* lp;
    lp = glp_create_prob();
    glp_set_prob_name(lp, "LP2");
    glp_set_obj_dir(lp, GLP_MAX);
    glp_add_rows(lp, right.size());
    for(size_t i=1; i<=right.size(); i++){
        if( i<=(G.size()+J.size()) ){
            glp_set_row_bnds(lp, i, GLP_FX, right[i-1], right[i-1]);
        }
        else{
            glp_set_row_bnds(lp, i, GLP_UP, 0, right[i-1]);
        }
    }
    glp_add_cols(lp, coef.size());
    for(size_t j=1; j<=coef.size(); j++){
        glp_set_col_bnds(lp, j, GLP_FR, 0, 0);
        glp_set_obj_coef(lp, j, coef[j-1]);
    }
    int cnt=0;
    for(size_t i=1; i<=left.size(); i++){
        for(size_t j=1; j<=left[0].size(); j++){
            if(left[i-1][j-1] != 0){
                ++cnt;
                ia[cnt] = i, ja[cnt] = j, ar[cnt] = left[i-1][j-1];
            }
        }
    }

    glp_load_matrix(lp, cnt, ia, ja, ar);
    glp_simplex(lp, NULL);
    z = glp_get_obj_val(lp);
//    DH_LOG("LP2 value is "+QString::number(z),0);
//    for(size_t j=1; j<=left[0].size(); j++){
//        x.push_back(glp_get_col_prim(lp,j));
//        DH_LOG(QString::number(glp_get_col_prim(lp, j)),0);
//    }

    return z;

}



double forceClosure_eval(dhArmature* arm, dhSkeletalSubspaceDeformation* bodySSD,
                         dhMesh* bodyMesh, dhMesh* objMesh,
                         vector<vector<QString>> MP, vector<vector<QString>> color_def,
                         vector<vector<QString>> area_to_bone)
{
    double FCeval;

    map<QString, int> bone_index;
    bone_index["ROOT"] = 0;     bone_index["CP"] = 1;       bone_index["TMCP"] = 2;
    bone_index["TPP"] = 3;      bone_index["TDP"] = 4;      bone_index["IMCP"] = 5;
    bone_index["IPP"] = 6;      bone_index["IMP"] = 7;      bone_index["IDP"] = 8;
    bone_index["MMCP"] = 9;     bone_index["MPP"] = 10;     bone_index["MMP"] = 11;
    bone_index["MDP"] = 12;     bone_index["RMCP"] = 13;    bone_index["RPP"] = 14;
    bone_index["RMP"] = 15;     bone_index["RDP"] = 16;     bone_index["PMCP"] =17;
    bone_index["PPP"] = 18;     bone_index["PMP"] = 19;     bone_index["PDP"] = 20;

    double friction_coefficient = 1.0;

    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();

    vector<double> mg = {0,-4.26,0};

    vector<vector<int>> DoFs = { {}, {}, {0,2}, {0,2}, {0}, {}, {0,2}, {0}, {0}, {}, {0,2}, {0},{0},
                                 {}, {0,2}, {0}, {0}, {}, {0,2}, {0}, {0}};

    vector<double> object_center = {15, 15, 75};       //後でAPIから関数探す
                                                        //グローバル座標系
    vector<int> areas;      //　↓areas定義
    for(size_t i=0; i<color_def.size(); i++)    areas.push_back(color_def[i][0].toInt());

    segment *segm;
    segm = new segment[areas.size()];

    computeContactRegion(bodyPoints, objectPoints, bodySSD, objMesh);
    segmentBodyPoints_muscle(bodyPoints, bodySSD, bodyMesh, objMesh, color_def, segm);

    areas.erase(areas.begin()+areas.size()-1);  //34は対応するboneないので削除
    size_t orgsize = areas.size();
    int sub=0;
    for(size_t area=0; area<orgsize; area++){      //接触が無い部分のareaを削除する      //has been debugged
        if(segm[area].BodyPointsID.empty()){
            int inc = area - sub;
            ++sub;
            areas.erase(areas.begin()+inc);        //残ったareasは，接触している部分
        }
    }
    vector<QString> contact_bones;
//    DH_LOG("contact_bones",0);
    for(size_t i=0; i<areas.size(); i++){       //上で抽出したareaに対応するboneをcontact_bonesに格納
        contact_bones.push_back(area_to_bone[areas[i]][1]);
//        DH_LOG(contact_bones[i],0);
    }

    if(contact_bones.size() == 0){
        DH_LOG("Force Closure does not exist.",0);
        return 0.0;
    }

    dhBone* link;
    link = dhnew<dhBone>();
    vector<QString> bones;
    vector<int>     depths;
    getArmatureStructure(arm, link, bones, depths);
    dhdelete(link);

    vector<QString> JacobianBones;
    GetJacobianBones(arm, bones, contact_bones, JacobianBones);
//    DH_LOG("JacobianBones",0);
//    for(int i=0;i<JacobianBones.size();i++){
//        DH_LOG(JacobianBones[i],0);
//    }


    vector<vector<double>> GraspMatrix = ComputeGraspMatrix(segm, areas, object_center);
    vector<vector<double>> FrictionMatrix = ComputeFrictionMatrix(arm, segm, areas,contact_bones, friction_coefficient);
    vector<double> eT = GetBoundMatrix(arm, contact_bones);
    vector<vector<double>> ContactJacobian = ComputeContactJacobian(arm, bone_index, JacobianBones, DoFs,
                                                  contact_bones, areas, segm);
    vector<vector<double>> MomentArmForce = GetMomentArm_Force(MP, JacobianBones, DoFs, bone_index);


    vector<vector<double>> left_lp1, left_lp2;
    vector<double> right_lp1, right_lp2;
    vector<double> coef_lp1, coef_lp2;

//    DH_LOG("G is:"+QString::number(GraspMatrix.size())+","+QString::number(GraspMatrix[0].size()),0);
//    DH_LOG("J is:"+QString::number(ContactJacobian.size())+","+QString::number(ContactJacobian[0].size()),0);
//    DH_LOG("F is:"+QString::number(FrictionMatrix.size())+","+QString::number(FrictionMatrix[0].size()),0);
//    DH_LOG("MuscleNumber is:"+QString::number(MP.size())+","+QString::number(MP[0].size()-2),0);

    Adapt_to_glpk1(GraspMatrix, FrictionMatrix, eT, left_lp1, right_lp1, coef_lp1);
    Adapt_to_glpk2(mg, GraspMatrix, ContactJacobian, MP, MomentArmForce, FrictionMatrix,
                   left_lp2, right_lp2, coef_lp2);



//    DH_LOG("LP2_left is:"+QString::number(left_lp2.size())+","+QString::number(left_lp2[0].size()),0);
//    DH_LOG("LP2_right is:"+QString::number(right_lp2.size()),0);
//    DH_LOG("LP2_coef is:"+QString::number(coef_lp2.size()),0);


    double fc_lp1, fc_lp2;
    fc_lp1 = GLPK_solve_LP1(left_lp1,right_lp1,GraspMatrix,coef_lp1);
    if(fc_lp1 == 0){
        DH_LOG("Force Closure does not exist.",0);
        FCeval = 1.0;
    }
    else{
        fc_lp2 = GLPK_solve_LP2(left_lp2, right_lp2, coef_lp2, GraspMatrix, ContactJacobian);
        if(fc_lp2 >= 0.2){
            DH_LOG("Force Closure exist.",0);
            FCeval = 0.0;
        }
        else if(fc_lp2 < 0.2){
            DH_LOG("Force Closure exist!",0);
            FCeval = 1.0 - 25*fc_lp2*fc_lp2;
        }
    }
    DH_LOG("Evaluate-value is "+QString::number(fc_lp2)+"->"+QString::number(FCeval),0);

    dhdelete(bodyPoints);
    dhdelete(objectPoints);

    return FCeval;
}