#include<cmath>

#include"csv.hpp"
#include"dhcontact.h"
#include"force_closure.hpp"
#include"dhShapeBox.h"
#include"glpk.h"


void prepare_forceClosure(vector<vector<QString>>& MP, vector<vector<QString>>& color_def,
                          vector<vector<QString>>& area_to_bone,double& coef, int age)
{

    QString mppath = "C:\\kenkyu\\ForceClosure\\csv\\MPdata0302.csv";
    QString rstpath = "C:\\kenkyu\\MuscleRestrict\\strength_age.csv";
//    QString parapath = "C:\\kenkyu\\ForceClosure\\csv\\max_muscle_force_0302_para.csv";


//MPdata0302.csvの読み込み==========================
    Csv csvmp(mppath);
    if(!csvmp.getCsv(MP)){
        DH_LOG("failed to read the file",0);
        return ;
    }
//================================================

//strength_age.csvの読み込み===========
    vector<vector<QString>> MP_rst;
    Csv csvpara(rstpath);
    if(!csvpara.getCsv(MP_rst)){
        DH_LOG("failed to read the file",0);
        return ;
    }
    MP_rst.erase(MP_rst.begin());

//=================================================

////max_muscle_force_0302_para.csvの読み込み===========
//    vector<vector<QString>> MP_para;
//    Csv csvpara(parapath);
//    if(!csvpara.getCsv(MP_para)){
//        DH_LOG("failed to read the file",0);
//        return ;
//    }
//    MP_para.erase(MP_para.begin());

////=================================================

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

    //加齢による筋力制限をかける
    double rst;
    for(size_t row=0; row<MP_rst.size(); row++){
        if(MP_rst[row][0].toInt() == age){
            rst = MP_rst[row][1].toDouble()/100;
            break;
        }
    }

    for(size_t i=1; i<MP.size(); i++){
        for(size_t j=2; j<MP[0].size(); j++){
            if(MP[i][j] != "Nan"){
                double force = MP[i][j].toDouble()*rst;
                MP[i][j] = QString::number(force);
            }
        }
    }

    //加齢による摩擦係数制限をかける
    const int MAX_AGE = 80;                 //[Mabuchi 2018]の論文データ
    const int MIN_AGE = 20;                 //最大値0.55を1とし，割合として扱えるように
    const double MAX_RATE = 1.0;
    const double MIN_RATE = 0.15/0.55;

    if(age<80){
        coef = (MIN_RATE-MAX_RATE)*(age-MIN_AGE)/(MAX_AGE-MIN_AGE) + MAX_RATE;
    }
    else{
        coef = MIN_RATE;
    }

//    //最大緊張力×制限率 ->　muscle_para用
//    for(size_t i=0; i<MP_para.size(); i++){
//        for(size_t j=2; j<MP[0].size(); j++){        //最初2列は行名
//            if(MP[0][j].contains(MP_para[i][0])){
//                for(size_t k=1; k<MP.size(); k++){       //最初1行は筋肉名
//                    if(MP[k][j] != "Nan"){
//                        double force = MP_para[i][1].toDouble() * MP[k][j].toDouble();
//                        MP[k][j] = QString::number(force);
//                    }
//                }
//            }
//        }
//    }
//    //確認用
//    for(size_t i=0; i<MP.size(); i++){
//        for(size_t j=0; j<MP[0].size(); j++){
//            DH_LOG(MP[i][j],0);
//        }
//    }

}

dhMat44 RotateAroundAxis(dhVec3 axis, double theta)     //ロドリゲスの公式
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
            dhdelete(tmplink);
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



vector<vector<double>> ComputeFrictionMatrix(segment* segm, vector<int> contact_areas, double friction_coefficient,
                                             vector<vector<QString>> ObjPs_normal, vector<int> force_areas)
{
    QList<int> thumb = {0, 1, 16, 33};
    QList<int> index = {2, 3, 4, 17, 28, 29, 30, 31, 32};
    QList<int> middle = {5, 6, 7, 18, 25, 26, 27};
    QList<int> ring = {8, 9, 10, 22, 23, 24};
    QList<int> pinky = {11, 12, 19, 20, 21};


    vector<vector<double>> FrictionMatrix(4*force_areas.size(),vector<double>(3*force_areas.size()));
    double theta = atan(friction_coefficient);

    string fn = "C:\\Users\\ynunakanishi\\Desktop\\log6.txt";
    ofstream log(fn, ios::app);

    for(int area=0; area<force_areas.size(); area++){

        dhVec3 fricCone_y = segm[force_areas[area]].BodyCoG_Normal;      //摩擦円錐の高さ方向ベクトル
        fricCone_y.normalize();                                            //指の法線はめり込むと逆向き法線になり得る

        dhVec3 obj_Normal;
        if(thumb.contains(force_areas[area]))
        {
            obj_Normal[0] = ObjPs_normal[0][1].toDouble();
            obj_Normal[1] = ObjPs_normal[0][2].toDouble();
            obj_Normal[2] = ObjPs_normal[0][3].toDouble();
        }
        else if(index.contains(force_areas[area]))
        {
            obj_Normal[0] = ObjPs_normal[1][1].toDouble();
            obj_Normal[1] = ObjPs_normal[1][2].toDouble();
            obj_Normal[2] = ObjPs_normal[1][3].toDouble();
        }
        else if(middle.contains(force_areas[area]))
        {
            obj_Normal[0] = ObjPs_normal[2][1].toDouble();
            obj_Normal[1] = ObjPs_normal[2][2].toDouble();
            obj_Normal[2] = ObjPs_normal[2][3].toDouble();
        }
        else if(ring.contains(force_areas[area]))
        {
            obj_Normal[0] = ObjPs_normal[3][1].toDouble();
            obj_Normal[1] = ObjPs_normal[3][2].toDouble();
            obj_Normal[2] = ObjPs_normal[3][3].toDouble();
        }
        else if(pinky.contains(force_areas[area]))
        {
            obj_Normal[0] = ObjPs_normal[4][1].toDouble();
            obj_Normal[1] = ObjPs_normal[4][2].toDouble();
            obj_Normal[2] = ObjPs_normal[4][3].toDouble();
        }
        else{
            DH_LOG("not belong to any fingers",0);
            DH_LOG(QString::number(force_areas[area]),0);
            obj_Normal = segm[force_areas[area]].ObjectNormal;
        }

        obj_Normal.normalize();

        log << force_areas[area] << endl;
        log << obj_Normal[0];   log << "," ; log << obj_Normal[1];  log<< ",";  log << obj_Normal[2] << endl;

        log << "FRICTION CONE" << endl;
        log << fricCone_y[0];   log << "," ; log << fricCone_y[1];  log<< ",";  log << fricCone_y[2] << endl;
        log << "Finger to Center" << endl;
        log << obj_Normal[0];   log << "," ; log << obj_Normal[1];  log<< ",";  log << obj_Normal[2] << endl;

        //指の法線ベクトルと物体の法線ベクトルの成す角は，90度以上必要
//        if(fricCone_y * obj_Normal > 0){
//            fricCone_y[0] = -fricCone_y[0];
//            fricCone_y[1] = -fricCone_y[1];
//            fricCone_y[2] = -fricCone_y[2];
//        }


        log << "FRICTION CONE AFTER" << endl;
        log << fricCone_y[0];   log << "," ; log << fricCone_y[1];  log<< ",";  log << fricCone_y[2] << endl;

        log << endl << endl;

        dhVec3 fricCone_x(1, 0, -fricCone_y[0]/fricCone_y[2]);      //y方向を0とする．
        fricCone_x.normalize();

        double zz = -1/(fricCone_x[2]);
        double yy = -fricCone_y[0]/fricCone_y[1] - zz*fricCone_y[2]/fricCone_y[1];

        dhVec3 fricCone_z(1, yy, zz);
        fricCone_z.normalize();

//        DH_LOG("coord1:"+QString::number(fricConeHeight[0])+","+QString::number(fricConeHeight[1])+","+QString::number(fricConeHeight[2]),0);
//        DH_LOG("coord2:"+QString::number(fricCone_x[0])+","+QString::number(fricCone_x[1])+","+QString::number(fricCone_x[2]),0);
//        DH_LOG("coord3:"+QString::number(fricCone_z[0])+","+QString::number(fricCone_z[1])+","+QString::number(fricCone_z[2]),0);

        dhMat33 z1 = RotateAroundAxis(fricCone_z, theta+M_PI/2).toMat33();
        dhMat33 z2 = RotateAroundAxis(fricCone_z,-(theta+M_PI/2)).toMat33();
        dhMat33 z3 = RotateAroundAxis(fricCone_x, theta+M_PI/2).toMat33();
        dhMat33 z4 = RotateAroundAxis(fricCone_x,-(theta+M_PI/2)).toMat33();

        dhVec3 n1 = z1*fricCone_y;
        dhVec3 n2 = z2*fricCone_y;
        dhVec3 n3 = z3*fricCone_y;
        dhVec3 n4 = z4*fricCone_y;

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

    }

    return FrictionMatrix;
}


vector<vector<double>> ComputeGraspMatrix(segment* segm, vector<int> contact_areas,
                                          dhVec3 object_center, vector<int> force_areas)
{
    vector<vector<double>> GraspMatrix(6,vector<double>(3*force_areas.size()));

    for(size_t i=0; i<force_areas.size(); i++){
        GraspMatrix[0][3*i]   = 1;      // 1 0 0
        GraspMatrix[1][3*i+1] = 1;      // 0 1 0
        GraspMatrix[2][3*i+2] = 1;      // 0 0 1

        double rx,ry,rz;
        rx = segm[force_areas[i]].ObjectCoG[0] - object_center[0];
        ry = segm[force_areas[i]].ObjectCoG[1] - object_center[1];
        rz = segm[force_areas[i]].ObjectCoG[2] - object_center[2];

        GraspMatrix[3][3*i+1] = -rz;     GraspMatrix[3][3*i+2] = ry;
        GraspMatrix[4][3*i] = rz;      GraspMatrix[4][3*i+2] = -rx;
        GraspMatrix[5][3*i] = -ry;       GraspMatrix[5][3*i+1] = rx;
    }

    return GraspMatrix;
}


vector<double> GetBoundMatrix(segment* segm, vector<int> contact_areas, dhSkeletalSubspaceDeformation* bodySSD,
                              vector<vector<QString>> ObjPs_normal, vector<int> &force_areas)
{
    string fn = "C:\\Users\\ynunakanishi\\Desktop\\log3.txt";
    ofstream log3(fn, ios::app);

    vector<double> eT;
    log3 << endl;
    for(size_t i=0; i<contact_areas.size(); i++){
        for(size_t j=0; j<ObjPs_normal.size(); j++){
            if(contact_areas[i] == ObjPs_normal[j][0].toInt()){
                log3 << contact_areas[i] << endl;
                force_areas.push_back(contact_areas[i]);

//                int id1 = -1;   int id2 = -1;
//                extract_nearpoints(segm[contact_areas[i]], bodySSD, id1, id2);
                dhVec3 depth_vec = segm[contact_areas[i]].BodyCoG_Normal;      //摩擦円錐の高さ方向ベクトル

                double absx, absy, absz;
                absx = (depth_vec[0] > 0) ? depth_vec[0] : -depth_vec[0];
                absy = (depth_vec[1] > 0) ? depth_vec[1] : -depth_vec[1];
                absz = (depth_vec[2] > 0) ? depth_vec[2] : -depth_vec[2];

                double elesum = absx + absy + absz;

                eT.push_back(depth_vec[0]/elesum);
                eT.push_back(depth_vec[1]/elesum);
                eT.push_back(depth_vec[2]/elesum);
            }
        }

    }
    log3 << "eT size is:";  log3 << eT.size() << endl;
    log3 << "force_areas is:";
    for(int i=0;i<force_areas.size();i++){
        log3 << force_areas[i];   log3 << ",";
    }

    return eT;
}



vector<vector<double>> ComputeContactJacobian(dhArmature* arm, map<QString, int> bone_index ,
                                              vector<QString> JacobianBones, vector<vector<int>> DoFs,
                                              vector<QString> contact_bones, vector<int> contact_areas,
                                              vector<int> force_areas, segment* segm, map<int,QString> atb)
{
    vector<vector<double>> ContactJacobian;
    for(int i=0; i<JacobianBones.size(); i++){
        dhBone* tmplink;
        tmplink = dhnew<dhBone>();
        vector<QString> tmpbones;
        vector<int> tmpdepths;
        getArmatureStructure(arm, tmplink, tmpbones, tmpdepths, JacobianBones[i]);
        dhdelete(tmplink);
        for(int j=0; j<DoFs[ bone_index[JacobianBones[i]] ].size(); j++){
            dhMat44 Tmat = arm->bone(JacobianBones[i])->Twj;
            dhVec3 axis = Tmat.column(DoFs[ bone_index[JacobianBones[i]] ][j]).toVec3().normalized();
            dhVec3 orig = arm->bone(JacobianBones[i])->Porigin().toVec3();

            vector<double> jacobiVec;
            for(int k=0; k<force_areas.size(); k++){
                int flag = 0;
                for(int l=0; l<tmpbones.size(); l++){
                    if(tmpbones[l] == atb[force_areas[k]]){
                        dhVec3 jacobi_ele = axis.cross( (segm[force_areas[k]].ObjectCoG - orig) );
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
//    for(int i=0;i<ContactJacobian.size(); i++){
//        for(int j=0;j<ContactJacobian[0].size();j++){
//            log3 << ContactJacobian[i][j];
//            log3 << ",";
//        }
//        log3 << endl;
//    }
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

//    for(int i=0;i<MF.size(); i++){
//        for(int j=0;j<MF[0].size();j++){
//            log4 << MF[i][j];
//            log4 << ",";
//        }
//        log4 << endl;
//    }
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
    string fn = "C:\\Users\\ynunakanishi\\Desktop\\log5.txt";
    ofstream log(fn, ios::app);

    double z;

    int *ia, *ja;
    double *ar;

    ia = (int*)malloc(sizeof(int)*(left.size()*left[0].size()+1));
    ja = (int*)malloc(sizeof(int)*(left.size()*left[0].size()+1));
    ar = (double*)malloc(sizeof(double)*(left.size()*left[0].size()+1));

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

    log << "left:(";  log << left.size();  log << ",";  log << left[0].size();  log << ")" << endl;
    log << "right:";  log << right.size() << endl;
    log << "LP1: "; log << z <<endl;

    for(int j=1;j<=left[0].size(); j++){
        log << "f";  log << j;  log << ":";  log << glp_get_col_prim(lp, j);    log << "  ";
        if(j%5 == 4)    log << endl;
    }
    log << endl;
//    DH_LOG("LP1 value is "+QString::number(z),0);
//    for(size_t j=1; j<=left[0].size(); j++){
//        x.push_back(glp_get_col_prim(lp,j));
//        DH_LOG(QString::number(glp_get_col_prim(lp, j)),0);
//    }

    free(ia);
    free(ja);
    free(ar);

    return z;

}

double GLPK_solve_LP2(vector<vector<double>> left, vector<double> right, vector<double> coef,
                      vector<vector<double>> G, vector<vector<double>> J,vector<vector<double>> MF)
{
    string fn = "C:\\Users\\ynunakanishi\\Desktop\\log.txt";
    ofstream log(fn, ios::app);

    double z;

    int *ia, *ja;
    double *ar;
    ia = (int*)malloc(sizeof(int)*(left.size()*left[0].size()+1));
    ja = (int*)malloc(sizeof(int)*(left.size()*left[0].size()+1));
    ar = (double*)malloc(sizeof(double)*(left.size()*left[0].size()+1));

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
    int ret = glp_simplex(lp, NULL);
    z = glp_get_obj_val(lp);

    if(ret == GLP_EFAIL)log << "GLP_EFAIL" << endl;
    if(ret == GLP_EOBJLL)log << "GLP_EOBJLL" << endl;
    if(ret == GLP_EOBJUL)log << "GLP_EOBJUL" << endl;
    if(ret == GLP_EITLIM)log << "GLP_EITLIM" << endl;
    if(ret == GLP_ETMLIM)log << "GLP_ETMLIM" << endl;
    if(ret == GLP_ENOPFS)log << "GLP_ENOPFS" << endl;
    if(ret == GLP_ENODFS)log << "GLP_ENODFS" << endl;
    if(ret == 0)    log << "success" << endl;

    log << "left:(";  log << left.size();  log << ",";  log << left[0].size();  log << ")" << endl;
    log << "right:";  log << right.size() << endl;

    log << "LP2: "; log << ret; log << "->"; log << z <<endl;
    for(size_t j=1; j<=left[0].size(); j++){
        log << "f"; log << j; log << ":"; log << glp_get_col_prim(lp, j); log << "  ";
        if(j%5 == 4)    log << endl;
    }
    log << endl << endl;

    log << "here, confirm part" << endl;
    log << "Gf = t" << endl;
    for(int i=0; i<G.size(); i++){
        double lsum = 0;

        for(int j=0; j<G[0].size(); j++){
            lsum += G[i][j]*glp_get_col_prim(lp,j+1);
        }
        double rsum = right[i];

        log << lsum; log << ","; log << rsum << endl;
    }
    log << endl;

    log << "Jf - MFA = 0" << endl;
    for(int i=0; i<J.size(); i++){
        double lsum=0;
        double rsum=0;
        for(int j=0; j<J[0].size(); j++){
            lsum += glp_get_col_prim(lp,j+1)*J[i][j];
        }
        for(int j=0; j<MF[0].size(); j++){
            rsum += glp_get_col_prim(lp,J[0].size()+1+j+1)*MF[i][j];
        }
        log << lsum; log << ","; log << rsum << endl;
    }
    log << endl << endl;


//    DH_LOG("LP2 value is "+QString::number(z),0);
//    for(size_t j=1; j<=left[0].size(); j++){
//        x.push_back(glp_get_col_prim(lp,j));
//        DH_LOG("f"+QString::number(j)+":"+QString::number(glp_get_col_prim(lp, j)),0);
//    }

    free(ia);
    free(ja);
    free(ar);

    return z;
}



double forceClosure_eval(dhArmature* arm, dhSkeletalSubspaceDeformation* bodySSD,
                         dhMesh* bodyMesh, dhMesh* objMesh, vector<vector<QString>> ObjPs_normal,
                         vector<vector<QString>> MP, vector<vector<QString>> color_def,
                         vector<vector<QString>> area_to_bone, dhPointCloudAsVertexRef* &bodyPoints,
                         double coef)
{
    double FCeval;
    const double fric_coef_young = 0.8;
    const double obj_mass = 1000;      //[g]
    const double gravity = 9.81;      //[m/s^2]
    dhVec3 object_center(25, 25, 125);       //後でAPIから関数探す
//    dhVec3 object_center(50, 25, 3.5);

    string fn = "C:\\Users\\ynunakanishi\\Desktop\\log2.txt";
    ofstream log2(fn, ios::app);

    map<QString, int> bone_index;
    bone_index["ROOT"] = 0;     bone_index["CP"] = 1;       bone_index["TMCP"] = 2;
    bone_index["TPP"] = 3;      bone_index["TDP"] = 4;      bone_index["IMCP"] = 5;
    bone_index["IPP"] = 6;      bone_index["IMP"] = 7;      bone_index["IDP"] = 8;
    bone_index["MMCP"] = 9;     bone_index["MPP"] = 10;     bone_index["MMP"] = 11;
    bone_index["MDP"] = 12;     bone_index["RMCP"] = 13;    bone_index["RPP"] = 14;
    bone_index["RMP"] = 15;     bone_index["RDP"] = 16;     bone_index["PMCP"] =17;
    bone_index["PPP"] = 18;     bone_index["PMP"] = 19;     bone_index["PDP"] = 20;

    map<int,QString> atb;
    atb[0] = "TDP";  atb[2] = "IDP";  atb[5] = "MDP";  atb[8] = "RDP";  atb[11] = "PDP";

    double friction_coefficient = fric_coef_young*coef;

    vector<double> mg = {obj_mass*gravity/1000, 0, 0};

    vector<vector<int>> DoFs = { {}, {}, {0,2}, {0,2}, {0}, {}, {0,2}, {0}, {0}, {}, {0,2}, {0},{0},
                                 {}, {0,2}, {0}, {0}, {}, {0,2}, {0}, {0}};


    //グローバル座標系
    vector<int> areas;      //　↓areas定義
    for(size_t i=0; i<color_def.size(); i++)    areas.push_back(color_def[i][0].toInt());

    segment *segm;
    segm = new segment[areas.size()];

    segmentBodyPoints_muscle(bodyPoints, bodySSD, bodyMesh, objMesh, color_def, segm);

    areas.erase(areas.begin()+areas.size()-1);     //34(csvデータ上は33)は対応するboneないので削除
    size_t orgsize = areas.size();
    int sub=0;
    for(size_t area=0; area<orgsize; area++){      //接触が無い部分のareaを削除する
        if(segm[area].BodyPointsID.empty()){
            int inc = area - sub;
            ++sub;
            areas.erase(areas.begin()+inc);        //残ったareasは，接触している部分
        }
    }
    for(size_t area=0; area<areas.size(); area++){
        log2 << areas[area] << endl;
    }
    log2 << endl;

    vector<QString> contact_bones;
    log2 << "contactBones:" << endl;
    for(size_t i=0; i<areas.size(); i++){       //上で抽出したareaに対応するboneをcontact_bonesに格納
        contact_bones.push_back(area_to_bone[areas[i]][1]);
    }
    for(size_t bone=0; bone<contact_bones.size(); bone++){
        log2 << contact_bones[bone] << endl;
    }
    log2 << endl;

    if(contact_bones.size() == 0)
    {
        DH_LOG("Force Closure does not exist.",0);
        return 1.0;
    }

    dhBone* link;
    link = dhnew<dhBone>();
    vector<QString> bones;
    vector<int>     depths;
    getArmatureStructure(arm, link, bones, depths);
    dhdelete(link);

    vector<QString> JacobianBones;
    GetJacobianBones(arm, bones, contact_bones, JacobianBones);

    log2 << "jacobianBones" << endl;
//    DH_LOG("JacobianBones",0);
    for(int i=0;i<JacobianBones.size();i++){
//        DH_LOG(JacobianBones[i],0);
        log2 << JacobianBones[i] << endl;
    }
    log2 << endl << endl;

    vector<int> force_areas;
    vector<double> eT = GetBoundMatrix(segm, areas, bodySSD, ObjPs_normal, force_areas);
    if(eT.size() == 0)
    {
        DH_LOG("Force Closure does not exist.",0);
        return 1.0;
    }

    log2 << "a";
    vector<vector<double>> GraspMatrix = ComputeGraspMatrix(segm, areas, object_center, force_areas);
    log2 << "b";
    vector<vector<double>> FrictionMatrix = ComputeFrictionMatrix(segm, areas, friction_coefficient,
                                                                  ObjPs_normal, force_areas);
    log2 << "c";
    vector<vector<double>> ContactJacobian = ComputeContactJacobian(arm, bone_index, JacobianBones, DoFs,
                                                  contact_bones, areas, force_areas, segm, atb);
    log2 << "d";
    vector<vector<double>> MomentArmForce = GetMomentArm_Force(MP, JacobianBones, DoFs, bone_index);
    log2 << "e";


//    delete segm;

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
        fc_lp2 = GLPK_solve_LP2(left_lp2, right_lp2, coef_lp2, GraspMatrix, ContactJacobian, MomentArmForce);
        if(fc_lp2 >= 0.1){
            FCeval = 0.0;
        }
        else if(fc_lp2 < 0.1){
            FCeval = 100*(fc_lp2 - 0.1)*(fc_lp2 - 0.1);
        }
    }
//    DH_LOG("Force Closure eval is "+QString::number(fc_lp2),0);

    return FCeval;
}
