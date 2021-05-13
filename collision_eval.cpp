#include <string>
#include <map>
#include <time.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/impl/point_types.hpp>

#include "dhApplication.h"
#include "dhMath.h"
#include "dhArmature.h"
#include "dhMoCapSequence.h"
#include "dhFeaturePoint.h"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhcontact.h"
#include "collision_eval.hpp"
#include "csv.hpp"


using namespace std;

void prepare_colleval(dhArmature* arm, double &hand_size, dhPointCloud* internal, dhMesh* objMesh,
                      vector<vector<QString>> &input_set)
{
//    string fn = "C:\\Users\\nakanishi\\Desktop\\log_setting.txt";
//    ofstream log(fn, ios::app);

    boost::property_tree::ptree pt;
    read_ini("filepath.ini", pt);
//=============================
// settings.csvの読み込み
//=============================
    QString setting;
    if(boost::optional<QString> setting_confirm = pt.get_optional<QString>("path.settings")){
        setting = setting_confirm.get();
    }
    else{
        setting = "";
        DH_LOG("settings is nothing",0);
    }

    Csv Obj_set(setting);
    if(!Obj_set.getCsv(input_set)){
        DH_LOG("cannot read settings.csv",0);
        return ;
    }
//    for(int i=0; i<input_set.size(); i++){
//        for(int j=0; j<input_set[i].size(); j++){
//            log << input_set[i][j] << endl;
//        }
//    }

//    log << "path1" << endl;
    hand_size = hand_length(arm);
//    log << "path2" << endl;
    generate_points_inobject(internal, objMesh, input_set);
//    log << "path3";
}



void generate_points_inobject(dhPointCloud* &internal, dhMesh* objMesh, vector<vector<QString>> input_set)
{
    string fn = "C:\\Users\\nakanishi\\Desktop\\seed_value.txt";
    ofstream log(fn, ios::app);

    time_t t = time(NULL);
    struct tm *local = localtime(&t);

    log << local->tm_mon + 1; log << "/";
    log << local->tm_mday;  log << "-";

    log << local->tm_hour ; log << ":";
    log << local->tm_min;   log << ":";
    log << local->tm_sec;   log << " -> ";

    const int OBJ_X = input_set[2][1].toDouble();
    const int OBJ_Y = input_set[2][2].toDouble();
    const int OBJ_Z = input_set[2][3].toDouble();

    const int Number_of_Pts = 5000;

    unsigned int time = (unsigned int)clock();
    log << time << endl;
    srand(time);
    int i=0;
    while(internal->pts.size() < Number_of_Pts){
        double x = rand() % OBJ_X + 1;
        double y = rand() % OBJ_Y + 1;
        double z = rand() % OBJ_Z + 1;

        //球の時
        if(input_set[4][1].toInt() == 2){
            x = x - OBJ_X/2;
            y = y - OBJ_Y/2;
            z = z - OBJ_Z/2;
            if((OBJ_Z/2)*(OBJ_Z/2) < x*x + y*y + z*z)   continue;
        }
        //ここまで
        //円柱の時
        if(input_set[4][1].toInt() == 3){
            x = x - OBJ_X/2;
            y = y - OBJ_Y/2;
            if((OBJ_X/2)*(OBJ_X/2) < x*x + y*y) continue;
        }
        //ここまで

        const dhVec4 pos(x, y, z, 0);

        dhPointAsIndependent pt;

        internal->pts << pt;
        internal->setPosition(internal->pts[i], pos);
        ++i;
    }

    internal->mergePoints(objMesh);     //物体内部に生成した点群と物体表面点群をマージする

}




//先に接触点群を抽出しておく関数
void extract_contactPoints(dhSkeletalSubspaceDeformation* ssd, dhPointCloud* internal,
                           dhPointCloudAsVertexRef* &bodyPoints, dhPointCloudAsVertexRef* &objectPoints)
{
    computeContactRegion(bodyPoints, objectPoints, ssd, internal);
}



double collision_eval(dhArmature* arm, dhPointCloudAsVertexRef* &bodyPoints,
                      dhPointCloudAsVertexRef* &objectPoints, double hand_size){

    map<int,string> bone_index;
    bone_index[0] = "ROOT";     bone_index[1] = "CP";       bone_index[2] = "TMCP";
    bone_index[3] = "TPP";      bone_index[4] = "TDP";      bone_index[5] = "IMCP";
    bone_index[6] = "IPP";      bone_index[7] = "IMP";      bone_index[8] = "IDP";
    bone_index[9] = "MMCP";     bone_index[10] = "MPP";     bone_index[11] = "MMP";
    bone_index[12] = "MDP";     bone_index[13] = "RMCP";    bone_index[14] = "RPP";
    bone_index[15] = "RMP";     bone_index[16] = "RDP";     bone_index[17] = "PMCP";
    bone_index[18] = "PPP";     bone_index[19] = "PMP";     bone_index[20] = "PDP";


    dhBone* link;
    link = dhnew<dhBone>();
    vector<QString> bones;
    vector<int>     depths;
    getArmatureStructure(arm, link, bones, depths);

//    segment* points_obj;
//    points_obj = new segment[bones.size()];
//    vector<dhVec3> cog_obj;
//    vector<int> keys_obj;

//    segmentObjectPoints(objectPoints,points_obj,arm,bones);     //objectPointsをsegment[最短bone]に所属させる

//    for(int index=0; index<bones.size(); index++){
//        if(points_obj[index].ObjectPoints.empty()){;    //==true削除
//        }
//        else{
//            cog_obj.push_back(points_obj[index].ObjectCoG);
//            keys_obj.push_back(index);
//        }
//    }

//        for(int k=0; k<cog_obj.size(); k++){          // CoGとそのbone名を出力(物体側)
//            dhVec3 tmp = cog_obj[k];
//            QString s = QString::fromStdString(bone_index[keys_obj[k]]);
//            QString x = QString::number(tmp[0],'f',5);
//            QString y = QString::number(tmp[1],'f',5);
//            QString z = QString::number(tmp[2],'f',5);
//            DH_LOG("bone:"+s+" x="+x+" y="+y+" z="+z,0);
//        }

    segment* points_hand;
    points_hand = new segment[bones.size()];
    vector<dhVec3> cog_hand;
    vector<int> keys_hand;

    segmentObjectPoints(bodyPoints, points_hand, arm, bones);

    for(int index=0; index<bones.size(); index++){
        if(points_hand[index].ObjectPoints.empty()){;
        }
        else{
            cog_hand.push_back(points_hand[index].ObjectCoG);
            keys_hand.push_back(index);
        }
    }

//    for(int k=0; k<cog_hand.size(); k++){         // CoGとそのbone名を出力(ハンドモデル側)
//        dhVec3 tmp = cog_hand[k];
//        QString s = QString::fromStdString(bone_index[keys_obj[k]]);
//        QString x = QString::number(tmp[0],'f',5);
//        QString y = QString::number(tmp[1],'f',5);
//        QString z = QString::number(tmp[2],'f',5);
//        DH_LOG("bone:"+s+" x="+x+" y="+y+" z="+z,0);
//    }


    vector<int> keys_hand_orig = keys_hand;

    int sub=0;
    //ハンドモデルについて干渉点が10点以下の場合，keys_handとcog_handから除外する
    for(int bone_num=0; bone_num < keys_hand_orig.size(); bone_num++){
        if(points_hand[keys_hand_orig[bone_num]].ObjectPoints.size() < 10){
            int inc = bone_num - sub;
            sub++;
            keys_hand.erase(keys_hand.begin() + inc);
            cog_hand.erase(cog_hand.begin() + inc);

//            for(int b =0; b<keys_hand.size(); b++){       //削除したbone_numと削除後に残ったkeys_handの確認
//                DH_LOG(QString::number(bone_num)+" "+QString::number(keys_hand[b]),0);
//            }

        }
    }

    vector<vector<dhVec3>> points_hand2;

    for(int i=0; i<keys_hand.size(); i++){
        vector<dhVec3> seg_pts = points_hand[keys_hand[i]].ObjectPoints;
        points_hand2.push_back(seg_pts);
    }



//ハンドモデルについて，干渉点から物体表面までの距離が5mm未満の場合にはその点を除外する  ->　dhCollisionDetection関数で考慮済
//    vector<dhVec3> opoints;
//    getPointsFromCloud(objectPoints, opoints);
//    for(int i=0; i<keys_hand.size(); i++){
//        vector<dhVec3> hpoints = points_hand[keys_hand[i]].ObjectPoints;
//        for(int j=0; j<hpoints.size(); j++){
//            double min_length = DBL_MAX;
//            dhVec3 hpoint = hpoints[j];
//            for(int k=0; k<opoints.size(); k++){
//                dhVec3 opoint = opoints[k];
//                double length = (opoint - hpoint).norm();

//                if(min_length > length){
//                    min_length = length;
//                }
//            }

//        }
//    }



//    for(int i=0; i<keys_hand.size(); i++){     //O^4　->アルゴリズムチェンジ必要，そもそもミスってる
//        int sub2 = 0;
//        vector<dhVec3> points = points_hand[keys_hand[i]].ObjectPoints;
//        for(int j=0; j<points.size(); j++){
//            double min_length = DBL_MAX;
//            dhVec3 point = points[j];
//            for(int k=0; k<keys_obj.size(); k++){
//                vector<dhVec3> obj_vecs = points_obj[keys_obj[k]].ObjectPoints;
//                for(int l=0; l<obj_vecs.size(); l++){
//                    dhVec3 obj_vec = obj_vecs[l];
//                    double length = (obj_vec - point).norm();
//                    DH_LOG("length is "+QString::number(length),0);
//                    if(min_length > length){
//                        min_length = length;
//                    }

//                }
//            }
//            if(min_length > 10){
//                int inc = j - sub2++;
//                points.erase(points.begin() + inc);
//            }
//        }
//        points_hand2.push_back(points);
//    }

//    for(int cnt=0; cnt<points_hand2.size(); cnt++){       //points_hand2の中身チェック
//        DH_LOG("bone is "+QString::fromStdString(bone_index[keys_hand[cnt]]),0);
//        DH_LOG("Size is "+QString::number(points_hand2[cnt].size()),0);
//        for(int jj=0; jj<points_hand2[cnt].size(); jj++){
//            dhVec3 tmp = points_hand2[cnt][jj];
//            QString x = QString::number(tmp[0],'f',7);
//            QString y = QString::number(tmp[1],'f',7);
//            QString z = QString::number(tmp[2],'f',7);
//           // DH_LOG("x="+x+" y="+y+" z="+z,0);
//            DH_LOG("["+x+","+y+","+z+"],",0);
//        }
//    }


//========================================
//PCLを使ってConvexHull作成　→　体積求値
//========================================

    double Volume = 0;

#pragma omp for default(private) reduction(+:Volume)
    for(int bone=0; bone<points_hand2.size(); bone++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for(int j=0; j<points_hand2[bone].size(); j++){
            pcl::PointXYZ buf(points_hand2[bone][j][0],points_hand2[bone][j][1],points_hand2[bone][j][2]);
            Cloud->push_back(buf);
        }

        pcl::ConvexHull<pcl::PointXYZ>::Ptr CHull(new pcl::ConvexHull<pcl::PointXYZ>);
        CHull->setInputCloud(Cloud);
        CHull->setDimension(3);
        CHull->setComputeAreaVolume(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
        CHull->reconstruct(*hull);

//            DH_LOG("          ",0);
//            for(int cnt =0; cnt<hull->size(); cnt++){        //中身確認
//                QString x = QString::number(hull->points[cnt].x,'f',7);
//                QString y = QString::number(hull->points[cnt].y,'f',7);
//                QString z = QString::number(hull->points[cnt].z,'f',7);
//                DH_LOG("x="+x+" y="+y+" z="+z,0);
//            }

        Volume += CHull->getTotalVolume();
    }

//    delete[] points_obj;
    delete[] points_hand;

    dhdelete(link);

//    dhdelete(bodyPoints);
//    dhdelete(objectPoints);

//    DH_LOG(QString::number(Volume),0);

    return Volume/(hand_size*hand_size*hand_size);

}

double hand_length(dhArmature* arm){
    dhMat44 M1,M2,M3,M4;        //中指の長さをhand_lengthとする
    M1 = arm->bone(9)->Tpj0;    //MMCP
    M2 = arm->bone(10)->Tpj0;   //MPP
    M3 = arm->bone(11)->Tpj0;   //MMP
    M4 = arm->bone(12)->Tpj0;   //MDP

    dhVec3 m1(M1[12],M1[13],M1[14]);    //右三つ(0~2行 3列目)抽出
    dhVec3 m2(M2[12],M2[13],M2[14]);    //右三つ(0~2行 3列目)抽出
    dhVec3 m3(M3[12],M3[13],M3[14]);    //右三つ(0~2行 3列目)抽出
    dhVec3 m4(M4[12],M4[13],M4[14]);    //右三つ(0~2行 3列目)抽出

    double length = m1.norm() + m2.norm() + m3.norm() + m4.norm();

    return length;
}
