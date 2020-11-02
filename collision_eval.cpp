#include <string>
#include <map>

#include <pcl/surface/convex_hull.h>
#include <pcl/impl/point_types.hpp>

#include "dhApplication.h"
#include "dhMath.h"
#include "dhArmature.h"
#include "dhMoCapSequence.h"
#include "dhFeaturePoint.h"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhcontact.h"
#include "segment.h"
#include "collision_eval.hpp"


using namespace std;


double collision_eval(dhSkeletalSubspaceDeformation* mesh1, dhMesh* mesh2, dhArmature* arm){

    map<int,string> bone_index;
    bone_index[0] = "ROOT";     bone_index[1] = "CP";       bone_index[2] = "TMCP";
    bone_index[3] = "TPP";      bone_index[4] = "TDP";      bone_index[5] = "IMCP";
    bone_index[6] = "IPP";      bone_index[7] = "IMP";      bone_index[8] = "IDP";
    bone_index[9] = "MMCP";     bone_index[10] = "MPP";     bone_index[11] = "MMP";
    bone_index[12] = "MDP";     bone_index[13] = "RMCP";    bone_index[14] = "RPP";
    bone_index[15] = "RMP";     bone_index[16] = "RDP";     bone_index[17] = "PMCP";
    bone_index[18] = "PPP";     bone_index[19] = "PMP";     bone_index[20] = "PDP";

    double size = hand_length(arm);
//    DH_LOG("hand length is" + QString::number(size),0);

    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();

    DH_LOG("the number of Mesh1 is"+QString::number(mesh1->PointCount()),0);
    DH_LOG("the number of Mesh2 is"+QString::number(mesh2->PointCount()),0);
    dhVertex* vertex = mesh1->Vi(0);

    computeContactRegion(bodyPoints,objectPoints,mesh1,mesh2);

    dhBone* root;
    root = dhnew<dhBone>();
    vector<QString> bones;
    vector<int>     depths;
    getArmatureStructure(arm,bones,depths,root);

    segment* points_obj;
    points_obj = new segment[bones.size()];
    vector<dhVec3> cog_obj;
    vector<int> keys_obj;

    segmentObjectPoints(objectPoints,points_obj,arm,bones);

    for(int index=0; index<bones.size(); index++){      //getCoGsに相当
        points_obj[index].getCoGs(points_obj,cog_obj,index,keys_obj);
    }

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

    segmentObjectPoints(bodyPoints,points_hand,arm,bones);

    for(int index=0; index<bones.size(); index++){      //getCoGsに相当
        points_hand[index].getCoGs(points_hand,cog_hand,index,keys_hand);
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

//                for(int b =0; b<keys_hand.size(); b++){       //削除したbone_numと削除後に残ったkeys_handの確認
//                    DH_LOG(QString::number(bone_num)+" "+QString::number(keys_hand[b]),0);
//                }

        }
    }


    //ハンドモデルについて，干渉点から物体表面までの距離が10mm未満の場合にはその点を除外する
    vector<vector<dhVec3>> points_hand2;

    for(int i=0; i<keys_hand.size(); i++){
        double min_length;
        int sub2 = 0;
        vector<dhVec3> points = points_hand[keys_hand[i]].ObjectPoints;
        for(int j=0; j<points.size(); j++){
            dhVec3 point = points[j];
            for(int k=0; k<keys_obj.size(); k++){
                vector<dhVec3> obj_vecs = points_obj[keys_obj[k]].ObjectPoints;
                for(int l=0; l<obj_vecs.size(); l++){
                    dhVec3 obj_vec = obj_vecs[l];
                    double length = (obj_vec - point).norm();
                    if(l == 0 && k == 0){     min_length = length;}
                    else{
                        if(min_length > length){
                            min_length = length;
                        }
                    }
                }
            }

            if(min_length < 5){
                int inc = j - sub2++;
                points.erase(points.begin() + inc);
            }
        }
        points_hand2.push_back(points);
    }


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


    delete[] points_obj;
    delete[] points_hand;


    dhdelete(root);

    dhdelete(bodyPoints);
    dhdelete(objectPoints);

    DH_LOG(QString::number(Volume),0);

//    return Volume/(size*size*size);
    return Volume/(551.99*551.99*551.99);

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
