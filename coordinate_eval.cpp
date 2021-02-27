#include<iostream>
#include<vector>
#include<fstream>
#include<sstream>
#include<cstring>
#include<QString>

#include "coordinate_eval.hpp"
#include "csv.hpp"
#include "dhMath.h"
#include "dhFeaturePoint.h"

using namespace std;


void prepare_coordeval(dhFeaturePoints* Fp,vector<vector<QString>>& ObjPs,
                       vector<vector<QString>>& ObjPs_normal, QStringList& fpname)
{
//    QString fpname[] = {"16-a","16-b","16-c","02-a","02-b","02-c","05-a","05-b","05-c",       //母指側面利用
//                        "11-a","11-b","11-c","08-a","08-b","08-c",
//                        };
//    QString fpname[] = {"00-a","00-b","00-c","02-a","02-b","02-c","05-a","05-b","05-c",      //母指腹利用
//                        "11-a","11-b","11-c","08-a","08-b","08-c",
//                        };


    fpname = Fp->allPointNames();

    boost::property_tree::ptree pt;
    read_ini("filepath.ini", pt);

// ===========================
// ObjectPoints3.csvの読み込み
// ==========================
    QString OP;
    if(boost::optional<QString> OP_confirm = pt.get_optional<QString>("path.ObjectPoints")){
        OP = OP_confirm.get();
    }
    else{
        OP = "";
        DH_LOG("ObjectPoints is nothing",0);
    }

    Csv Obj(OP);
    if(!Obj.getCsv(ObjPs)){
        cout << "cannot read ObjectPoints.csv" << endl;
        return ;
    }

    ObjPs.erase(ObjPs.begin());

    /* for(int row=0; row < ObjPs.size(); row++){		//Matrixの中を見る
        vector<string> rec = ObjPs[row];
        for(int col=0; col < rec.size(); col++){
            cout << rec[col];
            if(col < rec.size() -1)	cout << ",";
        }
            cout << endl;
    } */

//==============================
//ObjectNormalvecs3.csvの読み込み
//==============================
    QString ON;
    if(boost::optional<QString> ON_confirm = pt.get_optional<QString>("path.ObjectPoints")){
        ON = ON_confirm.get();
    }
    else{
        ON = "";
        DH_LOG("ObjectNormal is nothing",0);
    }
    Csv Obj_normal(ON);
    if(!Obj_normal.getCsv(ObjPs_normal)){
        cout << "cannot read ObjectNormal.csv" << endl;
        return ;
    }

    ObjPs_normal.erase(ObjPs_normal.begin());

    /* for(int row=0; row < ObjPs_normal.size(); row++){		//Matrixの中を見る
        vector<string> rec = ObjPs_normal[row];
        for(int col=0; col < rec.size(); col++){
            cout << rec[col];
            if(col < rec.size() -1)	cout << ",";
        }
            cout << endl;
    } */

}



float coord_eval(dhFeaturePoints* Fp, const vector<vector<QString>>& ObjPs,
                 const vector<vector<QString>>& ObjPs_normal, const QStringList& fpname){

    const int L = 180;

    double sum_length = 0;
    double sum_rad = 0;
    QStringList finger;

    for(int i=0;i<ObjPs.size();i++){
        dhVec3 ops_forhand(ObjPs[i][1].toDouble(),ObjPs[i][2].toDouble(),ObjPs[i][3].toDouble());
        dhVec3 ops_normal_forhand(ObjPs_normal[i][1].toDouble(),ObjPs_normal[i][2].toDouble(),ObjPs_normal[i][3].toDouble());
        double length,rad,estimate;

        for(int index=0; index<fpname.size(); index++){
            finger = fpname.filter(ObjPs[i][0]);

            for(int j=0; j<finger.size(); j++){
                dhFeaturePoint* np = Fp -> point(finger.value(j));        //特定のFeaturePointを指定

                dhVec4 rps_forobj2 = np -> position();              //選んだFeaturePointの位置座標取得
                dhVec4 rps_normal_forobj2 = np -> normal();         //選んだFeaturePointのベクトル取得

                dhVec3 rps_forobj = rps_forobj2.toVec3();
                dhVec3 rps_normal_forobj = rps_normal_forobj2.toVec3();

                double length_can,rad_can,estimate_can;

//                DH_LOG("ops_forhand is "+QString::number(ops_forhand[0],'f',5),0);
//                DH_LOG("rps_forobj is "+QString::number(rps_forobj[0],'f',5),0);

                length_can = (rps_forobj - ops_forhand).norm()/L;       //距離の評価式

                double cost,theta_can;
                cost = (rps_normal_forobj * ops_normal_forhand)/(rps_normal_forobj.norm() * ops_normal_forhand.norm());
                theta_can = acos(cost);
                rad_can = (PI - theta_can)/PI;                          //角度の評価式

                estimate_can = length_can + rad_can;


                if(j == 0){
                    estimate = estimate_can;
                    length = length_can;
                    rad = rad_can;
                }
                else{
                    if(estimate > estimate_can){
                        estimate = estimate_can;
                        length = length_can;
                        rad = rad_can;
                    }
                }
            }
            finger.clear();
        }

        sum_length += length;
        sum_rad    += rad;

    }

    double estimate_length,estimate_rad,eval;
    estimate_length = sum_length/ObjPs.size();
    estimate_rad    = sum_rad   /ObjPs.size();

    eval = 5*estimate_length + 4*estimate_rad;

//    DH_LOG("estimate_length is "+QString::number(estimate_length,'f',6),0);
//    DH_LOG("estimate_rad is "+QString::number(estimate_rad,'f',6),0);

    return eval;
}

