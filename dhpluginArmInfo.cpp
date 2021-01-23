#include "stdafx.h"
#include "dhpluginArmInfo.h"
#include<typeinfo>
#include<cmath>
#include<time.h>
#include<stdlib.h>
#include <string>
#include <fstream>
#include <sstream>


#include <QtGui>
#include <QInputDialog>
//#include <gsl/gsl_math.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>


#include <pcl/surface/convex_hull.h>
#include <pcl/impl/point_types.hpp>

#include "csv.hpp"
#include "rom_eval.hpp"
#include "coordinate_eval.hpp"
#include "collision_eval.hpp"
#include "finalpos.h"
#include "dhcontact.h"
#include "force_closure.hpp"
#include <map>



QString dhpluginArmInfo::echo(const QString &message)
{
    return message + "!";
}

#include "dhElementGroup.h"

void dhpluginArmInfo::initPlugin(void)
{
    //
    dhArmOpe* e = dhnew<dhArmOpe>();
    e->SetName("myArmOpe");
    dhElementGroup::findGroupAndAddChild("Plugins", e);
    dhApp::updateAllWindows();

    DH_LOG("dhPluginArmInfo.....Loaded.",0);
}


DH_INIT_TYPE(dhArmOpe, "ArmOpe", "IElement");

dhArmOpe::dhArmOpe()
{
    SET_DEFAULT_ELEM_NAME;
    this->trgA=NULL;
    this->trgSSD=NULL;
    this->trgFPs=NULL;
    this->trgMseq=NULL;
    this->fpname="sample_fp";
}
dhArmOpe::~dhArmOpe()
{
}


void dhArmOpe::writeBoneNum(dhArmature *arm)
{
    DH_LOG(QString("The number of bone is %1").arg(arm->NBones()),0);
}

void dhArmOpe::saveArmInfo(dhArmature *arm, QString IFname)
{
    QFile file( IFname );
    if ( !file.open(QIODevice::WriteOnly | QIODevice::Text) ) {
        DH_LOG( "(dhArmOpe) Cannot open fie: "+IFname, 0 );
        return;
    }
    QTextStream text( &file );

    text << "  Name  :  Angle  :  Position\n";
    text << "-------------------------------------\n";
    if ( arm->NBones() < 1 ) {
        text << " == No bones ==\n";
    }
    for ( int i=0; i < arm->NBones(); i++ ){
        dhBone *Bone = arm->bone(i);
        QString nm = Bone->name;
        //各ボーンの現在の姿勢への変換行列をロールピッチヨー角に変換
        dhVec4 r[2]; Bone->R.getRPYAngle(r[0], r[1]);
        double rx = r[0].p[0] / (2.0*3.141592) * 360.0;
        dhVec4 pos = Bone->Porigin(); // 回転中心の位置ベクトルを得る
        text << QString("%1 :  %2 : %3, %4, %5\n").arg(nm).arg(rx, 0, 'f', 4)
                .arg(pos.p[0], 0, 'f', 3).arg(pos.p[1], 0, 'f', 3).arg(pos.p[2], 0, 'f', 3);
    }

    file.close();

}

void dhArmOpe::saveArmInfoinMoCapSequence(dhArmature *arm, dhMoCapSequence *mocap, QString OFname)
//mocapシーケンスの保有する時系列のArmature姿勢データを，csvファイルとして保存する
{
    QFile file( OFname );
    if ( !file.open(QIODevice::WriteOnly | QIODevice::Text) ) {
        DH_LOG( "(dhArmOpe) Cannot open fie: "+OFname, 0 );
        return;
    }
    QTextStream text( &file );

    //List up bone name in the header Line
    text << "frame";
    for(int i=0; i<arm->NBones(); i++){
        dhBone *bone=arm->bone(i);
        text<<","<<bone->name<<",,";
    }
    text<<"\n";
    //Output armature joint angle in each frame
    for(int mocapIndex=mocap->firstFrame(); mocapIndex < mocap->lastFrame() + 1; mocapIndex +=10){
        mocap->goToFrameAt(mocapIndex);
        text << mocapIndex;
        for(int boneIndex=0; boneIndex<arm->NBones(); boneIndex++){
            dhBone *bone = arm->bone(boneIndex);
            dhVec4 r[2];
            dhMat44 mat=bone->R;
            mat.getRPYAngle(r[0],r[1]); //解が二つ出るが，ここでは0の方のみ書き出すことにする
            double rx = r[0].p[0] / (3.141592) * 180.0;
            double ry = r[0].p[1] / (3.141592) * 180.0;
            double rz = r[0].p[2] / (3.141592) * 180.0;
            text << ","<<rx<< ","<<ry<< ","<<rz;
        }
        text<<"\n";
    }

    file.close();

}


void dhArmOpe::PlotGivenPointTrajectory(dhMoCapSequence *mocap, dhFeaturePoints *fp, QString FPName)
{
    dhFeaturePoint *trgFP=fp->point(FPName);
    if(trgFP!=NULL){
        dhFeaturePoints *fpts=dhnew<dhFeaturePoints>();
        fpts->setRadius(15.0);
        for(int mocapIndex=mocap->firstFrame(); mocapIndex < mocap->lastFrame() + 1; mocapIndex +=10){
            mocap->goToFrameAt(mocapIndex);
            dhFeaturePoint *tmpFP=fpts->addPoint(QString("pt_f%1").arg(mocapIndex));

            tmpFP->setPosition(fp->point(FPName)->position());
        }
        fpts->constructCache();
    }
}

void dhArmOpe::Extract_maxmin(QString in){
//莫大なデータ量のassembledActiveDF01.csvから必要な最大最小値を先に取り出しておく関数

    vector<vector<QString>> DF;
    double buf,min,max;

    Csv objDF(in);
    objDF.getCsv(DF);

    QString out = in.remove(".csv") + "_maxmin.csv";;   //抜き出し先ファイル

    QFile opfile(out);
    if(!opfile.open(QIODevice::WriteOnly)){
        DH_LOG("cannot open the output file",0);
    }

    QTextStream op(&opfile);

    for(int col=2; col<DF[0].size()-2; col++){     //max and min, assembledActiveDF01.csvのデータは3列目からAU列まで
        for(int row=1; row<DF.size(); row++){
            buf = DF[row][col].toDouble();
            if(row == 1){
                min = buf;
                max = buf;
            }
            else{
                if(min>buf)	min=buf;
                if(max<buf)	max=buf;
            }
        }

        op << DF[0][col] << ',';
        op << max << ',';
        op << min << '\n';
    }

//以下，QStringを用いたものに書き換えたため不要（念のため，残しておく）=============================================================

//    ofstream ofs_output(out);

//    for(int col=2; col<DF[0].size()-2; col++){     //max and min, assembledActiveDF01.csvのデータは3列目からAU列まで
//        ofs_output << DF[0][col] << ',';
//        for(int row=1; row<DF.size(); row++){
//            buf = stof(DF[row][col]);
//            if(row == 1){
//                min = buf;
//                max = buf;
//            }
//            else{
//                if(min>buf)	min=buf;
//                if(max<buf)	max=buf;
//            }
//        }
//        ofs_output << max << ',';
//        ofs_output << min << std::endl;
//    }

//================================================================================================================
    DH_LOG("Completed!",0);
}

double dhArmOpe::RoM_evaluation(dhArmature* arm, int age)
{
    vector<vector<QString>> joints_list;
    vector<vector<QString>> joint_bone;
    vector<vector<QString>> DF;
    vector<alphashape> ashape_all;

    prepare_romeval(joints_list, joint_bone, DF, ashape_all, age);

    return rom_eval(arm,joints_list,joint_bone,DF,ashape_all);
}

double dhArmOpe::Coordinate_evaluation(dhFeaturePoints* fp)
{
    vector<vector<QString>> ObjPs;
    vector<vector<QString>> ObjPs_normal;
    QStringList fpname;

    prepare_coordeval(fp, ObjPs, ObjPs_normal, fpname);

    return coord_eval(fp, ObjPs, ObjPs_normal, fpname);
}

double dhArmOpe::Collision_evaluation(dhSkeletalSubspaceDeformation* ssd, dhMesh* objMesh, dhArmature* arm)
{
    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();
    extract_contactPoints(ssd, objMesh, bodyPoints, objectPoints);

    double eval = collision_eval(arm, bodyPoints, objectPoints);
    dhdelete(bodyPoints);
    dhdelete(objectPoints);

    return eval;
}

double dhArmOpe::ForceClosure_evaluation(dhArmature* arm, dhSkeletalSubspaceDeformation* bodySSD,
                                         dhMesh* bodyMesh, dhMesh* objMesh, int age)
{
    vector<vector<QString>> MP;
    vector<vector<QString>> color_def;
    vector<vector<QString>> area_to_bone;
    double coef;
    dhPointCloudAsVertexRef* bodyPoints = dhnew<dhPointCloudAsVertexRef>();
    dhPointCloudAsVertexRef* objectPoints = dhnew<dhPointCloudAsVertexRef>();

    prepare_forceClosure(MP, color_def, area_to_bone, coef, age);
    extract_contactPoints(bodySSD, objMesh, bodyPoints, objectPoints);

    double eval = forceClosure_eval(arm, bodySSD, bodyMesh, objMesh, MP, color_def, area_to_bone, bodyPoints, coef);
    dhdelete(bodyPoints);
    dhdelete(objectPoints);

    return eval;
}

void dhArmOpe::FinalPosture_create(dhArmature* arm,dhFeaturePoints* Fp, dhSkeletalSubspaceDeformation* ssd,
                                   dhMesh* handMesh, dhMesh* objMesh, int age)
{
    FinalPostureCreate(arm, Fp, ssd, handMesh, objMesh, age);
//    FinalPostureCreate_PSO(arm, Fp, ssd, handMesh, objMesh, age);
}

QStringList dhArmOpe::ElementActionTitles()
{
    QStringList sl=IDHElement::ElementActionTitles();
    sl<<"Get Bone Num"<<"Save Armature Info in File"<<"Save Armature Angle through MoCapSequence"
     <<"Add Feature Point on the Body"<<"Extract max and min"<<"RoM evaluation"<<"Coordinate evaluation"
    <<"Collision evaluation"<<"FinalPostureCreate"<<"Force Closure";
    return sl;
}

bool dhArmOpe::OnElementActionCalled(const QString& cmd)
{
    //# エレメントの名前
    QString armName   = "body_arm";
    QString ssdName   = "body_ssd";
    QString mocapName   = "myMoCapSequence";

    if(cmd=="Get Bone Num"){
        bool isOK;
        //Armatureをdialogで指定する
        IDHElement* e=dhApp::elementSelectionDialog(dhArmature::type,&isOK);

        if(isOK&& e){
            this->writeBoneNum(dynamic_cast<dhArmature*>(e));
        }
        return true;

    }else if(cmd=="Save Armature Info in File"){
        bool isOK;
        //Armatureをdialogで指定する
        IDHElement* e=dhApp::elementSelectionDialog(dhArmature::type,&isOK);
        if(isOK&& e){
            QString fileName = QFileDialog::getSaveFileName(0,"Select file to save armature infos","","txt (*.txt)");
            if(!fileName.isEmpty()){
                this->saveArmInfo(dynamic_cast<dhArmature*>(e),fileName);
            }else{
                DH_LOG("File was not selected",0);
            }
        }
        return true;

    }else if(cmd=="Save Armature Angle through MoCapSequence"){
        bool isOK;
        dhMoCapSequence *mocap= dynamic_cast<dhMoCapSequence*>(dhApp::elementSelectionDialog(dhMoCapSequence::type,&isOK));
        dhArmature *arm =dynamic_cast<dhArmature*>(dhApp::elementSelectionDialog(dhArmature::type,&isOK));
        QString fileName = QFileDialog::getSaveFileName(0,"Select file to save armature infos","","csv (*.csv)");
        if(mocap!=NULL && arm!=NULL && !fileName.isEmpty()){
            this->saveArmInfoinMoCapSequence(arm,mocap,fileName);
        }
        return true;

    }else if(cmd=="Add Feature Point on the Body"){
        bool isOK;

        DH_LOG("Add Feature Point on the Body",0);
        //Set Mesh Supplier
        this->trgSSD=dynamic_cast<dhSkeletalSubspaceDeformation*>(dhApp::elementSelectionDialog(dhSkeletalSubspaceDeformation::type,&isOK));
        IDHMeshSupplier *msply =dynamic_cast<IDHMeshSupplier*>(this->trgSSD);
        this->fpname=QInputDialog::getText(0,"Input FeaturePoint Name","Feature point name to be added",QLineEdit::Normal,this->fpname);
        if(msply!=NULL && !fpname.isEmpty()){
            dhFeaturePoints* fpts=dhnew<dhFeaturePoints>();
            //Set Mesh Supplier
            fpts->setPointSupplier(msply);
            fpts->setRadius(20.0);
            dhFeaturePoint *tmpFP=fpts->addPoint(this->fpname);
            tmpFP->setVertexID(436);
            fpts->constructCache();

            return true;
        }

    }

    else if(cmd == "Extract max and min")
    {

        QString DF = QFileDialog::getOpenFileName(nullptr,"Input CSV file name","","csv(*.csv)");
        this->Extract_maxmin(DF);

        return true;
    }

    else if(cmd == "RoM evaluation")
    {
        bool isOK,ok;
        IDHElement* e=dhApp::elementSelectionDialog(dhArmature::type,&isOK);
        if(isOK){
            dhArmature* arm = dynamic_cast<dhArmature*>(e);

            int age = QInputDialog::getInt(nullptr, tr("Input the target age"),
                                             tr("from 20 to 100 years old"), 20, 20, 100, 1, &ok);

            if(ok){
                double rom_eva;
                clock_t time1,time2;
                time1 = clock();

                rom_eva = this->RoM_evaluation(arm,age);

                time2 = clock();
                double etime = (double)(time2-time1)/1000;
                DH_LOG("RoM evaluation is "+QString::number(rom_eva,'f',5),0);
                DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);
            }
        }

        return true;
    }

    else if(cmd == "Coordinate evaluation")
    {
        bool isOK;
        IDHElement* e=dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK);

        if(isOK){
            dhFeaturePoints* Fp = dynamic_cast<dhFeaturePoints*>(e);

            float co_eva;
            clock_t time1,time2;
            time1 = clock();

            co_eva = this->Coordinate_evaluation(Fp);

            time2 = clock();
            double etime = (double)(time2-time1)/1000;
            DH_LOG("Coordinate evaluation is "+QString::number(co_eva,'f',5),0);
            DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);
        }

        return true;
    }

    else if(cmd == "Collision evaluation")
    {
        bool isOK,isOK2,isOK3;

        IDHElement* e1 = dhApp::elementSelectionDialog(dhSkeletalSubspaceDeformation::type,&isOK);
        if(isOK){
            dhSkeletalSubspaceDeformation* mesh1 = dynamic_cast<dhSkeletalSubspaceDeformation*>(e1);

            IDHElement* e2 = dhApp::elementSelectionDialog(dhMesh::type,&isOK2);
            if(isOK2){
                dhMesh* mesh2 = dynamic_cast<dhMesh*>(e2);

                IDHElement* e3 = dhApp::elementSelectionDialog(dhArmature::type,&isOK3);
                if(isOK3){
                    dhArmature* arm = dynamic_cast<dhArmature*>(e3);

                    clock_t time1,time2;
                    time1 = clock();
                    double handcol_eva;

                    handcol_eva = this->Collision_evaluation(mesh1, mesh2, arm);

                    time2 = clock();
                    double etime = (double)(time2-time1)/1000;
                    DH_LOG("Collision evaluation is "+QString::number(handcol_eva,'f',8),0);
                    DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);
                }
            }
        }

        return true;
    }

    else if(cmd == "Force Closure")
    {
        bool isOK,isOK2,isOK3,isOK4, ok;

        IDHElement* e1 = dhApp::elementSelectionDialog(dhArmature::type,&isOK);
        if(isOK){
            dhArmature* arm = dynamic_cast<dhArmature*>(e1);

            IDHElement* e2 = dhApp::elementSelectionDialog(dhSkeletalSubspaceDeformation::type,&isOK2);
            if(isOK2){
                dhSkeletalSubspaceDeformation* ssd = dynamic_cast<dhSkeletalSubspaceDeformation*>(e2);

                IDHElement* e3 = dhApp::elementSelectionDialog(dhMesh::type,&isOK3);
                if(isOK3){
                    dhMesh* bodyMesh = dynamic_cast<dhMesh*>(e3);

                    IDHElement* e4 = dhApp::elementSelectionDialog(dhMesh::type,&isOK4);
                    if(isOK4){
                        dhMesh* objMesh = dynamic_cast<dhMesh*>(e4);

                        int age = QInputDialog::getInt(nullptr, tr("Input the target age"),
                                                         tr("from 20 to 100 years old"), 20, 20, 100, 1, &ok);

                        if(ok){
                            double fceval = this->ForceClosure_evaluation(arm, ssd, bodyMesh, objMesh, age);
                        }
                    }
                }
            }
        }
        return true;
    }

    else if(cmd == "FinalPostureCreate")
    {
        clock_t time1,time2;
        time1 = clock();
        bool isOK,isOK2,isOK3,isOK4,isOK5,ok;
        IDHElement* e1=dhApp::elementSelectionDialog(dhArmature::type,&isOK);
        if(isOK){
            dhArmature* arm = dynamic_cast<dhArmature*>(e1);

            IDHElement* e2=dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK2);
            if(isOK2){
                dhFeaturePoints* Fp = dynamic_cast<dhFeaturePoints*>(e2);

                IDHElement* e3 = dhApp::elementSelectionDialog(dhSkeletalSubspaceDeformation::type,&isOK3);
                if(isOK3){
                    dhSkeletalSubspaceDeformation* ssd = dynamic_cast<dhSkeletalSubspaceDeformation*>(e3);

                    IDHElement* e4 = dhApp::elementSelectionDialog(dhMesh::type,&isOK4);
                    if(isOK4){
                        dhMesh* handMesh = dynamic_cast<dhMesh*>(e4);

                        IDHElement* e5 = dhApp::elementSelectionDialog(dhMesh::type,&isOK5);
                        if(isOK5){
                            dhMesh* objMesh = dynamic_cast<dhMesh*>(e5);

                            int age = QInputDialog::getInt(nullptr, tr("Input the target age"),
                                                             tr("from 20 to 100 years old"), 20, 20, 100, 1, &ok);
                            if(ok){
                                this->FinalPosture_create(arm, Fp, ssd, handMesh, objMesh, age);

                                time2 = clock();
                                double etime = (double)(time2-time1)/1000;
                                DH_LOG("elapsed time is "+QString::number(etime,'f',5),0);
                            }
                        }
                    }
                }
            }
        }

        return true;
    }


    return false;
}


void dhArmOpe::ConstructObjectProperty(void)
{
    IDHElement::ConstructObjectProperty();
    auto ElemName = [&](IDHElement* element)->QString{
        return element ? element->Name() : QString();
    };
    QString Tag1 = QString("Sample Action1:Armature infos");
    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag1, dhLang::tr("Target Armature"), "class:Armature", ElemName(trgA));
    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag1, dhLang::tr("Get Bone Num"), "button", QString());
//    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag1, dhLang::tr("Save Armature Info in File"), "button", QString());
    QString Tag2 = QString("Sample Action2:Plot Trajectory of the Point");
    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag2, dhLang::tr("Target MoCapSequence"), "class:MoCapSequence", ElemName(trgMseq));
    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag2, dhLang::tr("Target FPs"), "class:FeaturePoints", ElemName(this->trgFPs));
    DH_INSERT_TAGGED_OBJECT_PROPERTY(Tag2, dhLang::tr("Plot Trajectory of the Point"), "button", QString());
}



void dhArmOpe::OnObjectPropertyUpdated(const QString& propName)
{
    IDHElement::OnObjectPropertyUpdated(propName);
    bool isOK;

    dhPropertyValue& property = mObjectPropertyMap[propName];
    if(propName == dhLang::tr("Target Armature")){
        this->trgA = dynamic_cast<dhArmature*>(dhApp::findElement(property.value<QString>()));
    }
    else if(propName == dhLang::tr("Get Bone Num")){
        if(this->trgA!=NULL){
            this->writeBoneNum(this->trgA);
        }
    }
//    else if(propName == dhLang::tr("Save Armature Info in File")){
//        if(this->trgA==NULL)
//            this->trgA = dynamic_cast<dhArmature*>(dhApp::findElement(property.value<QString>()));
//        if(this->trgA!=NULL){
//            QString fileName = QFileDialog::getSaveFileName(0,"Select file to save armature infos","","txt (*.txt)");
//            if(!fileName.isEmpty()){
//                this->saveArmInfo(this->trgA,fileName);
//            }else{
//                DH_LOG("File was not selected",0);
//            }
//        }
//    }
    else if(propName == dhLang::tr("Target MoCapSequence")){
        this->trgMseq = dynamic_cast<dhMoCapSequence*>(dhApp::findElement(property.value<QString>()));
    }
    else if(propName == dhLang::tr("Target FPs")){
        this->trgFPs = dynamic_cast<dhFeaturePoints*>(dhApp::findElement(property.value<QString>()));
    }
    else if(propName == dhLang::tr("Plot Trajectory of the Point")){
        bool isOK1,isOK2;
        if(this->trgMseq==NULL) this->trgMseq=dynamic_cast<dhMoCapSequence*>(dhApp::elementSelectionDialog(dhMoCapSequence::type,&isOK1));
        if(this->trgFPs==NULL) this->trgFPs =dynamic_cast<dhFeaturePoints*>(dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK2));
        if(this->trgFPs!=NULL){
            QStringList fpnames;
            for(int i=0; i<this->trgFPs->pointCount(); i++) fpnames.append(this->trgFPs->point(i)->Name());
            this->fpname = QInputDialog::getItem(0,"Set FP to be plotted","Select one of the names",fpnames,0);
        }
        if(this->trgMseq!=NULL && this->trgFPs!=NULL && !this->fpname.isEmpty()){
            this->PlotGivenPointTrajectory(this->trgMseq,this->trgFPs,this->fpname);
        }

    }
}
