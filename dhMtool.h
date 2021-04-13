#pragma once
#ifndef DHMTOOL_H
#define DHMTOOL_H

//
//#ifdef DHPLUGINTDLL_EXPORTS
//#define DHPLUGINTDLL_MTOOL_API __declspec(dllexport) 
//#else
//#define DHPLUGINTDLL_MTOOL_API __declspec(dllimport) 
//#endif


#include "IDHElement.h"
#include "geneconst.h"
#include "matrix.h"
#include <vecmath.h>
#include "myJoint.h"
#include <PQP.h>

class dhArmature;
class dhMoCapSequence;
class dhMCControllerBase;
class dhMCArmatureController;
class dhMCRigidBodyController;
class dhMCMarkerController;
class dhFeaturePoints;
class dhSkeletalSubspaceDeformation;
class dhMesh;

class dhMtool : public IDHElement
{
	Q_OBJECT
public:
	DH_EXPOSE_TYPE;

	dhMtool(void);
	virtual ~dhMtool(void);
	//IDHElementのオーバーライド
	virtual const bool	IsValid(void)const{ return (1); }
	virtual void Update(void){}//何もしない設定
	void init();
public slots:
	void ReadJanglenFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName, QString jaxfileName="NOFILE");
	void ReadArmatureMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName);
	void ReadRigidBodyMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhFeaturePoints* obj_fp, QString fileName);
    dhMoCapSequence* ConvertArmatureAndObjPostureInObjCF(dhMoCapSequence *seq, dhSkeletalSubspaceDeformation *ssd, dhMesh *obj, QString newMcpSeqName="mseq_ObjCF");
    dhMoCapSequence* ConvertArmaturePostureInObjCF(dhMoCapSequence *seq, dhSkeletalSubspaceDeformation *ssd, dhMesh *obj, QString newMcpSeqName="mseq_ObjCF");
	void ArmaturePostureInObjCF(dhArmature *arm, dhMesh *obj,bool TransObj=true);
	QString ArmaturePostureInObjCFAsString(dhMoCapSequence *seq);
	QString ArmaturePostureAsString(dhArmature *arm);
	void ArmaturePostureFromGivenString(dhArmature *arm,QString posS);
	QString MarkerNamesAsString(dhMoCapSequence *seq);
	QString MarkerPosInObjCFAsString(dhMoCapSequence *seq);

	dhMoCapSequence* MoCapSeqWRTDifferentReference(dhMoCapSequence *seq, dhArmature *newrefarm);
    dhMCControllerBase* GetGivenNameController(dhMoCapSequence *seq, QString cName);
    dhMCArmatureController* GetExistingMCArmatureController(dhMoCapSequence* seq);
	dhMCRigidBodyController* GetExistingMCRigidBodyController(dhMoCapSequence* seq);
	dhMCMarkerController* GetExistingMCMarkerController(dhMoCapSequence *seq);

    void RegisterCurrentMeshAsKeyFrame(dhMoCapSequence *seq, dhMesh *refM, QString CtrlrName, int frameID);
    void RegisterCurrentFPsAsKeyFrame(dhMoCapSequence *seq, dhFeaturePoints *refFPs, dhMesh *refM, QString CtrlrName, int frameID);
    void RegisterCurrentWrldFPsAsKeyFrame(dhMoCapSequence *seq, dhFeaturePoints *refFPs, QString CtrlrName, int frameID);

    int AddCurrentAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA);
    int AddCurrentArmMeshAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM);
	int AddCurrentArmMeshFPsAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM, dhFeaturePoints *refFPs);
	int AddCurrentArmFPsAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhFeaturePoints *refFPs);
    void CalcRigidBodyMotionConsideringMarkExistence(dhMoCapSequence *seq, dhMesh *refM, dhFeaturePoints *refFPs);

	std::vector<Matrix*> CalCFDefDif(myLink* fL, myLink* tL);
	void saveArmatureAsMylinkCF(dhArmature *arm, QString OFName="NONE");
	dhArmature* CFDefRepairedArmature(dhArmature *arm, bool RightHand=true);
	void superimposeLinks(myLink *fL, myLink *gL);
	void saveArmatureAsMyLink(dhArmature *arm, QString fileName);
	void saveMoCapSequenceArmatureData(dhMoCapSequence *seq, QString fileName);
	void saveMoCapSequenceArmatureDataAsMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName="NONE", dhArmature *refArm=NULL);
	void saveMoCapSequenceArmatureDataAsDifferentCFMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName, dhArmature *refArm, bool Lefty=false);
	void saveMoCapSequenceRigidBodyMotionData(dhMoCapSequence* seq, QString fileName);
	void AppendArmatureOrRigidBodyMotionFiles(QString fnames, QString OFname);
	bool CheckRgdBdyMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *obj_fp, int frameID);
	bool CheckArmatureMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *body_fp, int frameID);
	void AppendFiles();
	void ChangeMeshVcolorMode(dhMesh *trgMesh, bool ShowVcol);
	dhVec4 Vcol(dhMesh *trgMesh, int vID);
	void setVcol(dhMesh *trgMesh, QString fileName);
	void SaveOBJ_withVColor(dhMesh *trgMesh, QString fileName);
	dhMat44 RotMatVec2Vec(dhVec4 &a, dhVec4 &b);
    dhVec4 EquivRotAxis(dhMat44 trM);
    float EquivRotAngle(dhMat44 trM);
    dhMat44 RotMatAroundGivenVec(dhVec4 k, double th);
    dhMat44 TransMat2Registrate3Points(dhVec4 &srcP0, dhVec4 &srcP1, dhVec4 &srcP2, dhVec4 &dstP0,  dhVec4 &dstP1, dhVec4 &dstP2, int basePid=-1);
    dhMat44 TransMat2RegistratePoints(QVariantList& srcPs, QVariantList& dstPs, int dstPID=-1);
	dhMesh* GenerateMirrorMesh(dhMesh *orgMesh, dhVec4 p, dhVec4 n);
	dhArmature* GenerateMirrorArmature(dhArmature* orgArm,dhVec4 p, dhVec4 n, bool resetPosture=true);
	void CopyJointConstraints(dhArmature* fromArm, dhArmature* toArm, bool Mirrored=true);
	void MirrorJointAngle(dhArmature* orgArm, dhArmature* trgArm, dhVec4 p, dhVec4 n);
	dhSkeletalSubspaceDeformation* GenerateMirrorHand(dhSkeletalSubspaceDeformation* ssd, dhVec4 p, dhVec4 n);				
	void setTag(dhMoCapSequence *seq, int frameID, QString theTag);
	void SaveGLScreen(QString FName);

    dhMat44 ObbCF(dhMesh *m, bool BuildBboxFPs=false, dhVec4 refPlusV=dhVec4(0,0,0));
    PQP_Model* BuildPQPModel(dhMesh *m);
    void AnalyzeNormalDifferences_PConMesh(dhMesh *m,double minR, double maxR, int Pnum=50000, QString colType = "hot-to-cold");
    void AnalyzeNormalDifferences_MeshVrtx(dhMesh *m,double minR, double maxR, QString colType = "hot-to-cold");
    dhVec4 GetColor(double v,double vmin,double vmax, QString colType = "hot-to-cold");//jet

public:
	QStringList ElementActionTitles();//static QStringList ElementActionTitles();
	bool OnElementActionCalled(const QString& cmd);//static bool OnElementActionCalled(const QString& cmd);
};

//Q_DECLARE_METATYPE(dhMtool)
Q_DECLARE_METATYPE(dhMtool*)

//class dhMtoolWrapper : public QObject{
//	/// for Python
//	Q_OBJECT
//public slots:
//	//staticの場合は，static_クラス名＿関数名　という名前を宣言する
//	//staticでない関数の場合は，第1引数はdhMtoolのポインタ dhMtool* ptr　　
//	void static_dhMtool_saveArmatureAsMylinkCF(dhArmature *arm, QString OFName="NONE"){dhMtool::saveArmatureAsMylinkCF(arm,OFName);}
//	dhArmature* static_dhMtool_CFDefRepairedArmature(dhArmature *arm, bool RightHand=true){return dhMtool::CFDefRepairedArmature(arm,RightHand);}
//	void static_dhMtool_ReadJanglenFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName, QString jaxfileName="NOFILE"){dhMtool::ReadJanglenFileAndSetMoCapSequence(seq,arm,fileName,jaxfileName);}
//	void static_dhMtool_ReadArmatureMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName){dhMtool::ReadArmatureMotionFileAndSetMoCapSequence(seq,arm,fileName);}
//	void static_dhMtool_ReadRigidBodyMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhFeaturePoints* obj_fp, QString fileName){dhMtool::ReadRigidBodyMotionFileAndSetMoCapSequence(seq,obj_fp,fileName);}
//	dhMoCapSequence* static_dhMtool_ConvertArmaturePostureInObjCF(dhMoCapSequence *seq, dhSkeletalSubspaceDeformation *ssd, dhMesh *obj, QString newMcpSeqName="mseq_ObjCF"){return dhMtool::ConvertArmaturePostureInObjCF(seq,ssd,obj);}
//	void static_dhMtool_ArmaturePostureInObjCF(dhArmature *arm, dhMesh *obj,bool TransObj=true){dhMtool::ArmaturePostureInObjCF(arm,obj,TransObj);}
//	QString static_dhMtool_ArmaturePostureInObjCFAsString(dhMoCapSequence *seq){return dhMtool::ArmaturePostureInObjCFAsString(seq);}
//	QString static_dhMtool_ArmaturePostureAsString(dhArmature *arm){return dhMtool::ArmaturePostureAsString(arm);}
//	void static_dhMtool_ArmaturePostureFromGivenString(dhArmature *arm,QString posS){dhMtool::ArmaturePostureFromGivenString(arm,posS);}
//
//	dhMoCapSequence* static_dhMtool_MoCapSeqWRTDifferentReference(dhMoCapSequence *seq, dhArmature *newrefarm){return dhMtool::MoCapSeqWRTDifferentReference(seq,newrefarm);}
//	dhMCArmatureController* static_dhMtool_GetExistingMCArmatureController(dhMoCapSequence* seq){return dhMtool::GetExistingMCArmatureController(seq);}
//	dhMCRigidBodyController* static_dhMtool_GetExistingMCRigidBodyController(dhMoCapSequence* seq){return dhMtool::GetExistingMCRigidBodyController(seq);}
//	void static_dhMtool_saveMoCapSequenceArmatureData(dhMoCapSequence *seq, QString fileName){dhMtool::saveMoCapSequenceArmatureData(seq,fileName);}
//	void static_dhMtool_saveMoCapSequenceArmatureDataAsMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName="NONE", dhArmature *refArm=NULL){dhMtool::saveMoCapSequenceArmatureDataAsMylink(seq,fileName,JaxfName, refArm);}
//	void static_dhMtool_saveMoCapSequenceArmatureDataAsDifferentCFMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName, dhArmature *refArm, bool Lefty=false){dhMtool::saveMoCapSequenceArmatureDataAsDifferentCFMylink(seq,fileName,JaxfName, refArm, Lefty);}
//	void static_dhMtool_saveMoCapSequenceRigidBodyMotionData(dhMoCapSequence *seq, QString fileName){dhMtool::saveMoCapSequenceRigidBodyMotionData(seq,fileName);}
//	void static_dhMtool_AppendArmatureOrRigidBodyMotionFiles(QString fnames, QString OFname){dhMtool::AppendArmatureOrRigidBodyMotionFiles(fnames,OFname);}
//	bool static_dhMtool_CheckRgdBdyMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *obj_fp, int frameID){return dhMtool::CheckRgdBdyMrkExistenceInMoCapSequence(seq,obj_fp,frameID);}
//	bool static_dhMtool_CheckArmatureMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *body_fp, int frameID){return dhMtool::CheckArmatureMrkExistenceInMoCapSequence(seq,body_fp,frameID);}
//	void static_dhMtool_AppendFiles(dhMtool* ptr){dhMtool::AppendFiles();}
//	int static_dhMtool_AddCurrentAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA){return dhMtool::AddCurrentAsKeyFrame(seq,refA);}
//	int static_dhMtool_AddCurrentArmMeshAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM){return dhMtool::AddCurrentArmMeshAsKeyFrame(seq,refA,refM);}
//	int static_dhMtool_AddCurrentArmMeshFPsAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM, dhFeaturePoints *refFPs){return dhMtool::AddCurrentArmMeshFPsAsKeyFrame(seq,refA,refM,refFPs);}
//	void static_dhMtool_ChangeMeshVcolorMode(dhMesh *trgMesh, bool ShowVcol){dhMtool::ChangeMeshVcolorMode(trgMesh,ShowVcol);}
//	dhVec4 static_dhMtool_Vcol(dhMesh *trgMesh, int vID){return dhMtool::Vcol(trgMesh,vID);}
//	void static_dhMtool_setVcol(dhMesh *trgMesh, QString fileName){dhMtool::setVcol(trgMesh,fileName);}
//	void static_dhMtool_SaveOBJ_withVColor(dhMesh *trgMesh, QString fileName){dhMtool::SaveOBJ_withVColor(trgMesh,fileName);}
//	//void CreateArrow(float headRatio, float ArrowLen){dhMtool::CreateArrow(headRatio,ArrowLen);}
//	dhMat44 static_dhMtool_RotMatVec2Vec(dhVec4 &a, dhVec4 &b){return dhMtool::RotMatVec2Vec(a,b);}
//	dhMesh* static_dhMtool_GenerateMirrorMesh(dhMesh *orgMesh, dhVec4 p, dhVec4 n){return dhMtool::GenerateMirrorMesh(orgMesh,p,n);}
//	dhArmature* static_dhMtool_GenerateMirrorArmature(dhArmature* orgArm,dhVec4 p, dhVec4 n){return dhMtool::GenerateMirrorArmature(orgArm,p,n);}
//	dhSkeletalSubspaceDeformation* static_dhMtool_GenerateMirrorHand(dhSkeletalSubspaceDeformation* ssd, dhVec4 p, dhVec4 n){return dhMtool::GenerateMirrorHand(ssd,p,n);}
//	void static_dhMtool_setTag(dhMoCapSequence *seq, int frameID, QString theTag){dhMtool::setTag(seq,frameID,theTag);}
//};


//
//#include "dhMesh.h"
//
//class dhMeshWrapper2 : public QObject
//{
//Q_OBJECT
//public slots:
//    void setColor(dhMesh* ptr, const int index, const dhVec4& col){ ptr->vlist[index].color = col; }
//};


#endif
