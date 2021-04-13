#include "stdafx.h"
#include "dhMtool.h"

#include "IDHMeshSupplier.h"
#include "dhMesh.h"
#include "dhMath.h"
#include "dhArmature.h"
#include "dhSkeletalSubspaceDeformation.h"
#include "dhMCControllers.h"
#include "dhShapePoint.h"
#include "dhArmatureController.h"
#include "dhMoCapSequence.h"
#include "dhMCControllers.h"
#include "utilfuncDW.h"
#include "softpol.h"
#include "matrix.h"
#include <PQP.h>
#include <map>
#include "utilfunc.h"
#include "dhShapeCapsule.h"
#include "IDHGeometryShape.h"
//#include "dhShapeLine.h"
#include "dhShapeCylinder.h"
#include "dhColorPointCloud2.h"

//scriptの方で，static 関数は　dhMtool.** という形で呼び出すことになる
DH_INIT_TYPE(dhMtool, "Mtool", "IElement");

DH_DECLARE_SCRIPT_TYPE_CREATOR_FOR_QOBJECT(dhMtool)
//Wrapper用
//DH_DECLARE_DECORATOR_FOR_QOBJECT(dhMtoolWrapper)
//DH_DECLARE_DECORATOR_FOR_QOBJECT(dhMeshWrapper2)


dhMtool::dhMtool(void)
{
	SET_DEFAULT_ELEM_NAME
}

dhMtool::~dhMtool(void)
{
	//
}

void dhMtool::init()
{

}

void dhMtool::ReadJanglenFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName, QString jaxfileName)
{
	if(!seq || fileName=="" ){
		if(!seq) DH_LOG("ReadJanglenFileAndSetMoCapSequence: exit due to empty MoCapSequence",0);
		if(fileName=="") DH_LOG("ReadJanglenFileAndSetMoCapSequence: exit due to empty filename",0);
	}else{
		dhMCArmatureController* c=GetExistingMCArmatureController(seq);
		if(c==NULL) c= seq->addArmatureController(arm);//seq->addArmatureMotion(arm);
		myLink *tmpL = BuildmyLinkFromArm(arm,myLink::LMType::LM3);
		if (jaxfileName!="NOFILE"){
			tmpL->readJaxis(jaxfileName);
		}
		QFile f(fileName);
		if (f.open(QIODevice::ReadOnly | QIODevice::Text)){
			QTextStream fin(&f);
			QString textLine, Jn;
			QStringList cntntslist,Jns,fields;
			std::map<QString,int> DatDofMode;	//角度ファイルによっては，１DOF関節のデータだけど３DOFモードで書いてある場合があるので
			std::map<QString,int>::iterator p_ddm;

			//1行目は説明　ここから，関節の並びとそれぞれのデータのDOFモードを覚えておく
			textLine = fin.readLine();
			cntntslist = textLine.split(",", QString::KeepEmptyParts);
			//０列目はTime １からTrootJ 16要素
			Jns << tmpL->TrootJ->jmname;
			DatDofMode.insert(pair<QString,int>(tmpL->TrootJ->jmname,3));
			//17行目以降は　残り関節角度
			int ThisJointDatDofMode;
			for(int j=17; j<cntntslist.size(); j+=3){
				ThisJointDatDofMode = 3;	//Defaultは３DOFモード
				fields = cntntslist[j].split(QRegExp("[ :]"));
				if(!fields[0].isEmpty()){
					Jn = fields[0];
					if(fields[1]=="th") ThisJointDatDofMode=1;
				}else{
					//最初に無駄にスペースが入っている場合があるので
					int i=1;
					while(Jn.isEmpty()&&i<fields.size()) Jn = fields[i++];
					if(i<fields.size()){
						if(fields[i]=="th") ThisJointDatDofMode=1;
					}
				}
				if(!Jn.contains("palm1") && !Jn.contains("thumb")){
					Jns << Jn;
					DatDofMode.insert(pair<QString,int>(Jn,ThisJointDatDofMode));
				}
			}

			bool TM0_in_W = true;
			//TM0の位置情報が，jiniTjかwoTjかを判定しておく
			if(cntntslist[cntntslist.size()-3].contains("jiniTj")){
				TM0_in_W = false;
				cout << "ReadJanglenFileAndSetMoCapSequence:  This jangle file is written in jiniTj mode\n";
			}
			Vector3d xax, yax, zax, cn;
			int frameID=0;
			while(!fin.atEnd()){
				tmpL->ResetPosture();
				textLine = fin.readLine();
				fields = textLine.split(",", QString::KeepEmptyParts);

				//FrameIDを読み込んでおく
				tmpL->frameID = fields[0].toInt();
				int accumnum=1;
				std::map<QString,myJoint*>::iterator p;
				for(int j=0; j<Jns.size(); j++){
					p=tmpL->jmap.find(Jns[j]);
					p_ddm=DatDofMode.find(Jns[j]);
					if(p!=tmpL->jmap.end() && p_ddm!=DatDofMode.end()){
						if(p->second==tmpL->TrootJ){
							xax.set(fields[accumnum].toDouble(), fields[accumnum+1].toDouble(), fields[accumnum+2].toDouble());
							accumnum += 4;
							yax.set(fields[accumnum].toDouble(), fields[accumnum+1].toDouble(), fields[accumnum+2].toDouble());
							accumnum += 4;
							zax.set(fields[accumnum].toDouble(), fields[accumnum+1].toDouble(), fields[accumnum+2].toDouble());
							accumnum += 4;
							cn.set(fields[accumnum].toDouble(), fields[accumnum+1].toDouble(), fields[accumnum+2].toDouble());
							(*(p->second->woTj)) = MakeTransformMat(&xax, &yax, &zax, &cn);
							accumnum += 4;
						}else{
							if(p_ddm->second==1){
								p->second->th=fields[accumnum].toDouble();
								(*(p->second->jiniTj)) = GetGeneRotMat(p->second->oj_jaxis.x, p->second->oj_jaxis.y, p->second->oj_jaxis.z, p->second->th);
								GetEulerAngleFromTransMat((*(p->second->jiniTj)), &(p->second->th_f), &(p->second->th_a), &(p->second->th_p));
								accumnum += 3;
							}else{
								p->second->th_f = fields[accumnum++].toDouble();
								p->second->th_a = fields[accumnum++].toDouble();
								p->second->th_p = fields[accumnum++].toDouble();
								(*(p->second->jiniTj)) = GetEulerRotMat(p->second->th_f, p->second->th_a, p->second->th_p);
								if(p->second->jDOF==1){
									Vector3d jaxis;
									EquivRotAxisAndAngle(p->second->jiniTj, &(jaxis.x), &(jaxis.y), &(jaxis.z), &(p->second->th));
									if(jaxis.dot(p->second->oj_jaxis)<0.0) p->second->th *= -1.0;
								}
							}
						}
					}
				}//End of for loop for(int j=0; j<Jns.size(); j++){

				//このあと，PALM1 TM0の位置が書かれているのでTM0６DOF対応なら読み込むべき
				//PALM1は無意味なので飛ばす
				accumnum += 3;
				//TM0の方だけ jiniTjのTranslation成分が入っている
				Vector3d newTM0(0,0,0);
				if(accumnum+2<fields.size()){
					//cout << "HoldJangleData accumnum=" << accumnum << endl;
					newTM0.set(fields[accumnum+0].toDouble(),fields[accumnum+1].toDouble(),fields[accumnum+2].toDouble());
				}else{
					//修正データが入っていなかったということなので，
					TM0_in_W= false;
				}
				//リンクの位置姿勢を計算しておく
				if(!TM0_in_W){
					SetVec3dtoMatCol(newTM0,(*(tmpL->GetTargJ("TM0")->jiniTj)),3);
				}
				tmpL->Calc_woTj_ByRotMatCopy(tmpL);
				if(TM0_in_W){
					//TM0の分だけ親指に修正をかける
					tmpL->MoveBaseOfF_withNewJCinW(0,newTM0);
				}

				Mlink2Armature(tmpL,arm,false);
				//c->pos1.recordCurrentPosture();
				c->pos1->recordCurrentPosture();
				c->setPropertyFromArmatureController(frameID);

				frameID+=1;
				//cout << " time when read " << p_fnmcd_j->second.jangdats[lsize-1]->time << endl;
			}

			f.close();
			seq->goToFrameAt(seq->firstFrame());
			dhApp::updateAllGLViews();
		}
	}
}


void dhMtool::ReadArmatureMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhArmature* arm, QString fileName)
//dhDlgMoCapCtrl::onBtnArmReadFileClicked(void)の内容
{
	if(!seq || fileName=="" ){
		if(!seq) DH_LOG("ReadArmatureMotionFileAndSetMoCapSequence: exit due to empty MoCapSequence",0);
		if(fileName=="") DH_LOG("ReadArmatureMotionFileAndSetMoCapSequence: exit due to empty filename",0);
	}else{
		dhMCArmatureController* c=GetExistingMCArmatureController(seq);
		if(c==NULL) c= seq->addArmatureController(arm);
		QFile f(fileName);
		if (f.open(QIODevice::ReadOnly | QIODevice::Text)){
			QTextStream ft(&f);
			int NFrames;  ft >> NFrames;

			c->mProps.clear();
			int Nr;   ft >> Nr;
	
			float x[3];

			for(int i=0;i<NFrames;i++){
				dhMCArmatureControllerProperty p(seq);

				ft >> p.frameID;

				ft >> x[0] >> x[1] >> x[2];
				p.t.setVec(x[0],x[1],x[2]);

				for(int k=0;k<Nr;k++){
					dhRotation r;
					ft >> x[0] >> x[1] >> x[2];
					r.setParam(x[0],x[1],x[2]);
					p.r.push_back(r);
				}
				c->mProps.insert(p.frameID, p);


			}
			f.close();
			seq->goToFrameAt(seq->firstFrame());
			dhApp::updateAllGLViews();
		}
	}
}

void dhMtool::ReadRigidBodyMotionFileAndSetMoCapSequence(dhMoCapSequence* seq, dhFeaturePoints* obj_fp, QString fileName)
//	dhDlgMoCapCtrl::onBtnRBReadFileClickedの内容
{
	if(!seq || fileName=="" ){
		if(!seq) DH_LOG("ReadRigidBodyMotionFileAndSetMoCapSequence: exit due to empty MoCapSequence",0);
		if(fileName=="") DH_LOG("ReadRigidBodyMotionFileAndSetMoCapSequence: exit due to empty filename",0);
	}else{

		dhMCRigidBodyController* c=GetExistingMCRigidBodyController(seq);
		if(c==NULL) c = seq->addRigidBodyController(obj_fp);
		QFile f(fileName);
		if (f.open(QIODevice::ReadOnly | QIODevice::Text)){
			QTextStream ft(&f);
			int NFrames;  ft >> NFrames;

			c->mProps.clear();

			for(int i=0;i<NFrames;i++){
				dhMCRigidBodyControllerProperty p(seq);
		
				ft >> p.frameID;

				for(int k=0;k<16;k++){
					ft >> p.trmat.p[k];	
				}
				c->mProps.insert(p.frameID, p);
			}
			f.close();
			seq->goToFrameAt(seq->firstFrame());
			dhApp::updateAllGLViews();

		}
	}
}

dhMoCapSequence* dhMtool::ConvertArmatureAndObjPostureInObjCF(dhMoCapSequence *seq, dhSkeletalSubspaceDeformation *ssd, dhMesh *obj, QString newMcpSeqName)
//すべてのフレームで計算がされていることが前提
//ConvertArmaturePostureInObjCFではArmatureControllerしか生成されなかったが，Objectの状態も記述する
{
    int beginFrame=seq->firstFrame();
    int endFrame=seq->lastFrame();
    dhArmature *arm = ssd->Armature();
//    dhBone *root=arm->bone("ROOT");
    QList<dhBone*> rootBs=arm->rootBones();
    dhBone *root=rootBs[0];


    dhMoCapSequence *newseq = dhnew <dhMoCapSequence>();
    newseq->SetName(newMcpSeqName);
    dhMCArmatureController* ctrl = newseq->addArmatureController(arm);
    dhMCRigidBodyController *rbc = newseq->addRigidBodyController(obj);

    newseq->openControlDock();
    dhMCArmatureController *org_actrl = GetExistingMCArmatureController(seq);
    dhMat44 II;
    QList<int> iList = org_actrl->mProps.keys();  //キーフレーム番号リストを取得

        for(int i=0;i<iList.size();i++){
            int k = iList[i];
            //DH_LOG(QString("frame %1").arg(i),0);
            //seq->goToFrameAt(i);
            seq->goToFrameAt(org_actrl->mProps[k].frameID);
#if 1
            dhMat44 oM = obj->M();
            //Matrix myoM(4,4); myoM = DHMat2Mat(oM);
            //myoM.printcomp("myoM");
            dhMat44 invM = oM.getInverse();
            //obj->SetM(invM*obj->M());
            root->R = root->Twj0.inverse()*invM*root->Twj;
            //root->R = root->Twj0.inverse()*invM*root->Twj0*root->R;
            arm->updateFrames();
#else
            dhMtool::ArmaturePostureInObjCF(arm,obj);
#endif
            //ctrl->pos1.recordCurrentPosture();
            ctrl->pos1->recordCurrentPosture();
            //ctrl->setPropertyFromArmatureController(i);
            ctrl->setPropertyFromArmatureController(org_actrl->mProps[k].frameID);

            dhMCRigidBodyControllerProperty p(newseq);
            p.frameID=org_actrl->mProps[k].frameID;
            p.trmat=II;
            rbc->mProps.insert(p.frameID, p);

        }
    dhApp::updateAllWindows();
    return newseq;
}








dhMoCapSequence* dhMtool::ConvertArmaturePostureInObjCF(dhMoCapSequence *seq, dhSkeletalSubspaceDeformation *ssd, dhMesh *obj, QString newMcpSeqName)
//すべてのフレームで計算がされていることが前提
{
    if(!seq) return NULL;
    if(!ssd) return NULL;
    if(!obj) return NULL;
    int beginFrame=seq->firstFrame();
	int endFrame=seq->lastFrame();
	dhArmature *arm = ssd->Armature();
//	dhBone *root=arm->bone("ROOT");
    QList<dhBone*> rootBs=arm->rootBones();
    dhBone *root=rootBs[0];

	dhMoCapSequence *newseq = dhnew <dhMoCapSequence>();
	newseq->SetName(newMcpSeqName);
	dhMCArmatureController* ctrl = newseq->addArmatureController(arm);
	newseq->openControlDock();
	dhMCArmatureController *org_actrl = GetExistingMCArmatureController(seq);
	QList<int> iList = org_actrl->mProps.keys();  //キーフレーム番号リストを取得

    for(int k=beginFrame; k<endFrame+1; k++){
    //		for(int i=0;i<iList.size();i++){
//			int k = iList[i];
			//DH_LOG(QString("frame %1").arg(i),0);
			//seq->goToFrameAt(i);
            seq->goToFrameAt(k);
//			seq->goToFrameAt(org_actrl->mProps[k].frameID);
#if 1
			dhMat44 oM = obj->M();
			//Matrix myoM(4,4); myoM = DHMat2Mat(oM);
			//myoM.printcomp("myoM");
			dhMat44 invM = oM.getInverse();
			//obj->SetM(invM*obj->M());
			root->R = root->Twj0.inverse()*invM*root->Twj;
			//root->R = root->Twj0.inverse()*invM*root->Twj0*root->R;
			arm->updateFrames();
#else
			dhMtool::ArmaturePostureInObjCF(arm,obj);
#endif
			ctrl->pos1->recordCurrentPosture();
//			ctrl->setPropertyFromArmatureController(org_actrl->mProps[k].frameID);
            ctrl->setPropertyFromArmatureController(k);

		}
	dhApp::updateAllWindows();
	return newseq;
}
void dhMtool::ArmaturePostureInObjCF(dhArmature *arm, dhMesh *obj, bool TransObj)
{
	dhBone *root=arm->bone("ROOT");
	dhMat44 trM = obj->M().inverse();
	dhMat44 newR = root->Twj0.inverse() * trM*root->Twj;
	root->R=newR;
	arm->Update();
	if(TransObj) obj->SetM(dhMat44());
	//dhApp::updateAllWindows();
}
QString dhMtool::ArmaturePostureInObjCFAsString(dhMoCapSequence *seq)
{
	dhMCArmatureController *ac=GetExistingMCArmatureController(seq);
	dhBone *root=ac->arm->bone("ROOT");
	dhMCRigidBodyController *rc=GetExistingMCRigidBodyController(seq);
	dhMat44 trM = rc->m->M().inverse();
	dhMat44 newR = root->Twj0.inverse() * trM*root->Twj;
	root->R=newR;
	ac->arm->Update();
	QString rotString=ArmaturePostureAsString(ac->arm);
	return rotString;
}

QString dhMtool::ArmaturePostureAsString(dhArmature *arm)
{
	QString posS;
	for (int i=0, bnum=arm->NBones(); i<bnum; i++){
		if(i>0) posS.append(",");
		posS.append(arm->bone(i)->R.toString());
	}
	return posS;
}

void dhMtool::ArmaturePostureFromGivenString(dhArmature *arm,QString posS)
{
	QStringList matStrs = posS.split(",");
	dhMat44 R;
	for (int i=0, bnum=arm->NBones(); i<bnum; i++){
		R.fromString(matStrs[i]);
		arm->bone(i)->R=R;
	}
	arm->Update();
}
QString dhMtool::MarkerNamesAsString(dhMoCapSequence *seq)
{
	QString mnameString;
	dhMCMarkerController *mc=GetExistingMCMarkerController(seq);
	dhMCMarkerControllerProperty pm=mc->mProps[0];
	QList<QString> keys = pm.item.keys();
	for(int i=0; i<keys.size(); i++){
		if(i>0) mnameString.append(",");
		mnameString.append(keys[i]);
	}
	return mnameString;

}

QString dhMtool::MarkerPosInObjCFAsString(dhMoCapSequence *seq)
	//[m0.x,m0.y,m0.z]/
{
	dhMCMarkerController *mc=GetExistingMCMarkerController(seq);
	dhMCRigidBodyController *rc=GetExistingMCRigidBodyController(seq);
	dhMat44 trM = rc->m->M().inverse();
	QString mposString;

	//dhFeaturePoint fp;
	int frameID = seq->currentFrameID();
	dhMCMarkerControllerProperty pm=mc->mProps[frameID];
	QMap<QString,dhMath::dhVec4>::iterator p;
	dhMath::dhVec4 tmpV,trV;
	for(p=pm.item.begin(); p!=pm.item.end(); p++){
		tmpV=p.value();
		trV = trM*tmpV;
		if(p!=pm.item.begin()) mposString.append("/");
		mposString.append(QString("[%1,%2,%3]").arg(trV(0)).arg(trV(1)).arg(trV(2)));
	}

	return mposString;
}

dhMoCapSequence* dhMtool::MoCapSeqWRTDifferentReference(dhMoCapSequence *seq, dhArmature *newrefarm)
//seqに計算された結果を，newrefarmからの角度としてnewseqに保存する
//newrefarmの関節座標系とseqのものが，定義は同じで角度が違うだけの場合はこれでOK
{

	int beginFrame=seq->firstFrame();
	int endFrame=seq->lastFrame();
	dhMCArmatureController *org_actrl = GetExistingMCArmatureController(seq);

	dhMoCapSequence *newseq = dhnew <dhMoCapSequence>();
	dhMCArmatureController* new_actrl = newseq->addArmatureController(newrefarm);
	newseq->openControlDock();

	for(int i=beginFrame; i<endFrame+1; i++){
		seq->goToFrameAt(i);
		newrefarm->resetPosture();
		newrefarm->fitToTargetArmature(org_actrl->arm);

		//new_actrl->pos1.recordCurrentPosture();
		new_actrl->pos1->recordCurrentPosture();
		new_actrl->setPropertyFromArmatureController(i);

	}

	dhApp::updateAllWindows();
	return newseq;
}

dhMCControllerBase* dhMtool::GetGivenNameController(dhMoCapSequence *seq, QString cName)
{
    for(int i=0; i<seq->controllerCount(); i++){
        if(seq->ctrl[i]->ControllerTitle()==cName)
            return seq->ctrl[i];
    }
    return NULL;
}

dhMCArmatureController* dhMtool::GetExistingMCArmatureController(dhMoCapSequence *seq)
{
	dhMCArmatureController *actrl=NULL;	//すでに存在しなければNULLが返る
	for(int i=0; i<seq->ctrl.size(); i++) {
		actrl=dynamic_cast<dhMCArmatureController*>(seq->ctrl[i]);//element_cast<dhMCArmatureController*>(seq->ctrl[i]);
		if(actrl!=NULL) break;
	}
	//DH_LOG(QString("mProps.size=%1").arg(actrl->mProps.size()),0);
	return actrl;
	
}
dhMCRigidBodyController* dhMtool::GetExistingMCRigidBodyController(dhMoCapSequence *seq)
{
	dhMCRigidBodyController *rctrl=NULL;
	for(int i=0; i<seq->ctrl.size(); i++) {
		rctrl=dynamic_cast<dhMCRigidBodyController*>(seq->ctrl[i]);//element_cast<dhMCRigidBodyController*>(seq->ctrl[i]);
		if(rctrl!=NULL) break;
	}
	return rctrl;
	
}
dhMCMarkerController* dhMtool::GetExistingMCMarkerController(dhMoCapSequence *seq)
{
	dhMCMarkerController *ctrl=NULL;
	for(int i=0; i<seq->ctrl.size(); i++) {
		ctrl=dynamic_cast<dhMCMarkerController*>(seq->ctrl[i]);//element_cast<dhMCMarkerController*>(seq->ctrl[i]);
		if(ctrl!=NULL) break;
	}
	return ctrl;
}

void dhMtool::saveArmatureAsMyLink(dhArmature *arm, QString fileName)
{
	myLink *tmpL = BuildmyLinkFromArm(arm,myLink::LMType::LM3);
	tmpL->saveCFdat3(fileName);
}



void dhMtool::saveMoCapSequenceArmatureData(dhMoCapSequence *seq, QString fileName)
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	if(c!=NULL && fileName!=""){
		QFile f(fileName);
		if (f.open(QIODevice::WriteOnly | QIODevice::Text)){
			if(c->mProps.size()>0){
				QTextStream ft(&f);

				QString qtab("\t");
				ft << c->mProps.size()<<qtab<< c->mProps.begin()->r.size() << QString("\n");  // ここで1行目を出力
 
				QList<int> iList = c->mProps.keys();  //キーフレーム番号リストを取得
				for(int i=0;i<iList.size();i++){
					int k = iList[i];
					ft << c->mProps[k].frameID << qtab;        //各行の1項目めはフレーム番号
					dhVec4& v = c->mProps[k].t;
					ft << v(X) <<qtab<< v(Y) <<qtab<< v(Z);//次がルートボーンの平行移動量
					for(int j=0;j<c->mProps[k].r.size();j++){
						c->mProps[k].r[j].convertToRPY();    //つづいて各ボーンのRPY回転角がならぶ
						ft <<qtab<< c->mProps[k].r[j](0) <<qtab<< c->mProps[k].r[j](1) <<qtab<< c->mProps[k].r[j](2);
					}
					ft  << QString("\n");
				}
			}
			f.close();
		}
	}
}

void dhMtool::RegisterCurrentFPsAsKeyFrame(dhMoCapSequence *seq, dhFeaturePoints *refFPs, dhMesh *refM, QString CtrlrName, int frameID)
{
    dhMCControllerBase *c=this->GetGivenNameController(seq,CtrlrName);
    dhMCMarkerController *mc;
    if(c==NULL){
        mc=seq->addMarkerController();
        mc->SetControllerTitle(CtrlrName);
    }else
        mc = dynamic_cast<dhMCMarkerController*>(c);
    dhMCMarkerControllerProperty pm(seq);
    pm.frameID=frameID;
    dhMath::dhMat44 meshM;
    meshM.initMat();
    if(refM==NULL)
        DH_LOG("RegisterCurrentFPsAsKeyFrame:  Mesh does not exist",0);
    for(int i=0; i<refFPs->pointCount(); i++){
        if(refM!=NULL) meshM=refM->M();

        pm.item.insert(refFPs->point(i)->pointName(),meshM*(refFPs->point(i)->position()));
    }
    mc->mProps.insert(pm.frameID, pm);
    mc->updateMarkerNameCache();

}

void dhMtool::RegisterCurrentWrldFPsAsKeyFrame(dhMoCapSequence *seq, dhFeaturePoints *refFPs, QString CtrlrName, int frameID)
{
    dhMCControllerBase *c=this->GetGivenNameController(seq,CtrlrName);
    dhMCMarkerController *mc;
    if(c==NULL){
        mc=seq->addMarkerController();
        mc->SetControllerTitle(CtrlrName);
    }else
        mc = dynamic_cast<dhMCMarkerController*>(c);
    dhMCMarkerControllerProperty pm(seq);
    pm.frameID=frameID;
    for(int i=0; i<refFPs->pointCount(); i++){
        pm.item.insert(refFPs->point(i)->pointName(),refFPs->point(i)->position());
    }
    mc->mProps.insert(pm.frameID, pm);
    mc->updateMarkerNameCache();

}


void dhMtool::RegisterCurrentMeshAsKeyFrame(dhMoCapSequence *seq, dhMesh *refM, QString CtrlrName, int frameID)
{
    dhMCControllerBase *c=this->GetGivenNameController(seq,CtrlrName);
    dhMCRigidBodyController *rc;
    if(c==NULL){
        rc=seq->addRigidBodyController(refM);
        rc->SetControllerTitle(CtrlrName);
    }else
        rc = dynamic_cast<dhMCRigidBodyController*>(c);

    dhMCRigidBodyControllerProperty p(seq);
    p.frameID=frameID;
    p.trmat=refM->M();
    //for(int k=0;k<16;k++){
    //	ft >> p.trmat.p[k];
    //}
    rc->mProps.insert(frameID, p);

}


int dhMtool::AddCurrentAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA)
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	if(c==NULL) c = seq->addArmatureController(refA);

	int frameID=c->mProps.size();
	//c->pos1.recordCurrentPosture();
	c->pos1->recordCurrentPosture();
	c->setPropertyFromArmatureController(frameID);
	dhApp::updateAllWindows();
	return frameID;
}


int dhMtool::AddCurrentArmMeshAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM)
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	dhMCRigidBodyController *rbc=GetExistingMCRigidBodyController(seq);
	
	if(c==NULL) c = seq->addArmatureController(refA);
    if(rbc==NULL) rbc = seq->addRigidBodyController();

	int frameID=c->mProps.size();
	//c->pos1.recordCurrentPosture();
	c->pos1->recordCurrentPosture();
	c->setPropertyFromArmatureController(frameID);


	dhMCRigidBodyControllerProperty p(seq);
	p.frameID=frameID;
	p.trmat=refM->M();
	//for(int k=0;k<16;k++){
	//	ft >> p.trmat.p[k];	
	//}
	rbc->mProps.insert(p.frameID, p);
	dhApp::updateAllWindows();
	return frameID;
}

int dhMtool::AddCurrentArmMeshFPsAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhMesh *refM, dhFeaturePoints *refFPs)
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	dhMCRigidBodyController *rbc=GetExistingMCRigidBodyController(seq);
	dhMCMarkerController *mc=GetExistingMCMarkerController(seq);
	
	if(c==NULL) c = seq->addArmatureController(refA);
	if(rbc==NULL) rbc = seq->addRigidBodyController();
	if(mc==NULL) mc = seq->addMarkerController();
	

	int frameID=c->mProps.size();
	//c->pos1.recordCurrentPosture();
	c->pos1->recordCurrentPosture();
	c->setPropertyFromArmatureController(frameID);


	dhMCRigidBodyControllerProperty p(seq);
	p.frameID=frameID;
	p.trmat=refM->M();
	rbc->mProps.insert(p.frameID, p);
	

	dhMCMarkerControllerProperty pm(seq);
	pm.frameID=frameID;
	
	//dhFeaturePoint fp;
	for(int i=0; i<refFPs->pointCount(); i++){
		pm.item.insert(refFPs->point(i)->pointName(),refFPs->point(i)->position());
	}
	mc->mProps.insert(pm.frameID, pm);
	mc->updateMarkerNameCache();
	dhApp::updateAllWindows();
	return frameID;
}
int dhMtool::AddCurrentArmFPsAsKeyFrame(dhMoCapSequence *seq, dhArmature *refA, dhFeaturePoints *refFPs)
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	dhMCMarkerController *mc=GetExistingMCMarkerController(seq);
	
	if(c==NULL) c = seq->addArmatureController(refA);
	if(mc==NULL) mc = seq->addMarkerController();
	

	int frameID=c->mProps.size();
	//c->pos1.recordCurrentPosture();
	c->pos1->recordCurrentPosture();
	c->setPropertyFromArmatureController(frameID);



	dhMCMarkerControllerProperty pm(seq);
	pm.frameID=frameID;
	
	//dhFeaturePoint fp;
	for(int i=0; i<refFPs->pointCount(); i++){
		pm.item.insert(refFPs->point(i)->pointName(),refFPs->point(i)->position());
	}
	mc->mProps.insert(pm.frameID, pm);
	mc->updateMarkerNameCache();
	dhApp::updateAllWindows();
	return frameID;
}


void dhMtool::CalcRigidBodyMotionConsideringMarkExistence(dhMoCapSequence *seq, dhMesh *refM, dhFeaturePoints *refFPs)
{
    dhMCMarkerController *mc=GetExistingMCMarkerController(seq);
    QList<dhVec4> srcPs, dstPs;

    dhMCRigidBodyController *rc=seq->addRigidBodyController(refM);
    //FeaturePoint設定
    rc->setLandmarkFittingDesc(refFPs);
    QMap<QString,dhMath::dhVec4>::iterator p_sv;
    int beginFrame=seq->firstFrame();
    int endFrame=seq->lastFrame();
    for(int frameID=beginFrame; frameID<endFrame+1; frameID++){

        dhMCMarkerControllerProperty mp = mc->Prop(frameID);
        for(int i=0; i<rc->fpts->pointCount(); i++){
            QString pname=rc->fpts->point(i)->pointName();
            if(mp.item.contains(pname) && refFPs->point(pname)->isEnabled()){
                srcPs.append(refFPs->point(pname)->position());
                p_sv=mp.item.find(pname);
                dstPs.append(p_sv.value());
            }
        }
        if(srcPs.size()>=3){
            dhMat44 Tr=TransMatToRegistratePointsToPoints(srcPs,dstPs);
            //dhMat44 Tr=TransMat2RegistratePoints(srcPs,dstPs);
            dhMCRigidBodyControllerProperty rcp(seq);
            rcp.frameID=frameID;
            rcp.trmat=Tr;
            rc->mProps.insert(frameID, rcp);
        }
        if(srcPs.size()>0) srcPs.clear();
        if(dstPs.size()>0) dstPs.clear();
    }
    seq->updateEditDockUI();
    dhApp::updateAllWindows();
}


std::vector<Matrix*> dhMtool::CalCFDefDif(myLink* fL, myLink* tL)
{
	std::vector<Matrix*> defdif;
	myJoint* tJ;

	std::map<QString,Matrix*>::iterator p;
	for(int j=0; j<(int)fL->jlist.size(); j++){
		tJ = tL->GetTargJ(fL->jlist[j]->jmname);
		Matrix *tmpT = new Matrix(4,4);
		tmpT->SetIdentityMatrix();
		if(fL->jlist[j]!=fL->TrootJ){
			(*(tmpT)) = (*(fL->jlist[j]->woTj)).InverseMatrix()*(*(tJ->woTj));
			tmpT->ZeroTransFac();
		}
		
		defdif.push_back(tmpT);

	}
	return defdif;

}


void dhMtool::saveMoCapSequenceArmatureDataAsMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName, dhArmature *refArm)
//refarmが，MoCapでの個別リンクと同じリンク構造定義の場合はこれを利用する
//異なる場合は　saveMoCapSequenceArmatureDataAsDifferentCFMylink　を利用する
//refArmがNULLの場合はseqに存在するArmatureの角度をそのまま出す
//refArmに別のものが指定されている場合は，指定されたりんくを基準とする角度を出力する
//JaxfName=="NONE" or "" ならEuler角出力のみ
//JaxfNameに何か指定されていたら　元のfileName-DOF.csvとしても保存しておく
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	if(c!=NULL && fileName!=""){
		if(c->mProps.size()>0){
			//角度計算用に基準Armatureを作る
			dhArmature *arm0;
			if(refArm==NULL){
				arm0 = dhnew <dhArmature>();
				arm0->copyFrom(c->arm);
			}else arm0=refArm;
			arm0->resetPosture();

			myLink *bsL = BuildmyLinkFromArm(arm0,myLink::LMType::LM3);
			bsL->SetRefLink(bsL);

			QString fileNameDOF;
			//DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  JaxfName="+JaxfName,0);
			if(JaxfName!="NONE" && JaxfName!=""){
				fileNameDOF=QFileInfo(fileName).absolutePath()+"/"+QFileInfo(fileName).baseName()+"-DOF.csv";
				DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  fileNameDOF="+fileNameDOF,0);
				bsL->readJaxis(JaxfName);
			}
#if 1
			for(int i=seq->firstFrame();i<seq->lastFrame();i++){
				seq->goToFrameAt(i);
				bsL->frameID=i;
				bsL->ResetPosture();
				Arm2myL(c->arm,bsL);
				if(i==seq->firstFrame()) bsL->saveJangle(fileName);
				else bsL->saveJangleByAddition(fileName);
				if(JaxfName!="NONE" && JaxfName!=""){
					if(i==seq->firstFrame()) bsL->saveJangle_DOF(fileNameDOF,QIODevice::WriteOnly);
					else bsL->saveJangle_DOF(fileNameDOF,QIODevice::Append);
				}

			}
#else
			QList<int> iList = c->mProps.keys();  //キーフレーム番号リストを取得
			for(int i=0;i<iList.size();i++){
				int k = iList[i];
				seq->goToFrameAt(c->mProps[k].frameID);
				bsL->frameID=c->mProps[k].frameID;
				bsL->ResetPosture();
				Arm2myL(c->arm,bsL);
				
				if(i==0) bsL->saveJangle(fileName);
				else bsL->saveJangleByAddition(fileName);
				if(JaxfName!="NONE" && JaxfName!=""){
					if(i==0) bsL->saveJangle_DOF(fileNameDOF,QIODevice::WriteOnly);
					else bsL->saveJangle_DOF(fileNameDOF,QIODevice::Append);
				}
			}
#endif
		}
	}
}
void dhMtool::saveArmatureAsMylinkCF(dhArmature *arm, QString OFName)
{
	if(OFName=="NONE"){
		OFName = QFileDialog::getSaveFileName(0,"Input CF file name","","csv (*.csv)");
	}
	myLink *bsL = BuildmyLinkFromArm(arm,myLink::LMType::LM3);
	if(OFName!="NONE" && !OFName.isEmpty())	bsL->saveCFdat3(OFName);
}

dhArmature* dhMtool::CFDefRepairedArmature(dhArmature *arm, bool RightHand)
{
	myLink *refL = BuildmyLinkFromArm(arm,myLink::LMType::LM3);
	refL->setRightHand(RightHand);

	Matrix newCF(4,4), RotM(4,4);
	newCF.SetIdentityMatrix();
	Vector3d xax, yax, yax2, zax, tmpzax;

	//まずIM
	yax = refL->GetTargJ("MD1")->w_jc - refL->GetWristJ()->w_jc;
	yax.normalize();
	yax2 = refL->GetTargJ("ID1")->w_jc - refL->GetWristJ()->w_jc;
	yax2.normalize();
	
	if(refL->righthand) zax.cross(yax,yax2);
	else zax.cross(yax2,yax);
	zax.normalize();
	xax.cross(yax,zax);
	newCF=MakeTransformMat(&xax, &yax, &zax, &(refL->GetWristJ()->w_jc));
	(*(refL->GetWristJ()->woTj)) = newCF;


	//TrootJも
	tmpzax 	= zax;
	yax = refL->GetWristJ()->w_jc - refL->TrootJ->w_jc; yax.normalize();
	xax.cross(yax,tmpzax);
	zax.cross(xax,yax);
	newCF=MakeTransformMat(&xax, &yax, &zax, &(refL->TrootJ->w_jc));
	(*(refL->TrootJ->woTj)) = newCF;


	//次にRP
	yax = refL->GetTargJ("RN1")->w_jc - refL->GetWristJ()->w_jc;
	yax.normalize();
	yax2 = refL->GetTargJ("PN1")->w_jc - refL->GetWristJ()->w_jc;
	yax2.normalize();
	if(refL->righthand) zax.cross(yax2,yax);
	else zax.cross(yax,yax2);
	zax.normalize();
	xax.cross(yax,zax);
	newCF=MakeTransformMat(&xax, &yax, &zax, &(refL->GetWristJ()->w_jc));
	(*(refL->GetTargJ("RP")->woTj)) = newCF;



	myJoint *trgJ, *ztrgJ;
	for(int fID=0; fID<5; fID++){
		trgJ=refL->fing[fID].StartJ;
		while(trgJ!=NULL){
			if(fID==0){
				ztrgJ = refL->fing[fID].StartJ->childJ;
			}else{
				ztrgJ=trgJ->rootJ;
			}
			//if(trgJ==refL->fing[fID].StartJ || trgJ==){
			//	if(fID==0) ztrgJ=trgJ->childJ;//thumb根元のとき
			//	else ztrgJ=trgJ->rootJ;
			//}else if(trgJ->childJ==NULL) ztrgJ=trgJ->rootJ;//末端のとき
			//else ztrgJ=trgJ;
			SetMatColtoVec3d((*(ztrgJ->woTj)),2,tmpzax);
			yax = (trgJ->childJ!=NULL) ? trgJ->childJ->w_jc - trgJ->w_jc : trgJ->w_tipP - trgJ->w_jc;
			yax.normalize();
			xax.cross(yax,tmpzax); xax.normalize();
			zax.cross(xax,yax); xax.normalize();
			newCF=MakeTransformMat(&xax, &yax, &zax, &(trgJ->w_jc));
			(*(trgJ->woTj)) = newCF;
			trgJ=trgJ->childJ;
		}
	}

	refL->Calc_rjTj();
	refL->Restore_w_jc();
	refL->CalcTipPinCFj();

	QString OFName = "tmpCF.csv";
	refL->saveCFdat3(OFName);
	dhArmature *arm1=dhnew<dhArmature>();
	arm1->importDhaibaHandCSVFile(OFName);
	QFile::remove(OFName);

	//for(int i=0; i<refL->jlist.size(); i++){
	//	for(int k=0; k<refL->jlist[i]->armbnames.size(); k++){
	//		dhBone* b=arm->bone(refL->jlist[i]->armbnames[k]);
	//		//並進成分含めてセットする必要あり
	//		//b->R=最初の位置からの並進（）;
	//		if(b){
	//			b->Twj0 = b->Twj = Mat2DHMat((*(refL->jlist[i]->woTj)));
	//		}
	//	}
	//}
	return arm1;
}

void dhMtool::superimposeLinks(myLink *fL, myLink *gL)
//woTjしか基本的には変更しない
{
	Matrix newCF(4,4), RotM(4,4);
	newCF.SetIdentityMatrix();
	Vector3d xax, yax, yax2, zax;
	
	yax = gL->GetTargJ("MD1")->w_jc - gL->GetWristJ()->w_jc;
	yax.normalize();
	
	yax2 = gL->GetTargJ("ID1")->w_jc - gL->GetWristJ()->w_jc;
	yax2.normalize();
	
	if(fL->righthand) zax.cross(yax,yax2);
	else zax.cross(yax2,yax);
	zax.normalize();
	xax.cross(yax,zax);

	newCF=MakeTransformMat(&xax, &yax, &zax, &(gL->GetWristJ()->w_jc));
	newCF.printcomp("newCF");
	//fLのIMを動かす
	fL->TransWrist(newCF);

	//fLのRPを動かす
	myJoint *trgJ,*glJ;
	trgJ=fL->GetTargJ("RP");
	glJ=gL->GetTargJ("RP");
	std::vector<Vector3d> srcPs, dstPs;
	srcPs.push_back(trgJ->w_jc);
	srcPs.push_back(fL->GetTargJ("RN1")->w_jc);
	srcPs.push_back(fL->GetTargJ("PN1")->w_jc);
	dstPs.push_back(glJ->w_jc);
	dstPs.push_back(gL->GetTargJ("RN1")->w_jc);
	dstPs.push_back(gL->GetTargJ("PN1")->w_jc);
	Matrix Tr(4,4), jiniTj(4,4);
	double th[3];
	Tr = TransMatToRegistratePointsToPoints(srcPs,dstPs);
	Tr.ZeroTransFac();
	newCF = Tr*(*(trgJ->woTj));
	jiniTj = (*(trgJ->woTj)).InverseMatrix()*newCF;
	GetEulerAngleFromTransMat(jiniTj,&th[0],&th[1],&th[2]);
	fL->MoveTargJ("RP",th);

	for(int fID=0; fID<5; fID++){
		trgJ=fL->fing[fID].StartJ;
		glJ=gL->fing[fID].StartJ;
		while(trgJ!=NULL){
			yax=(trgJ->childJ!=NULL) ? trgJ->childJ->w_jc - trgJ->w_jc: trgJ->w_tipP - trgJ->w_jc;
			yax2=(glJ->childJ!=NULL) ? glJ->childJ->w_jc - glJ->w_jc: glJ->w_tipP - glJ->w_jc;
			Tr = RotMat44VecToVec(yax,yax2);
			newCF = Tr*(*(trgJ->woTj));
			jiniTj = (*(trgJ->woTj)).InverseMatrix()*newCF;
			GetEulerAngleFromTransMat(jiniTj,&th[0],&th[1],&th[2]);
			fL->MoveTargJ(trgJ->jmname,th);
			trgJ = trgJ->childJ;
			glJ = glJ->childJ;
		}
	}
}



void dhMtool::saveMoCapSequenceArmatureDataAsDifferentCFMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName, dhArmature *refArm, bool Lefty)
//refArmがNULLの場合はseqに存在するArmatureの角度をそのまま出す
//refArmに別のものが指定されている場合は，指定されたりんくを基準とする角度を出力する
//JaxfName=="NONE" or "" ならEuler角出力のみ
//JaxfNameに何か指定されていたら　元のfileName-DOF.csvとしても保存しておく
//refarmが計測したものと同じ定義になっていれば，角度としては問題がないが，そうでない場合は最初の状態に近づけたものを
{
	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
	if(c!=NULL && fileName!=""){
		if(c->mProps.size()>0){
			//角度計算用に基準Armatureを作る
			dhArmature *mcarm = dhnew<dhArmature>();
			mcarm->copyFrom(c->arm);
			mcarm->resetPosture();

			myLink *mcL = BuildmyLinkFromArm(mcarm,myLink::LMType::LM3);
			myLink *refL = BuildmyLinkFromArm(refArm,myLink::LMType::LM3);

			refL->SetRefLink(refL);
			mcL->SetRefLink(refL);
			if(Lefty){
				refL->setRightHand(false); 
				mcL->setRightHand(false);
			}
			QString fileNameDOF;
			//DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  JaxfName="+JaxfName,0);
			if(JaxfName!="NONE" && JaxfName!=""){
				fileNameDOF=QFileInfo(fileName).absolutePath()+"/"+QFileInfo(fileName).baseName()+"-DOF.csv";
				DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  fileNameDOF="+fileNameDOF,0);
				refL->readJaxis(JaxfName);
				mcL->readJaxis(JaxfName);
			}
			
			//refLをmcLに重なるようにする
			superimposeLinks(refL, mcL);
			//mcL->saveCFdat3(QFileInfo(fileName).absolutePath()+"/CFtestMC.csv");
			//refL->saveCFdat3(QFileInfo(fileName).absolutePath()+"/CFtest.csv");

			////この状態で，CF定義の違いを覚えておく
			//std::vector<Matrix*> difCF=CalCFDefDif(mcL,refL);
			//refL->ResetPosture();

			QList<int> iList = c->mProps.keys();  //キーフレーム番号リストを取得
			for(int i=0;i<iList.size();i++){
				int k = iList[i];
				seq->goToFrameAt(c->mProps[k].frameID);
				mcL->frameID=c->mProps[k].frameID;
				mcL->ResetPosture();
				Arm2myL(c->arm,mcL);


				mcL->CalcAngle_a();
				if(i==0) mcL->saveJangle(fileName);
				else mcL->saveJangleByAddition(fileName);
				if(JaxfName!="NONE" && JaxfName!=""){
					if(i==0) mcL->saveJangle_DOF(fileNameDOF,QIODevice::WriteOnly);
					else mcL->saveJangle_DOF(fileNameDOF,QIODevice::Append);
				}
			}
		}
	}
}

//void dhMtool::saveMoCapSequenceArmatureDataAsDifferentCFMylink(dhMoCapSequence *seq, QString fileName, QString JaxfName, dhArmature *refArm, bool Lefty)
////refArmがNULLの場合はseqに存在するArmatureの角度をそのまま出す
////refArmに別のものが指定されている場合は，指定されたりんくを基準とする角度を出力する
////JaxfName=="NONE" or "" ならEuler角出力のみ
////JaxfNameに何か指定されていたら　元のfileName-DOF.csvとしても保存しておく
////refarmが計測したものと同じ定義になっていれば，角度としては問題がないが，そうでない場合は最初の状態に近づけたものを
//{
//	dhMCArmatureController *c=GetExistingMCArmatureController(seq);
//	if(c!=NULL && fileName!=""){
//		if(c->mProps.size()>0){
//			//角度計算用に基準Armatureを作る
//			dhArmature *mcarm = new dhArmature;
//			mcarm->copyFrom(c->arm);
//			mcarm->resetPosture();
//
//			myLink *mcL = BuildmyLinkFromArm(mcarm,myLink::LMType::LM3);
//			myLink *refL = BuildmyLinkFromArm(refArm,myLink::LMType::LM3);
//
//			refL->SetRefLink(refL);
//			mcL->SetRefLink(refL);
//			if(Lefty){
//				refL->setRightHand(false); 
//				mcL->setRightHand(false);
//			}
//			QString fileNameDOF;
//			//DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  JaxfName="+JaxfName,0);
//			if(JaxfName!="NONE" && JaxfName!=""){
//				fileNameDOF=QFileInfo(fileName).absolutePath()+"/"+QFileInfo(fileName).baseName()+"-DOF.csv";
//				DH_LOG("saveMoCapSequenceArmatureDataAsMylink:  fileNameDOF="+fileNameDOF,0);
//				refL->readJaxis(JaxfName);
//				mcL->readJaxis(JaxfName);
//			}
//			
//			//refLをmcLに重なるようにする
//			superimposeLinks(refL, mcL);
//			//mcL->saveCFdat3(QFileInfo(fileName).absolutePath()+"/CFtestMC.csv");
//			//refL->saveCFdat3(QFileInfo(fileName).absolutePath()+"/CFtest.csv");
//
//			//この状態で，CF定義の違いを覚えておく
//			std::vector<Matrix*> difCF=CalCFDefDif(mcL,refL);
//			refL->ResetPosture();
//
//			QList<int> iList = c->mProps.keys();  //キーフレーム番号リストを取得
//			for(int i=0;i<iList.size();i++){
//				int k = iList[i];
//				seq->goToFrameAt(c->mProps[k].frameID);
//				mcL->frameID=c->mProps[k].frameID;
//				mcL->ResetPosture();
//				Arm2myL(c->arm,mcL);
//
//				for(int j=0; j<mcL->jlist.size(); j++){
//					(*(mcL->jlist[j]->woTj)) = (*(mcL->jlist[j]->woTj))*(*(difCF[j]));
//				}
//				mcL->CalcAngle_a();
//				if(i==0) mcL->saveJangle(fileName);
//				else mcL->saveJangleByAddition(fileName);
//				if(JaxfName!="NONE" && JaxfName!=""){
//					if(i==0) mcL->saveJangle_DOF(fileNameDOF,QIODevice::WriteOnly);
//					else mcL->saveJangle_DOF(fileNameDOF,QIODevice::Append);
//				}
//			}
//		}
//	}
//}
void dhMtool::saveMoCapSequenceRigidBodyMotionData(dhMoCapSequence *seq, QString fileName)
{
	if(fileName!=""){
		QFile f(fileName);
		if (f.open(QIODevice::WriteOnly | QIODevice::Text)){
			QTextStream ft(&f);

			QString qtab("\t");
			dhMCRigidBodyController* c=GetExistingMCRigidBodyController(seq);
			ft << c->mProps.size()<< QString("\n");  // ここで1行目を出力
			QList<int> iList = c->mProps.keys();  //キーフレーム番号リストを取得
			for(int i=0;i<iList.size();i++){
				int k=iList[i];
				ft << c->mProps[k].frameID;        //各行の1項目めはフレーム番号
		//
				c->seq->goToFrameAt(c->mProps[k].frameID);
				dhMat44 objCF = c->m->M();
				for(int j=0;j<4;j++){
					for(int f=0;f<4;f++){
						ft <<qtab<< objCF(f,j);
					}
				}
				ft  << QString("\n");
			}
			f.close();
		}	
	}
}

void dhMtool::AppendArmatureOrRigidBodyMotionFiles(QString fnamesInCsvString, QString OFname)
	//QList<QString>で渡せなかったので，カンマ区切りのStringにして渡す
{
	//
	if(OFname=="") return;
	QTextStream ef_in;
	QString textLine;
	QStringList txtLs;
	QStringList fields;
	int frameNum=0;
	int accumframeNum=0;
	int boneNum;
	QStringList fnames = fnamesInCsvString.split(",");
	bool IsArmatureData=false;

	QFile file_e;
	
	for(int i=0; i<fnames.size();i++){
		file_e.setFileName(fnames[i]);
		if(!file_e.open(QIODevice::ReadOnly)){
			DH_LOG("AppendFiles: Cannot open file "+ fnames[i],0);
		}else{
			ef_in.setDevice(&file_e);
			textLine = ef_in.readLine();
			fields = textLine.split("\t");
			frameNum += fields[0].toInt();

			if(fields.size()>=2){
				IsArmatureData = true;
				boneNum=fields[1].toInt();
			}
			while(!ef_in.atEnd()){
				textLine = ef_in.readLine();
				fields = textLine.split("\t");
				accumframeNum++;
				fields[0]=QString("%1").arg(accumframeNum);
				textLine = fields.join("\t");
				txtLs.append(textLine);
			}
			file_e.close();
		}
	}
	//出力
	QFile f(OFname);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
			 return;
	QTextStream out(&f);
	out << frameNum;
	if(IsArmatureData){
		out <<"\t" << boneNum;
	}
	out << "\n";
	for(int i=0; i<txtLs.size(); i++){
		out << txtLs[i] << "\n";
	}
	f.close();
}



bool dhMtool::CheckRgdBdyMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *obj_fp, int frameID)
{
	dhMCMarkerController* mc;
	for(int i=0;i<seq->ctrl.size();i++){
		mc = dynamic_cast<dhMCMarkerController*>(seq->ctrl[i]);//element_cast<dhMCMarkerController*>(seq->ctrl[i]);
		if(mc) break;
	}
    if(!mc){DH_LOG("Read marker motion file first.",0); return false; }
	dhMCMarkerControllerProperty mp = mc->Prop(frameID);

	dhMCRigidBodyController *mcrc;
	for(int i=0;i<seq->ctrl.size();i++){
		mcrc = dynamic_cast<dhMCRigidBodyController*>(seq->ctrl[i]);
		if(mcrc) break;
	}
	if(!mcrc){
		mcrc = seq->addRigidBodyController(obj_fp);
	}


	bool MrkExstnc=true;

	for(int i=0; i<mcrc->fpts->pointCount(); i++){
	//Q_FOR_EACH(dhFeaturePoint, fpts->fpts){
		QString pname(mcrc->fpts->point(i)->pointName());
		if(!mp.item.contains(pname) ){
            DH_LOG(QString("frameID: %1   %2 missed:").arg(frameID).arg(pname),0);
			MrkExstnc=false;
		}
	}

	return MrkExstnc;
}

bool dhMtool::CheckArmatureMrkExistenceInMoCapSequence(dhMoCapSequence *seq, dhFeaturePoints *body_fp, int frameID)
{
	dhMCMarkerController* mc;
	for(int i=0;i<seq->ctrl.size();i++){
		mc = dynamic_cast<dhMCMarkerController*>(seq->ctrl[i]);
		if(mc) break;
	}
    if(!mc){DH_LOG("Read marker motion file first.",0); return false; }
	dhMCMarkerControllerProperty mp = mc->Prop(frameID);

	dhMCArmatureController *mcac;
	for(int i=0;i<seq->ctrl.size();i++){
		mcac = dynamic_cast<dhMCArmatureController*>(seq->ctrl[i]);
		if(mcac) break;
	}
	if(!mcac){
		mcac = seq->addArmatureController(body_fp);
	}

	bool MrkExstnc=true;

	for(int i=0; i<mcac->fpts->pointCount(); i++){
	//Q_FOR_EACH(dhFeaturePoint, fpts->fpts){
		QString pname(mcac->fpts->point(i)->pointName());
		if(!mp.item.contains(pname) ){
            DH_LOG(QString("frameID: %1   %2 missed:").arg(frameID).arg(pname),0);
			MrkExstnc=false;
		}
	}

	return MrkExstnc;
}

void dhMtool::AppendFiles()
{
	AppendFilesGUI();
}


void dhMtool::ChangeMeshVcolorMode(dhMesh *trgMesh, bool ShowVcol)
{
	IDHMeshSupplier* ms = dhobject_cast<IDHMeshSupplier*>(trgMesh);
	//ms->SetDisplayType();
	dhMeshDisplayType mdt = ms->DisplayType();
	mdt.useVertexColor=ShowVcol;
	ms->SetDisplayType(mdt);
    //dhApp::updateAllGLViews();
    dhApp::updateAllWindows();
	//mDisplayType().useVertexColor=true;
}

dhVec4 dhMtool::Vcol(dhMesh *trgMesh, int vID)
{
	return (dhVec4(trgMesh->Vi(vID)->color(0),trgMesh->Vi(vID)->color(1),trgMesh->Vi(vID)->color(2)));
}

void dhMtool::setVcol(dhMesh *trgMesh, QString fileName)
{
	QFile f(fileName);
	if (f.open(QIODevice::ReadOnly | QIODevice::Text)){
		DH_LOG("setVcol: setting Vcol from "+fileName,0);
		QTextStream fin(&f);
		float c[3];
		QString textLine;
		QStringList fs;
		int nV=trgMesh->Nv();
		for(int i=0; i<nV; i++){
			textLine=fin.readLine();
			fs=textLine.split(",");
			for(int j=0; j<3; j++){
				c[j]=fs[j].toDouble();
			}
			trgMesh->Vi(i)->color=dhVec4((int)(255.0*c[0]),(int)(255.0*c[1]),(int)(255.0*c[2]));
		}
		f.close();
		dhApp::updateAllGLViews();
	}
}

void dhMtool::SaveOBJ_withVColor(dhMesh *trgMesh, QString fileName)
{
	QFile f(fileName);
	if (f.open(QIODevice::WriteOnly | QIODevice::Text)){
		DH_LOG("SaveOBJ_withVColor: Saving OBJ as "+fileName,0);
		QTextStream fout(&f);
		int Nv=trgMesh->Nv();
		for(int i=0; i<Nv; i++){
			
			fout << "v";
			for(int j=0; j<3; j++) fout <<" " << trgMesh->V(i)(j);
			for(int j=0; j<3; j++) fout <<" " << (int)(255.0*trgMesh->Vi(i)->color(j));
			fout << "\n";
		}
		int Nf=trgMesh->Nf();
		for(int i=0; i<Nf; i++){
			fout << "f";
			for(int k=0; k<3; k++){
				//OBJファイルの頂点番号は１始まりなので，１を足して書いておく
				fout << " " << trgMesh->Fi(i)->vtx_id(k)+1;
			}
			fout << endl;
		}
		f.close();
	}
	
}

//void dhMtool::CreateArrow(float headRatio, float ArrowLen)
//{
//	float coneH = headRatio*ArrowLen;
//	float bodyH = ArrowLen-coneH;
//	float coneR = headRatio*ArrowLen*0.1;
//	dhMesh* top=createCone("top",coneR,coneH,dhVec4(0,0,bodyH),dhVec4(0,0,1));
//	dhShapeCylinder *body = new dhShapeCylinder();
//	body->mScale.set(1.0,1.0,5);
//	
//	dhApp::updateAllGLViews();
//}

dhMat44 dhMtool::RotMatVec2Vec(dhVec4 &a, dhVec4 &b)
{
	dhMat44 m;
	m.init();

	Vector3d tmpa(a(0),a(1),a(2));	tmpa.normalize();
	Vector3d tmpb(b(0),b(1),b(2));	tmpb.normalize();
	double th = tmpa.angle(tmpb);
	if(th<1.0e-5 || fabs(3.1415-th) < 1.0e-5){ 
		//平行なときは外積が求まらないので Identity Matrixのまま返す
		//正反対なときは外積が求まらないので Identity Matrixのまま返す
	}else{
		Vector3d prod_ab;
		prod_ab.cross(tmpa,tmpb);
		prod_ab.normalize();
		Matrix R=GetGeneRotMat(prod_ab.x,prod_ab.y,prod_ab.z,th);
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				m(i,j)=R[i][j];
			}
		}
	}
	
	return m;
}

dhVec4 dhMtool::EquivRotAxis(dhMat44 TrM)
{
    dhVec4 k;
    float th;
    int stat=EquivRotAxisAndAngle(TrM, k, th);
    return k;
}

float dhMtool::EquivRotAngle(dhMat44 TrM)
{
    dhVec4 k;
    float th;
    int stat=EquivRotAxisAndAngle(TrM, k, th);
    return th;
}


dhMat44 dhMtool::RotMatAroundGivenVec(dhVec4 k, double th)
//[内容]  kr = (kx, ky, kz)周りにｔｈ[rad]回転する　一般回転行列4x4を求める
{
    dhMat44 Rot;
    k=k.normalized();
    Rot.setValue(0,0,k(0)*k(0)*(1-cos(th)) + cos(th));
    Rot.setValue(0,1,k(1)*k(0)*(1-cos(th)) - k(2)*sin(th));
    Rot.setValue(0,2,k(2)*k(0)*(1-cos(th)) + k(1)*sin(th));
    Rot.setValue(0,3,0.0);
    Rot.setValue(1,0,k(0)*k(1)*(1-cos(th)) + k(2)*sin(th));
    Rot.setValue(1,1,k(1)*k(1)*(1-cos(th)) + cos(th));
    Rot.setValue(1,2,k(2)*k(1)*(1-cos(th)) - k(0)*sin(th));
    Rot.setValue(1,3,0.0);
    Rot.setValue(2,0,k(0)*k(2)*(1-cos(th)) - k(1)*sin(th));
    Rot.setValue(2,1,k(1)*k(2)*(1-cos(th)) + k(0)*sin(th));
    Rot.setValue(2,2,k(2)*k(2)*(1-cos(th)) + cos(th));
    Rot.setValue(2,3,0.0);
    Rot.setValue(3,0,0.0);
    Rot.setValue(3,1,0.0);
    Rot.setValue(3,2,0.0);
    Rot.setValue(3,3,1.0);
    return(Rot);
}


dhMat44 dhMtool::TransMat2Registrate3Points(dhVec4 &srcP0, dhVec4 &srcP1, dhVec4 &srcP2, dhVec4 &dstP0,  dhVec4 &dstP1, dhVec4 &dstP2, int basePid)
{
	std::vector<Vector3d> _srcPs, _dstPs;
	_srcPs.push_back(Vec4ToVector3d(srcP0));
	_srcPs.push_back(Vec4ToVector3d(srcP1));
	_srcPs.push_back(Vec4ToVector3d(srcP2));
	_dstPs.push_back(Vec4ToVector3d(dstP0));
	_dstPs.push_back(Vec4ToVector3d(dstP1));
	_dstPs.push_back(Vec4ToVector3d(dstP2));
    Matrix TrM = TransMatToRegistratePointsToPoints(_srcPs, _dstPs, basePid);
	dhMat44 m;
	m=Mat2DHMat(TrM);
	return m;
}

dhMat44 dhMtool::TransMat2RegistratePoints(QVariantList& srcPs, QVariantList& dstPs, int dstPID)
//srcPs: 点群元の位置
//dstPs: 点群の行き先
//dstPID: どれかの点を原点としたい場合何番目の点か（デフォルトは-1。-1のときは重心が原点）
{
    dhMat44 TrM;
    if(srcPs.size()!=dstPs.size()){
        DH_LOG("TransMat2RegistratePoints:  size of srcPs != dstPs\n",0);
    }else{
#if 1
        QList<dhVec4> _srcPs, _dstPs;
        for(int i=0; i<srcPs.size(); i++){
            _srcPs.append(srcPs[i].value<dhVec4>());
            _dstPs.append(dstPs[i].value<dhVec4>());
        }
        TrM=   TransMatToRegistratePointsToPoints(_srcPs,_dstPs,dstPID);
        _srcPs.clear();
        _dstPs.clear();
#else
        std::vector<Vector3d> _srcPs, _dstPs;
        dhVec4 tmpV4;
        for(int i=0; i<srcPs.size(); i++){
            tmpV4=srcPs[i].value<dhVec4>();
            _srcPs.push_back(Vec4ToVector3d(tmpV4));
            tmpV4=dstPs[i].value<dhVec4>();
            _dstPs.push_back(Vec4ToVector3d(tmpV4));
        }
        Matrix tmpTrM=TransMatToRegistratePointsToPoints(_srcPs,_dstPs, dstPID);
        TrM=Mat2DHMat(tmpTrM);
        _srcPs.clear();
        _dstPs.clear();
#endif
    }
    return TrM;
}


dhMesh* dhMtool::GenerateMirrorMesh(dhMesh *orgMesh, dhVec4 p, dhVec4 n)
{
	dhMesh *mirrorMesh=dhnew<dhMesh>();
	dhVertex *v;
	dhVec4 mp;
	mirrorMesh->beginMeshGeneration();

	int vnum=orgMesh->PointCount();//->Npts();
	for(int i=0; i<vnum; i++){
		v=orgMesh->Vi(i);
		mp=MirrorP(v->pt,p,n);
		mirrorMesh->addVertex(mp);
	}
	int fnum=orgMesh->Nf();
	for(int i=0; i<fnum; i++){
		mirrorMesh->addFace(orgMesh->Fi(i)->vtx_id(2),orgMesh->Fi(i)->vtx_id(1),orgMesh->Fi(i)->vtx_id(0));
	}
	mirrorMesh->endMeshGeneration();
	return mirrorMesh;
}
dhArmature* dhMtool::GenerateMirrorArmature(dhArmature* orgArm,dhVec4 p, dhVec4 n,bool resetPosture)
{
	dhArmature *mirrorArm=dhnew<dhArmature>();
	mirrorArm->copyFrom(orgArm);
	int bnum=orgArm->boneCount();
	
	for(int i=0; i<bnum; i++){
		//mirrorArm->bone(i)->Twj0=mirrorArm->bone(i)->Twj= MirrorCF(orgArm->bone(i)->Twj0,p,n);
		mirrorArm->bone(i)->Twj0= MirrorCF(orgArm->bone(i)->Twj0,p,n);
		mirrorArm->bone(i)->Twj = MirrorCF(orgArm->bone(i)->Twj,p,n);
		//DH_LOG(mirrorArm->bone(i)->Twj0.toString(),0);
		dhVec4 mp;
		mp = MirrorP(orgArm->bone(i)->Ptail0,p,n);
		mp(3)=1.0;
		//mirrorArm->bone(i)->Ptail0 = mirrorArm->bone(i)->Ptail = mp;
		mirrorArm->bone(i)->Ptail0 = mp;
		mp = MirrorP(orgArm->bone(i)->Ptail,p,n);
		mp(3)=1.0;
		mirrorArm->bone(i)->Ptail = mp;
	}
	mirrorArm->calcInitFrames();
	if(resetPosture) mirrorArm->resetPosture();
	return mirrorArm;
}

void dhMtool::CopyJointConstraints(dhArmature* fromArm, dhArmature* toArm, bool Mirrored)
{
	dhVec4 jcmax, jcmin;
	for(int i=0, bn=fromArm->boneCount(); i<bn; i++){
		dhBone* fb=fromArm->bone(i);
		dhBone* tb=toArm->bone(fb->name);
		if(!Mirrored){
			tb->jcMax=fb->jcMax;
			tb->jcMin=fb->jcMin;
		}else{
			for(int j=0; j<3; j++){
                if(j==0 | (fb->jcMax(j)<fb->jcMin(j))){
                    tb->jcMax(j)= fb->jcMax(j);
                    tb->jcMin(j)= fb->jcMin(j);
				}else{
					tb->jcMax(j)=-fb->jcMin(j);
					tb->jcMin(j)=-fb->jcMax(j);
				}
			}
		}
	}
}
void dhMtool::MirrorJointAngle(dhArmature* orgArm,dhArmature* trgArm, dhVec4 p, dhVec4 n)
{
	if (trgArm==NULL){

	}

}

dhSkeletalSubspaceDeformation* dhMtool::GenerateMirrorHand(dhSkeletalSubspaceDeformation* ssd, dhVec4 p, dhVec4 n)
{
	dhMesh *mirrorMesh=GenerateMirrorMesh(dhobject_cast<dhMesh*>(ssd->MeshSupplier()),p,n);
	dhArmature *mirrorArm=GenerateMirrorArmature(ssd->Armature(),p,n);
	dhSkeletalSubspaceDeformation *newSSD=dhnew<dhSkeletalSubspaceDeformation>();
 //       desc.weightFilePath = weightFilePath; //desc.autoCalcWeight = autoCalcWeight;
 //       if(autoCalcWeight) desc.vertWtCalcMethod = useHeatEq ? dhSSD::HeatEquilibrium : dhSSD::ReciprocalOfDist;
 //       else               desc.vertWtCalcMethod = weightFilePath.isEmpty() ? dhSSD::NoCalc : dhSSD::FromFile;
 //       if(desc.vertWtCalcMethod == dhSSD::ReciprocalOfDist) desc.rodLinkNames = rodLinkNames;

	//descを作る
	dhSSD::Desc desc;
	desc.surface = dhobject_cast<IDHMeshSupplier*>(mirrorMesh);
	desc.armature = mirrorArm->AsIArmatureSupplier();//mirrorArm
	desc.weightFilePath="";
	desc.vertWtCalcMethod = dhSSD::HeatEquilibrium;
	newSSD->initialize(desc);
	newSSD->calcInitialGeometryFromCurrentPosture(dhobject_cast<IDHMeshSupplier*>(mirrorMesh));
	CopyJointConstraints(ssd->Armature(),mirrorArm,true);
	dhApp::updateAllGLViews();
	return newSSD;
}

void dhMtool::setTag(dhMoCapSequence *seq, int frameID, QString theTag)
{
	QMap<int,QString>::iterator p=seq->tags.find(frameID);
	if(p!=seq->tags.end()) seq->tags.insert(frameID, theTag);
	else seq->tags[frameID]=theTag;
}

void dhMtool::SaveGLScreen(QString FName)
{
	//dhQMainWindow* window = dhApp::window();
	
	//QPixmap pixmap= QPixmap::grabWidget(window);
	QPixmap pixmap= QPixmap::grabWidget(dhApp::mainTabWidget());
	const QImage img = pixmap.toImage();
	img.save(FName);
}

dhMat44 dhMtool::ObbCF(dhMesh *m, bool BuildBboxFPs, dhVec4 refPlusV)
{
    dhMat44 ObbCF;
    //Rapidのオブジェクトを作る
    PQP_Model* pmod = BuildPQPModel(m);
    //DH_LOG("ObbCF 0",0);

    Vector3d obbCenter;
    Vector3d axy;
//    Vector4d dst;
    Matrix3d R;
//    Matrix4d orgCF,Inv_woTc;
    Vector3d oaX,oaY,oaZ;
    BV *b=pmod->b;

    // box の中心座標 (世界座標系)
    obbCenter.set(b->To[0], b->To[1], b->To[2]);

    // 正規化されていない3軸
    R.set(b->R[0][0], b->R[0][1], b->R[0][2],
          b->R[1][0], b->R[1][1], b->R[1][2],
          b->R[2][0], b->R[2][1], b->R[2][2]);
    oaX.set(b->R[0][0],b->R[1][0],b->R[2][0]);	oaX.normalize();//多分normalizeは不要
    oaY.set(b->R[0][1],b->R[1][1],b->R[2][1]);	oaY.normalize();//多分normalizeは不要
    oaZ.set(b->R[0][2],b->R[1][2],b->R[2][2]);	oaZ.normalize();//多分normalizeは不要

    // OBBの軸を長さ順にソートする
    const float magx = (float)(fabs(b->d[0]));  // d[0] はいつも最長軸
    float magy = (float)(fabs(b->d[1]));
    float magz = (float)(fabs(b->d[2]));

    Vector3d ax(R.m00, R.m10, R.m20); // 最長軸
    Vector3d ay(R.m01, R.m11, R.m21);
    Vector3d az(R.m02, R.m12, R.m22);

    if (magy < magz) { // x -> y -> z の順に軸を長くする
        std::swap(ay, az);
        std::swap(magy, magz);
    }

    // OBB の軸を右手系にそろえる (y と z)
    axy.cross(ax, ay);
    if (axy.dot(az) < 0.0) {
        az.negate();
    }
    ObbCF.setColumn(0,dhVec4(ax.x,ax.y,ax.z));
    ObbCF.setColumn(1,dhVec4(ay.x,ay.y,ay.z));
    ObbCF.setColumn(2,dhVec4(az.x,az.y,az.z));
    ObbCF.setColumn(3,dhVec4(obbCenter.x,obbCenter.y,obbCenter.z));

    dhMat44 bboxDef;
    bboxDef.setColumn(0,magx*dhVec4(ax.x,ax.y,ax.z));
    bboxDef.setColumn(1,magy*dhVec4(ay.x,ay.y,ay.z));
    bboxDef.setColumn(2,magz*dhVec4(az.x,az.y,az.z));
    bboxDef.setColumn(3,dhVec4(obbCenter.x,obbCenter.y,obbCenter.z));
    // box の8隅の点の座標を計算するための単位立方体
    static const dhVec4 ibox[8] = {
        dhVec4(-1.0, -1.0, -1.0),
        dhVec4(-1.0, -1.0, +1.0),
        dhVec4(-1.0, +1.0, -1.0),
        dhVec4(-1.0, +1.0, +1.0),
        dhVec4(+1.0, -1.0, -1.0),
        dhVec4(+1.0, -1.0, +1.0),
        dhVec4(+1.0, +1.0, -1.0),
        dhVec4(+1.0, +1.0, +1.0),
    };
    //Bounding Boxに相当するFeaturePoints作成
    if (BuildBboxFPs){
        dhFeaturePoints* bbxPs = dhnew<dhFeaturePoints>();
        for(int i=0; i<8; i++){
            dhFeaturePoint *fp=bbxPs->addPoint(QString("P%1").arg(i));
            fp->setPosition(bboxDef*ibox[i]);
        }
        bbxPs->constructCache();
    }
    DH_LOG(QString("magx y z=%1 %2 %3").arg(magx).arg(magy).arg(magz),0);
    //長軸Plus方向が思っている方向かどうかのチェックをする　DefaultのままでNormゼロのときは判定しない
    if(refPlusV.size()>EPS_SAMEVAL_CHECK){

    }
    dhApp::updateAllWindows();

    delete pmod;
    return ObbCF;
}

PQP_Model* dhMtool::BuildPQPModel(dhMesh *m)
{
    PQP_Model* pqp_mod = new PQP_Model;

    double p0[3], p1[3], p2[3];
    int vID;
    pqp_mod->BeginModel();
    dhFace *f;
    //for(unsigned int j=0; j<this->polgrps[this->repGrID].f.size(); j++){
    for(int j=0; j<m->Nf(); j++){
        f = m->Fi(j);
        vID= f->vtx_id(0);
        for(int i=0; i<3; i++) p0[i]=m->V(vID)(i);
        vID= f->vtx_id(1);
        for(int i=0; i<3; i++) p1[i]=m->V(vID)(i);
        vID= f->vtx_id(2);
        for(int i=0; i<3; i++) p2[i]=m->V(vID)(i);
        //三角形に追加
        pqp_mod->AddTri(p0, p1, p2, j);
    }
    pqp_mod->EndModel();


    return pqp_mod;
}
void dhMtool::AnalyzeNormalDifferences_PConMesh(dhMesh *m, double minR, double maxR, int Pnum, QString colType)
{
    dhPointCloudAsFaceInternal *pc=m->GeneratePointCloud(Pnum);
    QList<dhVec3> aveNrmlList_min,aveNrmlList_max;
    QList<double> vecdots;
    for(int i=0;i<Pnum; i++){
        aveNrmlList_min.clear();
        aveNrmlList_max.clear();
        //i以外の頂点との距離に応じて，MaxR以内に入っているか，さらにMinR内に入っているかをチェックし，入っているNormalをリストアップ
        for(int ii=0; ii<Pnum; ii++){
            if(ii!=i){
                dhVec3 difV=(pc->Position(ii)-pc->Position(i)).toVec3();
                if (difV.norm()<maxR){
                    aveNrmlList_max.append(pc->Normal(ii).toVec3());
                    if(difV.norm()<minR){
                        aveNrmlList_min.append(pc->Normal(ii).toVec3());
                    }
                }
            }
        }

        //minR内の平均頂点Normalと，maxR内の平均頂点Normalの比較
        dhVec3 aveNrml_min, aveNrml_max;
        aveNrml_min.init();
        aveNrml_max.init();
        for(int ii=0; ii<aveNrmlList_min.size(); ii++) aveNrml_min += aveNrmlList_min[ii];
        if(aveNrmlList_min.size()>0){
            aveNrml_min = 1.0/double(aveNrmlList_min.size())*aveNrml_min;
            aveNrml_min.normalize();
        }
        for(int ii=0; ii<aveNrmlList_max.size(); ii++) aveNrml_max += aveNrmlList_max[ii];
        if(aveNrmlList_max.size()>0){
            aveNrml_max = 1.0/double(aveNrmlList_max.size())*aveNrml_max;
            aveNrml_max.normalize();
        }
        vecdots.append(aveNrml_min*aveNrml_max);//内積-1 - 1
    }
    dhColorPointCloud2* cpc=dynamic_cast<dhColorPointCloud2*>(dhApp::createObject("ColorPointCloud"));
    cpc->setNPoints(Pnum);
    for(int i=0; i<Pnum; i++){
        dhVec4 c = GetColor(vecdots[i],-1.0,1.0,colType);
        cpc->setPosAndColor(i,pc->Position(i),dhVec4((int)(255.0*c[0]),(int)(255.0*c[1]),(int)(255.0*c[2])));
    }
    dhApp::updateAllWindows();
}


void dhMtool::AnalyzeNormalDifferences_MeshVrtx(dhMesh *m,double minR, double maxR, QString colType)
//mの頂点Normalを参照し，近傍の頂点のNormalとの違いを解析してカラーマップを作る
{
    int vnum=m->Nv();
    QList<dhVec3> aveNrmlList_min,aveNrmlList_max;
    QList<double> vecdots;
    for(int i=0;i<vnum; i++){
        aveNrmlList_min.clear();
        aveNrmlList_max.clear();
        for(int ii=0; ii<vnum; ii++){
            if(ii!=i){
                dhVec3 difV=(m->V(ii)-m->V(i)).toVec3();
                if (difV.norm()<maxR){
                    aveNrmlList_max.append(m->Vnorm(ii).toVec3());
                    if(difV.norm()<minR){
                        aveNrmlList_min.append(m->Vnorm(ii).toVec3());
                    }
                }
            }
        }

        //minR内の平均頂点Normalと，maxR内の平均頂点Normalの比較
        dhVec3 aveNrml_min, aveNrml_max;
        aveNrml_min.init();
        aveNrml_max.init();
        for(int ii=0; ii<aveNrmlList_min.size(); ii++) aveNrml_min += aveNrmlList_min[ii];
        if(aveNrmlList_min.size()>0){
            aveNrml_min = 1.0/double(aveNrmlList_min.size())*aveNrml_min;
            aveNrml_min.normalize();
        }
        for(int ii=0; ii<aveNrmlList_max.size(); ii++) aveNrml_max += aveNrmlList_max[ii];
        if(aveNrmlList_max.size()>0){
            aveNrml_max = 1.0/double(aveNrmlList_max.size())*aveNrml_max;
            aveNrml_max.normalize();
        }
        vecdots.append(aveNrml_min*aveNrml_max);//内積-1 - 1
    }
    for(int i=0;i<vnum; i++){
        dhVec4 c = GetColor(vecdots[i],-1.0,1.0,colType);
        m->Vi(i)->color=dhVec4((int)(255.0*c[0]),(int)(255.0*c[1]),(int)(255.0*c[2]));
    }
    ChangeMeshVcolorMode(m,true);
}



dhVec4 dhMtool::GetColor(double v,double vmin,double vmax, QString colType)
//colType="hot-to-cold"/"cold-to-hot"/"jet"
{
    typedef struct {
        double r,g,b;
    }COLOR;
    COLOR c = {1.0,1.0,1.0}; // white
    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    double dv = vmax - vmin;
    if(colType=="hot-to-cold"){
       if (v < (vmin + 0.25 * dv)) {
          c.r = 0;
          c.g = 4 * (v - vmin) / dv;
          c.b = 1;
       } else if (v < (vmin + 0.5 * dv)) {
          c.r = 0;
          c.g = 1;
          c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
       } else if (v < (vmin + 0.75 * dv)) {
          c.r = 4 * (v - vmin - 0.5 * dv) / dv;
          c.g = 1;
          c.b = 0;
       } else {
          c.r = 1;
          c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
          c.b = 0;
       }
    }else if(colType=="cold-to-hot"){
        if (v < (vmin + 0.25 * dv)) {
           c.b = 0;
           c.g = 4 * (v - vmin) / dv;
           c.r = 1;
        } else if (v < (vmin + 0.5 * dv)) {
           c.b = 0;
           c.g = 1;
           c.r = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
        } else if (v < (vmin + 0.75 * dv)) {
           c.b = 4 * (v - vmin - 0.5 * dv) / dv;
           c.g = 1;
           c.r = 0;
        } else {
           c.b = 1;
           c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
           c.r = 0;
        }
    }
    return(dhVec4(c.r,c.g,c.b));
}

QStringList dhMtool::ElementActionTitles()
{
	QStringList sl=IDHElement::ElementActionTitles(); 
    sl<<"Append Files"<<"Test"<<"Transform Matrix from srcPs to dstPs";

	return sl;
}

bool dhMtool::OnElementActionCalled(const QString& cmd)
{
	if(cmd=="Test"){
		dhShapeCapsule *s = dhnew<dhShapeCapsule>();

		return true;
	}else if(cmd=="Append Files"){
		AppendFilesGUI();
		return true;
    }else if(cmd=="Transform Matrix from srcPs to dstPs"){
        QList<dhVec4> srcPs, dstPs;
        bool isOK;
        dhFeaturePoints* srcFPs =dynamic_cast<dhFeaturePoints*>(dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK));
        dhFeaturePoints* dstFPs =dynamic_cast<dhFeaturePoints*>(dhApp::elementSelectionDialog(dhFeaturePoints::type,&isOK));
        for(int i=0; i<srcFPs->pointCount(); i++){
            srcPs.append(srcFPs->point(i)->position());
            dstPs.append(dstFPs->point(i)->position());
        }
        dhMat44 trM = TransMatToRegistratePointsToPoints(srcPs,dstPs);
        dhMesh* srcM=dynamic_cast<dhMesh*>(dhApp::elementSelectionDialog(dhMesh::type,&isOK));
        srcM->SetM(trM*srcM->M());
        return true;
    }
	return IDHElement::OnElementActionCalled(cmd);
}

