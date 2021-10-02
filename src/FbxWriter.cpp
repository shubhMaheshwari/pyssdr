///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - Skinning Decomposition Library                  //
//         Copyright (c) 2019, Electronic Arts. All rights reserved.         //
///////////////////////////////////////////////////////////////////////////////



#include "FbxWriter.h"
#include "FbxShared.h"
#include "LogMsg.h"
#include <sstream>
#include <Eigen/Dense>
#include <DemBones/MatBlocks.h>
#include <set>

using namespace std;
using namespace Eigen;

#define err(msgStr) {msg(1, msgStr); return false;}

class FbxSceneExporter: public FbxSceneShared {
public:
	FbxSceneExporter(bool embedMedia=true): FbxSceneShared(false) {
		(*(lSdkManager->GetIOSettings())).SetBoolProp(EXP_FBX_EMBEDDED, embedMedia);
	}

	//https://help.autodesk.com/view/FBX/2017/ENU/?guid=__cpp_ref__export_scene01_2main_8cxx_example_html
	//https://help.autodesk.com/view/FBX/2017/ENU/?guid=__cpp_ref__switch_binding_2main_8cxx_example_html
	bool save(const string& fileName) {
		// Create an exporter.
		FbxExporter* lExporter=FbxExporter::Create(lSdkManager, "");

		// Get the appropriate file format.
		int lFileFormat=lSdkManager->GetIOPluginRegistry()->GetNativeWriterFormat();

		// Initialize the exporter.
		if (!lExporter->Initialize(fileName.c_str(), lFileFormat, lSdkManager->GetIOSettings())) err("Call to FbxExporter::Initialize() failed.\n");

		// Export the scene to the file.
		lExporter->Export(lScene);

		lExporter->Destroy();

		return true;
	}

	void createMesh(const MatrixXd verts, const vector<vector<int>> faces){
		FbxMesh pmesh = FbxMesh::Create(lSdkManager,"mesh");
		pmesh->InitControlPoints(verts.cols())	
	}

	void createJoints(const vector<string>& name, const ArrayXi& parent, double radius) {
		FbxNode* lRoot=NULL;
		for (int j=0; j!=name.size(); j++)
			if (parent(j)==-1) {
				FbxSkeleton* lSkeletonAttribute=FbxSkeleton::Create(lScene, name[j].c_str());
				lSkeletonAttribute->SetSkeletonType(((parent==j).count()>0)?FbxSkeleton::eRoot:FbxSkeleton::eLimb);
				lSkeletonAttribute->Size.Set(radius);
				FbxNode* lSkeleton=FbxNode::Create(lScene, name[j].c_str());
				lSkeleton->SetNodeAttribute(lSkeletonAttribute);
				lSkeleton->SetRotationOrder(FbxNode::eSourcePivot, eEulerXYZ);
				lScene->GetRootNode()->AddChild(lSkeleton);
				lRoot=lSkeleton;
			}

		for (int j=0; j!=name.size(); j++) 
			if (parent(j)!=-1) {
				FbxSkeleton* lSkeletonAttribute=FbxSkeleton::Create(lScene, name[j].c_str());
				lSkeletonAttribute->SetSkeletonType(FbxSkeleton::eLimb);
				lSkeletonAttribute->Size.Set(radius);
				FbxNode* lSkeleton=FbxNode::Create(lScene, name[j].c_str());
				lSkeleton->SetNodeAttribute(lSkeletonAttribute);
				lSkeleton->SetRotationOrder(FbxNode::eSourcePivot, eEulerXYZ);
				lRoot->AddChild(lSkeleton);
			}
	}

	void addToCurve(const VectorXd& val, const VectorXd& fTime, FbxAnimCurve* lCurve) {
		lCurve->KeyModifyBegin(); 
		int idx=0;
		int nFr=(int)fTime.size();
		FbxTime lTime;
		for (int k=0; k<nFr; k++) {
			lTime.SetSecondDouble(fTime(k));
			idx=lCurve->KeyAdd(lTime);
			lCurve->KeySetValue(idx, (float)val(k));
			lCurve->KeySetInterpolation(idx, FbxAnimCurveDef::eInterpolationCubic);
			lCurve->KeySetTangentMode(idx, FbxAnimCurveDef::eTangentAuto);
		}
		lCurve->KeyModifyEnd();
	}

	void setJoints(const vector<string>& name, const VectorXd& fTime, const MatrixXd& lr, const MatrixXd& lt, const MatrixXd& lbr, const MatrixXd& lbt) {
		// Animation stack & layer.
		cout << " Set Skinning Weights" << endl;

		FbxString lAnimStackName="demBones";
		FbxAnimStack* lAnimStack=FbxAnimStack::Create(lScene, lAnimStackName);
		FbxAnimLayer* lAnimLayer=FbxAnimLayer::Create(lScene, "Base Layer");
		lAnimStack->AddMember(lAnimLayer);
		cout << " Set Skinning Weights" << endl;

		VectorXd val;
		for (int j=0; j!=name.size(); j++) {
			cout << j << " Set Skinning Weights" << endl;
			FbxNode* lSkeleton=lScene->FindNodeByName(FbxString(name[j].c_str()));
			lSkeleton->LclRotation.Set(FbxDouble3(lbr(0, j), lbr(1, j), lbr(2, j)));
			lSkeleton->LclTranslation.Set(FbxDouble3(lbt(0, j), lbt(1, j), lbt(2, j)));
			val=lr.col(j);
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data(), val.size()/3), fTime, lSkeleton->LclRotation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_X, true));
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data()+1, val.size()/3), fTime, lSkeleton->LclRotation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y, true));
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data()+2, val.size()/3), fTime, lSkeleton->LclRotation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z, true));
			val=lt.col(j);
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data(), val.size()/3), fTime, lSkeleton->LclTranslation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_X, true));
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data()+1, val.size()/3), fTime, lSkeleton->LclTranslation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_Y, true));
			addToCurve(Map<VectorXd, 0, InnerStride<3>>(val.data()+2, val.size()/3), fTime, lSkeleton->LclTranslation.GetCurve(lAnimLayer, FBXSDK_CURVENODE_COMPONENT_Z, true));
		}
		cout << " Set Skinning Weights" << endl;

	}



	
	void setSkinCluster(const vector<string>& name, const SparseMatrix<double>& w, const MatrixXd& gb) {
		cout << " Set Skinning Weights" << endl;
		FbxMesh* lMesh=firstMesh(lScene->GetRootNode());
		cout << " Set Skinning Weights" << endl;

		FbxSkin* lSkin=firstSkin(lMesh);
		if (lSkin==NULL) {
			lSkin=FbxSkin::Create(lScene, "demSkinCluster");
			lMesh->AddDeformer(lSkin);
			lSkin->SetSkinningType(FbxSkin::eLinear);
		}
		cout << " Set Skinning Weights" << endl;

		//Clear all clusters
		while (lSkin->GetClusterCount()) {
			FbxCluster* lCluster=lSkin->GetCluster(lSkin->GetClusterCount()-1);
			lSkin->RemoveCluster(lCluster);
		}
		cout << " Set Skinning Weights" << endl;

		//Clear all poses
		while (lScene->GetPoseCount()) lScene->RemovePose(lScene->GetPoseCount()-1);
		cout << " Set Skinning Weights" << endl;

		//Create a new bind pose
		FbxPose* lPose=FbxPose::Create(lScene, "demBindPose");
		lPose->SetIsBindPose(true);
		cout << " Set Skinning Weights" << endl;

		FbxAMatrix gMat=lMesh->GetNode()->EvaluateGlobalTransform();
		lPose->Add(lMesh->GetNode(), gMat);
		cout << " Set Skinning Weights" << endl;

		SparseMatrix<double> wT=w.transpose();
		int nB=(int)name.size();
		set<FbxNode*> added;
		for (int j=0; j<nB; j++) {
			ostringstream s;
			s<<"demCluster"<<j;
			FbxCluster* lCluster=FbxCluster::Create(lScene, s.str().c_str());
			FbxNode* lNode=lScene->FindNodeByName(FbxString(name[j].c_str()));
			
			lCluster->SetLink(lNode);
			lCluster->SetLinkMode(FbxCluster::eTotalOne);
			for (SparseMatrix<double>::InnerIterator it(wT, j); it; ++it) lCluster->AddControlPointIndex((int)it.row(), it.value());
			lCluster->SetTransformMatrix(gMat);

			//Equaivalent to jointMat=lJoint->EvaluateGlobalTransform(), but with better accuracy
			FbxAMatrix jointMat;
			Map<Matrix4d> mm(&jointMat.mData[0][0], 4, 4);
			mm=gb.blk4(0, j);

			lCluster->SetTransformLinkMatrix(jointMat);
			lSkin->AddCluster(lCluster);

			//Add to bind pose
			while ((lNode)&&(added.find(lNode)==added.end())) {
				added.insert(lNode);
				lPose->Add(lNode, lNode->EvaluateGlobalTransform());
				lNode=lNode->GetParent();
			}
		}
		cout << " Set Skinning Weights" << endl;

		lScene->AddPose(lPose);
	}
};

bool writeFBXs(string fileName,  DemBonesExt<double, float>& model, bool embedMedia) {
	msg(1, "Writing outputs:\n");

	FbxSceneExporter exporter(embedMedia);

	bool needCreateJoints=(model.boneName.size()==0);
	double radius;

	if (needCreateJoints) {
		model.boneName.resize(model.nB);
		for (int j=0; j<model.nB; j++) {
			ostringstream s;
			s<<"joint"<<j;
			model.boneName[j]=s.str();
		}
		radius=sqrt((model.u-(model.u.rowwise().sum()/model.nV).replicate(1, model.nV)).cwiseAbs().rowwise().maxCoeff().squaredNorm()/model.nS);
	}

	for (int s=0; s<model.nS; s++) {
		msg(1, "--> \""<<fileName<<"\" ");

		// Write Mesh
		exporter.createMesh(model.u,model.fv);

		cout << model.boneName.size() << " Bones" << endl;
		MatrixXd lr, lt, gb, lbr, lbt;
		model.computeRTB(s, lr, lt, gb, lbr, lbt);
		cout << model.boneName.size() << " Bones" << endl;

		if (needCreateJoints) exporter.createJoints(model.boneName, model.parent, radius);
		cout << model.boneName.size() << " Bones" << endl;
	
		exporter.setJoints(model.boneName, model.fTime.segment(model.fStart(s), model.fStart(s+1)-model.fStart(s)), lr, lt, lbr, lbt);
		cout << model.boneName.size() << " Bones" << endl;
		exporter.setSkinCluster(model.boneName, model.w, gb);
		cout << model.boneName.size() << " Bones" << endl;

		if (!exporter.save(fileName)) err("Error on exporting file.\n");
		cout << model.boneName.size() << " Bones" << endl;

		msg(1, "("<<model.fStart(s+1)-model.fStart(s)<<" frames)\n");
	}

	return true;
}
