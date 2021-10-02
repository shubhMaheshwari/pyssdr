///////////////////////////////////////////////////////////////////////////////
//               Dem Bones - Skinning Decomposition Library                  //
//         Copyright (c) 2019, Electronic Arts. All rights reserved.         //
///////////////////////////////////////////////////////////////////////////////

#include <DemBones/DemBonesExt.h>
#include <DemBones/MatBlocks.h>
#include "NumpyReader.h"
#include "FbxReader.h"
#include "FbxWriter.h"
#include "LogMsg.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

using namespace std;
using namespace Eigen;
using namespace Dem;

class MyDemBones: public DemBonesExt<double, float> {
public:
	double tolerance;
	int patience;

	MyDemBones(): tolerance(1e-3), patience(3) { nIters=100; }

	void compute() {
		prevErr=-1;
		np=patience;
		DemBonesExt<double, float>::compute();
	}

	void cbIterBegin() {
		msg(1, "    Iter #"<<iter<<": ");
	}

	bool cbIterEnd() {
		double err=rmse();
		msg(1, "RMSE = "<<err<<"\n");
		if ((err<prevErr*(1+weightEps))&&((prevErr-err)<tolerance*prevErr)) {
			np--;
			if (np==0) {
				msg(1, "    Convergence is reached!\n");
				return true;
			}
		} else np=patience;
		prevErr=err;
		return false;
	}

	void cbInitSplitBegin() {
		msg(1, ">");
	}

	void cbInitSplitEnd() {
		msg(1, nB);
	}

	void cbWeightsBegin() {
		msg(1, "Updating weights");
	}

	void cbWeightsEnd() {
		msg(1, " Done! ");
	}

	void cbTranformationsBegin() {
		msg(1, "Updating trans");
	}

	void cbTransformationsEnd() {
		msg(1, " Done! ");
	}

	bool cbTransformationsIterEnd() {
		msg(1, ".");
		return false;
	}

	bool cbWeightsIterEnd() {
		msg(1, ".");
		return false;
	}

	int run(MatrixX vert_data,vector< vector<int> > face_data,int init_bones=30,string outFile=""){
		// # Check if output and input is provided
		msg(1, "Reading Numpy array Rows:" << vert_data.rows() << "Vertices:" << vert_data.cols());

		msg(1, "Parameters:\n");

		msg(1, "    nBones (target)    = "<< nB <<"\n");
		msg(1, "    nInitIters         = "<< nInitIters << "\n");

		msg(1, "    nIters             = "<< nIters << "\n");
		msg(1, "    tolerance          = "<< tolerance << "\n");
		msg(1, "    patience           = "<< patience << "\n");

		msg(1, "    nTransIters        = "<< nTransIters << "\n");
		msg(1, "    nWeightsIters      = "<< nWeightsIters << "\n");
		
		msg(1, "    bindUpdate         = "<< bindUpdate << "\n");
		switch (bindUpdate) {
			case 0: msg(1, " (no update)"); break;
			case 1: msg(1, " (update joint positions)"); break;
			case 2: msg(1, " (regroup joints under one root)"); break;
		};

		msg(1, "    transAffine        = "<< transAffine<< "\n");
		msg(1, "    transAffineNorm    = "<< transAffineNorm<< "\n");
		msg(1, "    nnz                = "<< nnz<< "\n");
		msg(1, "    weightsSmooth      = "<< weightsSmooth<< "\n");
		msg(1, "    weightsSmoothStep  = "<< weightsSmoothStep<< "\n");

		if (!readNumpy(vert_data,face_data,*this)) return 1;

		if (nB==0) {
			nB = init_bones;
			msg(1, "Initializing bones: 1");
			init();
			msg(1, "\n");
		}

		msg(1, "Computing Skinning Decomposition:\n");
		
		compute();


		if (!writeFBXs(outFile, *this) and outFile!="") return 1;

		return 0;
	}	



private:
	double prevErr;
	int np;
};


PYBIND11_MODULE(pyssdr, handle){
	handle.doc() = "Python Wrapper for SSDR\nDem Bones - (c) Electronic Arts 2019\n - This tool only handles clean input data, i.e. only one piece of geometry with one skinCluster and no excessive joint.\n\
     - To hard-lock the transformations of bones: in the input fbx files, create bool attributes for joint nodes (bones) with name \"demLock\" and set the value to \"true\".\n\
     - To soft-lock skinning weights of vertices: in the input fbx files, paint per-vertex colors in gray-scale. The closer the color to white, the more skinning weights of the vertex are preserved.", '=', "1.2.0";

	pybind11::class_<MyDemBones>(
		handle,"MyDemBones"
		)
	.def(pybind11::init<>())
	.def("run",&MyDemBones::run)
	.def_readwrite("weightsSmoothStep",&MyDemBones::weightsSmoothStep)
	.def_readwrite("weightsSmooth",&MyDemBones::weightsSmooth)
	.def_readwrite("nnz",&MyDemBones::nnz)
	.def_readwrite("nWeightsIters",&MyDemBones::nWeightsIters)

	.def_readwrite("transAffineNorm",&MyDemBones::transAffineNorm)
	.def_readwrite("transAffine",&MyDemBones::transAffine)
	.def_readwrite("bindUpdate",&MyDemBones::bindUpdate)
	.def_readwrite("nTransIters",&MyDemBones::nTransIters)

	.def_readwrite("patience",&MyDemBones::patience)
	.def_readwrite("tolerance",&MyDemBones::tolerance)

	.def_readwrite("nIters",&MyDemBones::nIters)
	.def_readwrite("nInitIters",&MyDemBones::nInitIters)

	.def_readwrite("nB",&MyDemBones::nB);


}



