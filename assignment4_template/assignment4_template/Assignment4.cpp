#include "stdAfx.h"
#include "Assignment4.h"

// Assignment4 Source File 

////////////////////////////////////////////////////////////////////////////////
// A brief description of C2DPoint and C3DPoint
//
// class C2DPoint
// {
// public:
//        double x; // stores the x coordinate
//        double y; // stores the y coordinate
// };
//
// class C3DPoint
// {
// public:
//        double x; // stores the x coordinate
//        double y; // stores the y coordinate
//        double z; // stores the y coordinate
// };
//
// Note:
// Although C2DPoint and C3DPoint do contain other data members,
// don't bother with them
//

BOOL CCamera::Calibrate(const vector<C2DPoint*>& src2D, const vector<C3DPoint*>& src3D, 
                        const vector<C2DPoint*>& corners, CMatrix<double>& matPrj)
{
    // INPUT:
    //     vector<C2DPoint*>& src2D      This contains a list of 2D coordinates for the image points specified 
    //                                   by the user in the image. Each point in this list is in 1-to-1 
    //                                   correspondence with the point having the same index in src3D.
    //
    //     vector<C3DPoint*>& src3D      This contains a list of 3D coordinates for the image points specified
    //                                   by the user in the image. Each point in this list is in 1-to-1 
    //                                   correspondence with the point having the same index in src2D.
    //
    //     vector<C2DPoint*>& corners    This contains a list of 2D coordinates for the detected corners.
    //
    // OUTPUT:
    //     CMatrix<double>& pPrjMatrix   A 3x4 camera projection matrix computed from the detected corners.
    //
    // Please refer to the tutorial for the usage of the related libraries.

    matPrj.SetSize(3,4,0);

    //////////////////////////
    // Begin your code here
    
    // Step 1: Classify the input 3D points into points on the x-z planes, points on
    //         the y-z plane, and points not on any calibration planes
    vector<C3DPoint*> src3Dxz;
    vector<C2DPoint*> src2Dxz;
    vector<C3DPoint*> src3Dyz;
    vector<C2DPoint*> src2Dyz;

    for (int i = 0; i < src3D.size(); i++){
        if (src3D[i]->x == 0){
            src3Dyz.push_back(src3D[i]);
            src2Dyz.push_back(src2D[i]);
        }
        if (src3D[i]->y == 0){
            src3Dxz.push_back(src3D[i]);
            src2Dxz.push_back(src2D[i]);
        }
    }
    // Step 2: Estimate a plane-to-plane projectivity for each of the calibration planes
    //         using the input 2D/3D point pairs
    CMatrix<double> b(8,1);

    CMatrix<double> Axz(8,8), Uxz, Dxz, Vxz;
    for (int i = 0; i < src3Dxz.size(); i++){
        Axz(i*2,0) = src3Dxz[i]->x;
        Axz(i*2,1) = src3Dxz[i]->z;
        Axz(i*2,2) = 1;
        Axz(i*2,3) = 0;
        Axz(i*2,4) = 0;
        Axz(i*2,5) = 0;
        Axz(i*2,6) = -src2Dxz[i]->x * src3Dxz[i]->x;
        Axz(i*2,7) = -src2Dxz[i]->x * src3Dxz[i]->z;
        b(i*2,0) = -src2Dxz[i]->x;
        
        Axz(i*2+1,0) = src3Dxz[i]->x;
        Axz(i*2+1,1) = src3Dxz[i]->z;
        Axz(i*2+1,2) = 1;
        Axz(i*2+1,3) = 0;
        Axz(i*2+1,4) = 0;
        Axz(i*2+1,5) = 0;
        Axz(i*2+1,6) = -src2Dxz[i]->x * src3Dxz[i]->x;
        Axz(i*2+1,7) = -src2Dxz[i]->x * src3Dxz[i]->z;
        b(i*2+1,0) = -src2Dxz[i]->y;
    }
    CMatrix<double> Axztran = Axz.Transpose();
    CMatrix<double> temp = Axztran * Axz;
    CMatrix<double> Axzcross = temp.Inverse() * Axztran;
    CMatrix<double> p = Axzcross * b;
    
    int ctr = 0;
    CMatrix<double> projXZ(3,3);
    projXZ(2,2) = 1; 
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            if (i == 2 && j == 2)
                break;
            projXZ(i,j) = p(ctr++,0);
        }
    }

    // Step 3: Using the estimated plane-to-plane projectivities, assign 3D coordinates
    //         to all the detected corners on the calibration pattern

    vector<C3DPoint*> 3DCornerXZ;
    for (int i = 0; i < 5; i++){
    	for (int j = 0; j < 4; j++){
    		C3DPoint* n = new C3DPoint();
    		n->x = 0.5 + (i*2);
    		n->y = 0;
    		n->z = 0.5 + (j*2);
    		3DCornerXZ.push_back(n);
    	}
    }
    vector<C3DPoint*> 3DCornerYZ;
    for (int i = 0; i < 5; i++){
    	for (int j = 0; j < 4; j++){
    		C3DPoint* n = new C3DPoint();
    		n->x = 0;
    		n->y = 0.5 + (i*2);
    		n->z = 0.5 + (j*2);
    		3DCornerXZ.push_back(n);
    	}
    }
    vector<C2DPoint*> 2DCornerFakeXZ;
    //Apply the plane projection
    for(int i = 0; i < 3DCornerXZ.size(); i++){
    	CMatrix<double> temp(3,1);
    	temp(0,0) = 3DCornerXZ[i]->x;
    	temp(1,0) = 3DCornerXZ[i]->y;
    	temp(2,0) = 3DCornerXZ[i]->z;
		
		CMatrix<double> 2Dxz(3,1);
    	2Dxz = projXZ * temp;
        u = 2Dxz(0,0)/2Dxz(2,0);
        v = 2Dxz(1,0)/2Dxz(2,0);

        2DCornerFakeXZ.push_back(new C2DPoint(u,v));
    }
    double threshold = 10;
    for (int i = 0; i < 2DCornerFakeXZ.size(); i++){
        u = 2DCornerFakeXZ[i]->x;
        v = 2DCornerFakeXZ[i]->y;
        for (int j = 0; j < corners.size(); j++){
            if (corners[j]->x - u < threshold && corners[j]->y - v < threshold){
                corners3D.push_back(3DCornerXZ[i]);
            }
        }
    }
    // Step 4: Estimate a 3x4 camera projection matrix from all the detected corners on
    //         the calibration pattern using linear least squares
    
    corners3D

    return TRUE;
}

void CCamera::Decompose(const CMatrix<double>& prjMatrix, CMatrix<double>& prjK, CMatrix<double>& prjRt)
{
    // INPUT:
    //     CMatrix<double>& prjMatrix    This is a 3x4 camera projection matrix to be decomposed.
    //
    // OUTPUT:
    //     CMatrix<double>& prjK         This is the 3x3 camera calibration matrix K.
    //
    //     CMatrix<double>& prjRt        This is the 3x4 matrix composed of the rigid body motion of the camera.
    //
    // Please refer to the tutorial for the usage of the related libraries.

    prjK.SetSize(3,3,0);
    prjRt.SetSize(3,4,0);

    //////////////////////////
    // Begin your code here
    
    // Step 1: Decompose the 3x3 sub-matrix composed of the first 3 columns of
    //         prjMatrix into the product of K and R using QR decomposition
    CMatrix<double>	temp(3,3), trans(3,1);
    CMatrix<double> Q(3,3), Rinv(3,3), Rdash(3,3), K(3,3);
    temp = prjMatrix.SubMat(0,2,0,2);
    T = prjMatrix.SubMat(0,2,3,3);
    temp = temp.Inverse();
    temp.QR(Q,Rdash);
    R = Q.Transpose();
    K = Rdash.Inverse();

    // Step 2: Compute the translation vector T from the last column of prjMatrix

    // Step 3: Normalize the 3x3 camera calibration matrix K
    if (K(2,2) != 1){
    	for (int i = 0; i < 3; i++){
        	for (int j = 0; j < 3; j++){
            	K(i,j) = K(i,j)/K(2,2);
        	}
    	}
    }
    if (K(0,0) < 0){
    	K(0,0) = K(0,0) * -1;
    	for (int i = 0; i < 3; i++){
        	R(0,i) = R(0,i) * -1;
    	}
    }
    if (K(1,1) < 0){
    	K(0,1) = K(0,1) * -1;
    	K(1,1) = K(1,1) * -1;
    	for (int i = 0; i < 3; i++){
        	R(1,i) = R(1,i) * -1;
    	}
    }

    return;
}

void CCamera::Triangulate(const vector<CMatrix<double>*>& prjMats, const vector<vector<C2DPoint*>*>& src2Ds,
                            vector<C3DPoint*>& res3D)
{
    // INPUT:
    //     vector<CMatrix<double>*> prjMats     A list of projection matrices
    //
    //     vector<vector<C2DPoint*>*> src2Ds    A list of image point lists, each image point list is in 1-to-1
    //                                          correspondence with the projection matrix having the same index in prjMats.
    //
    // OUTPUT:
    //     vector<C3DPoint*> res3D                A list of 3D coordinates for the triangulated points.
    //
    // Note:
    //    - src2Ds can be considered as a 2D array with each 'column' containing the image positions 
    //      for the same 3D point in different images. If any of the image does not contain the image for a particular
    //      point, the corresponding element in src2Ds will be a Null vector. For example, if there are two images, 
    //      and we know 8 pairs of corresponding points, then
    //      
    //            prjMats.size() = 2
    //            src2Ds.size() = 2
    //            src2Ds[k]->size() = 8           // k >= 0 and k < no. of images - 1
    //    
    //    - If for any reason the 3D coordinates corresponding to a 'column' in src2Ds cannot be computed,
    //      please push in a NULL as a place holder. That is, you have to make sure that each point in res3D
    //      must be in 1-to-1 correspondence with a column in src2Ds, i.e., 
    //      
    //            src2Ds[k]->size() == src3D.size()    // k >= 0 and k < no. of images - 1
    //
    // Please refer to the tutorial for the usage of related libraries.

    //////////////////////////
    // Begin your code here
    vector<CMatrix<double>*> prjMatsInv;
    prjMatsInv.push_back(prjMats[0].Inverse());
    prjMatsInv.push_back(prjMats[1].Inverse());

    for (int j = 0; j < 8; i++){
		CMatrix<double> A(4,4), U, D, V;
		if(src2Ds[0][j] != NULL && src2Ds[1][j] != NULL){
  			for (int i = 0; i < 2 ; i++){
        		A(i*2,0) = src2Ds[i][j]->x * prjMat[i](2,0) - prjMat[i](0,0);
        		A(i*2,1) = src2Ds[i][j]->x * prjMat[i](2,1) - prjMat[i](0,1);
        		A(i*2,2) = src2Ds[i][j]->x * prjMat[i](2,2) - prjMat[i](0,2);
        		A(i*2,3) = src2Ds[i][j]->x * prjMat[i](2,3) - prjMat[i](0,3);
        	
       			A(i*2+1,0) = src2Ds[i][j]->y * prjMat[i](2,0) - prjMat[i](0,0);
        		A(i*2+1,1) = src2Ds[i][j]->y * prjMat[i](2,1) - prjMat[i](0,1);
        		A(i*2+1,2) = src2Ds[i][j]->y * prjMat[i](2,2) - prjMat[i](0,2);
        		A(i*2+1,3) = src2Ds[i][j]->y * prjMat[i](2,3) - prjMat[i](0,3);
    		}
    		A.SVD2(U,D,V);
			X = SubMat(0,3,3,3);
			if (X(3,0) == 0)
				res3D.push_back(NULL);
			else
				res3D.push_back(new C3DPoint(X(0,0), X(1,0), X(2,0), X(3,0)));
		}
	}
    return;
}


