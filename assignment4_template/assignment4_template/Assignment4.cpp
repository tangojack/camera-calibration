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
    //     CMatrix<double>& pPrjMatrix  A 3x4 camera projection matrix computed from the detected corners.
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

    for (int i = 0; i < (int)src3D.size(); i++){
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
    CMatrix<double> A;
	A.SetSize(2*src3Dxz.size(),8,0);
    for (int i = 0; i < src3Dxz.size(); i++){
        A(i*2,0) = src3Dxz[i]->x;
        A(i*2,1) = src3Dxz[i]->z;
        A(i*2,2) = 1;
        A(i*2,3) = 0;
        A(i*2,4) = 0;
        A(i*2,5) = 0;
        A(i*2,6) = -src2Dxz[i]->x * src3Dxz[i]->x;
        A(i*2,7) = -src2Dxz[i]->x * src3Dxz[i]->z;
        b(i*2,0) = src2Dxz[i]->x;
        
        A(i*2+1,0) = 0;
        A(i*2+1,1) = 0;
        A(i*2+1,2) = 0;
        A(i*2+1,3) = src3Dxz[i]->x;
        A(i*2+1,4) = src3Dxz[i]->z;
        A(i*2+1,5) = 1;
        A(i*2+1,6) = -src2Dxz[i]->y * src3Dxz[i]->x;
        A(i*2+1,7) = -src2Dxz[i]->y * src3Dxz[i]->z;
        b(i*2+1,0) = src2Dxz[i]->y;
    }
    CMatrix<double> Atran = A.Transpose();
    CMatrix<double> temp = Atran * A;
    CMatrix<double> Across = temp.Inverse() * Atran;
    CMatrix<double> pXZ = Across * b;
    
	
	
    int ctr = 0;
    CMatrix<double> projXZ(3,3);
    projXZ(2,2) = 1; 
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            if (i == 2 && j == 2)
                break;
            projXZ(i,j) = pXZ(ctr++,0);
        }
    }
	
		
	A.SetSize(2*src3Dyz.size(),8,0);
    for (int i = 0; i < src3Dyz.size(); i++){
        A(i*2,0) = src3Dyz[i]->y;
        A(i*2,1) = src3Dyz[i]->z;
        A(i*2,2) = 1;
        A(i*2,3) = 0;
        A(i*2,4) = 0;
        A(i*2,5) = 0;
        A(i*2,6) = -src2Dyz[i]->x * src3Dyz[i]->y;
        A(i*2,7) = -src2Dyz[i]->x * src3Dyz[i]->z;
        b(i*2,0) = src2Dyz[i]->x;
        
        A(i*2+1,0) = 0;
        A(i*2+1,1) = 0;
        A(i*2+1,2) = 0;
        A(i*2+1,3) = src3Dyz[i]->y;
        A(i*2+1,4) = src3Dyz[i]->z;
        A(i*2+1,5) = 1;
        A(i*2+1,6) = -src2Dyz[i]->y * src3Dyz[i]->y;
        A(i*2+1,7) = -src2Dyz[i]->y * src3Dyz[i]->z;
        b(i*2+1,0) = src2Dyz[i]->y;
    }
    Atran = A.Transpose();
    temp = Atran * A;
    Across = temp.Inverse() * Atran;
    CMatrix<double> pYZ = Across * b;
    
    ctr = 0;
    CMatrix<double> projYZ(3,3);
    projYZ(2,2) = 1; 
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            if (i == 2 && j == 2)
                break;
            projYZ(i,j) = pYZ(ctr++,0);
        }
    }
	/*ctr = 0;
	for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            matPrj(i,j) = projYZ(i,j);
        }
    }*/
    
	// Step 3: Using the estimated plane-to-plane projectivities, assign 3D coordinates
    //         to all the detected corners on the calibration pattern
	
    vector<C3DPoint*> CornerXZ3D;
    for (int i = 0; i < 5; i++){
    	for (int j = 0; j < 4; j++){
    		C3DPoint* n = new C3DPoint(0.5 + (i*2), 0, 0.5 + (j*2));
    		CornerXZ3D.push_back(n);
    	}
    }
	
    vector<C3DPoint*> CornerYZ3D;
    for (int i = 0; i < 5; i++){
    	for (int j = 0; j < 4; j++){
    		C3DPoint* n = new C3DPoint(0, 0.5 + (i*2), 0.5 + (j*2));
    		CornerYZ3D.push_back(n);
    	}
    }
	
    vector<C2DPoint*> CornerFakeXZ2D;
	double u, v;
	//Apply the plane projection
    for(int i = 0; i < CornerXZ3D.size(); i++){
    	CMatrix<double> temp(3,1);
    	temp(0,0) = CornerXZ3D[i]->x;
    	temp(1,0) = CornerXZ3D[i]->y;
    	temp(2,0) = CornerXZ3D[i]->z;
		
		CMatrix<double> xz2D(3,1);
    	xz2D = projXZ * temp;
        u = xz2D(0,0);
        v = xz2D(1,0);
        CornerFakeXZ2D.push_back(new C2DPoint(u,v));
    }
	

	vector<C2DPoint*> CornerFakeYZ2D;
	for(int i = 0; i < CornerYZ3D.size(); i++){
    	CMatrix<double> temp(3,1);
    	temp(0,0) = CornerYZ3D[i]->x;
    	temp(1,0) = CornerYZ3D[i]->y;
    	temp(2,0) = CornerYZ3D[i]->z;
		
		CMatrix<double> yz2D(3,1);
    	yz2D = projYZ * temp;
        u = yz2D(0,0);
        v = yz2D(1,0);
        CornerFakeYZ2D.push_back(new C2DPoint(u,v));
    }
	

	vector<C3DPoint*> corners3D;
	vector<C2DPoint*> corners2D;
	int threshold = 25;
	int yo = 0;
    for (int i = 0; i < CornerFakeXZ2D.size(); i++){
        u = CornerFakeXZ2D[i]->x;
        v = CornerFakeXZ2D[i]->y;
        for (int j = 0; j < corners.size(); j++){
            if (corners[j]->x - u < threshold && corners[j]->x - u > -threshold && corners[j]->y - v < threshold && corners[j]->y - v > -threshold){
				yo++;
				corners3D.push_back(CornerXZ3D[i]);
				corners2D.push_back(corners[j]);
            }
        }
    }
	
	
    for (int i = 0; i < CornerFakeYZ2D.size(); i++){
        u = CornerFakeYZ2D[i]->x;
        v = CornerFakeYZ2D[i]->y;
        for (int j = 0; j < corners.size(); j++){
            if (corners[j]->x - u < threshold && corners[j]->x - u > -threshold && corners[j]->y - v < threshold && corners[j]->y - v > -threshold){
				yo++;
                corners3D.push_back(CornerYZ3D[i]);
				corners2D.push_back(corners[j]);
            }
        }
    }
    // Step 4: Estimate a 3x4 camera projection matrix from all the detected corners on
    //         the calibration pattern using linear least squares
    A.SetSize(2*corners3D.size(),12,0);
	CMatrix<double> U,D,V,P;
	for (int i = 0; i < corners3D.size(); i++){
        A(i*2,0) = corners3D[i]->x;
        A(i*2,1) = corners3D[i]->y;
		A(i*2,2) = corners3D[i]->z;
        A(i*2,3) = 1;
        A(i*2,4) = 0;
        A(i*2,5) = 0;
        A(i*2,6) = 0;
		A(i*2,7) = 0;
        A(i*2,8) = -corners2D[i]->x * corners3D[i]->x;
        A(i*2,9) = -corners2D[i]->x * corners3D[i]->y;
		A(i*2,10) = -corners2D[i]->x * corners3D[i]->z;
        A(i*2,11) = -corners2D[i]->x;
        
        A(i*2+1,0) = 0;
        A(i*2+1,1) = 0;
        A(i*2+1,2) = 0;
        A(i*2+1,3) = 0;
        A(i*2+1,4) = corners3D[i]->x;
        A(i*2+1,5) = corners3D[i]->y;
		A(i*2+1,6) = corners3D[i]->z;
        A(i*2+1,7) = 1;
		A(i*2+1,8) = -corners2D[i]->y * corners3D[i]->x;
        A(i*2+1,9) = -corners2D[i]->y * corners3D[i]->y;
		A(i*2+1,10) = -corners2D[i]->y * corners3D[i]->z;
        A(i*2+1,11) = -corners2D[i]->y;
    }
	A.SVD2(U,D,V);
	P = V.SubMat(0,11,11,11);
	ctr = 0;
    matPrj(2,3) = yo;
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            if (i == 2 && j == 3)
                break;
            matPrj(i,j) = P(ctr++,0);
        }
    }
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
    //         prjMatsrix into the product of K and R using QR decomposition
    CMatrix<double> temp(3,3);
	temp = prjMatrix.SubMat(0,2,0,2); //Other Part of the projection matrix
	
	temp = temp.Transpose(); //Temp is KR, we want K and R
	
	CMatrix<double> Q(3,3), R_qr(3,3); //Defining the QR matrices
	temp.QR2(Q,R_qr); // R_qr is lower triangular

	CMatrix<double> R(3,3); //Rotation
	R = Q.Transpose();
	prjK = R_qr.Transpose();

    // Step 2: Compute the translation vector T from the last column of prjMatsrix

	CMatrix<double> translation(3,1);
	translation = prjMatrix.SubMat(0,2,3,3); //Translation Vector

    // Step 3: Normalize the 3x3 camera calibration matrix K
    
	if (prjK(2,2) != 1){
    	for (int i = 0; i < 3; i++){
        	for (int j = 0; j < 3; j++){
            	prjK(i,j) = prjK(i,j)/prjK(2,2);
        	}
    	}
    }
    if (prjK(0,0) < 0){
    	prjK(0,0) = prjK(0,0) * -1;
    	for (int i = 0; i < 3; i++){
        	R(0,i) = R(0,i) * -1;
    	}
    }
    if (prjK(1,1) < 0){
    	prjK(0,1) = prjK(0,1) * -1;
    	prjK(1,1) = prjK(1,1) * -1;
    	for (int i = 0; i < 3; i++){
        	R(1,i) = R(1,i) * -1;
    	}
    }
	

	//Making RT from R and translation
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			prjRt(i,j) = R(i,j);
		}
	}
	for (int j = 0; j < 3; j++){
			prjRt(j,3) = translation(j,0);
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
    //                                          correspondence with the projection matrix having the same index in prjMatss.
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
    //            prjMatss.size() = 2
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

	CMatrix<double> X;
    for (int j = 0; j < 8; j++){
		CMatrix<double> A(4,4), U, D, V;
		if((*src2Ds[0])[j] != NULL && (*src2Ds[1])[j] != NULL){
  			for (int i = 0; i < 2 ; i++){
        		A(i*2,0) = (*src2Ds[i])[j]->x * (*prjMats[i])(2,0) - (*prjMats[i])(0,0);
        		A(i*2,1) = (*src2Ds[i])[j]->x * (*prjMats[i])(2,1) - (*prjMats[i])(0,1);
        		A(i*2,2) = (*src2Ds[i])[j]->x * (*prjMats[i])(2,2) - (*prjMats[i])(0,2);
        		A(i*2,3) = (*src2Ds[i])[j]->x * (*prjMats[i])(2,3) - (*prjMats[i])(0,3);
        	
       			A(i*2+1,0) = (*src2Ds[i])[j]->y * (*prjMats[i])(2,0) - (*prjMats[i])(0,0);
        		A(i*2+1,1) = (*src2Ds[i])[j]->y * (*prjMats[i])(2,1) - (*prjMats[i])(0,1);
        		A(i*2+1,2) = (*src2Ds[i])[j]->y * (*prjMats[i])(2,2) - (*prjMats[i])(0,2);
        		A(i*2+1,3) = (*src2Ds[i])[j]->y * (*prjMats[i])(2,3) - (*prjMats[i])(0,3);
    		}
    		A.SVD2(U,D,V);
			X = V.SubMat(0,3,3,3);
			if (X(3,0) == 0)
				res3D.push_back(NULL);
			else
				res3D.push_back(new C3DPoint(X(0,0), X(1,0), X(2,0), X(3,0)));
		}
	}
    return;
}


