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
    vector<C3DPoint*> corners3D;
    vector<C2DPoint*> corners2D;
    CMatrix<double> 3Dxz(3,1);
    CMatrix<double> 2Dxz(3,1);
    3Dxz(2,0) = 1;
    double u;
    double v;
    double threshold;
    for (int i = 0; i < src3Dxz.size(); i++){
        3Dxz(0,0) = src3Dxz[i]->x;
        3Dxz(1,0) = src3Dxz[i]->z;
        2Dxz = projXZ * 3Dxz;
        u = 2Dxz(0,0)/2Dxz(2,0);
        v = 2Dxz(1,0)/2Dxz(2,0);
        for (int j = 0; j < corners.size(); j++){
            if (corners[j]->x - u < threshold && corners[j]->y - v < threshold){
                corners3D.push_back(src3Dxz[i]);
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

    // Step 2: Compute the translation vector T from the last column of prjMatrix

    // Step 3: Normalize the 3x3 camera calibration matrix K

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

    return;
}


