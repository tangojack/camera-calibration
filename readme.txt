Calibrate Function
1. I've found out which points are on the XZ plane by checking if the y coordinate of the 3D point is zero and which points are on
the YZ plane by checking if the by checking if the x coordinate of the 3D point is zero
2. The plane projectivity matrixes are estimated by using the pseudo inverse method. The inbuilt functions of matrix.h are used.
3. Coordinates of 180 corners are created and pushed into two vectors. CornerXZ3D has 90 3D corner coordinates and CornerYZ3D has 90
4. I've then looped through the 90 corners in two separate loops and applied the estimated plane-projectivity matrices computed in ttep two.
For each u and v got from this process, I check if they are close enough(depending on the threshold) to any of the 2D corner coordinates
which we're given in the beginning. If its close enough, the 2D corner coordinates are push backed into the vector corners2D and the
orresponding 3D corner coordinates are pushed into the vector corners3D.
5. Now, that we have enough points in corners2D and corners3D and the values in them are accurate, we calculate the camera projection 
matrix using the SVD2() function.
6. The value of matPrj is normalized by dividing it by the scale factor.

Decompose Function
1. QR decomposition is used to decompose he projection matrix. Helper functions used have been described in matrix.h
The camera calibration matrix prjK is normalized, and the Rotation and translation vectors are calculated separately and then combined to
form prjRt.

Triangulate Function
1. The src2Ds is esentially a 2D matrix in which each row consists of different object points(u,v coordinates) with a certain
projection matrix. Each column is the image of a the same object with different u,v coordinates and different projection matrix
2. I've looped through each column of the matrix and calculated the number of rows in which the image point is a NULL vector. If 
the number of rows is less than 2, then NULL is pushed back into the res3D vector as 1 point is not enough to calculate the 3D point..
Else, the 3D point is calulated using the SVD2 method
