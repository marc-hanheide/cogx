    **************************************************************
                                HOMEST
                              version 1.3
                          By Manolis Lourakis

                     Institute of Computer Science
            Foundation for Research and Technology - Hellas
                       Heraklion, Crete, Greece
    **************************************************************


GENERAL
This is homest, a copylefted C implementation of a technique for non-linear, robust
homography estimation from matched image point features. homest requires my levmar
Levenberg-Marquardt non-linear least squares implementation and LAPACK, available
from http://www.ics.forth.gr/~lourakis/levmar and http://www.netlib.org/clapack,
respectively. Also, it implements robust regression techniques for coping with
outliers. Note that homest does *not* include any means for detecting and matching
point features between images. Such functionality can be supplied by other software
such as D. Lowe's SIFT or (in some cases) S. Birchfield's KLT.

Briefly, the technique implemented by homest is the following:

1) Normalization of point coordinates to improve conditioning as
   described in
   R.I. Hartley "In Defense of the Eight-Point Algorithm",
   IEEE Trans. on PAMI, Vol. 19, No. 6, pp. 580-593, June 1997.

2) Least Median of Squares (LMedS) linear fitting to detect outliers,
   see P.J. Rousseeuw, "Least Median of Squares Regression", Journal
   of the American Statistics Association, Vol. 79, No. 388, pp. 871-880,
   Dec. 1984. To ensure adequate spatial distribution of point quadruples
   over the image, LMedS random sampling employs the bucketing technique
   proposed in
   Z. Zhang, R. Deriche, O. Faugeras, Q.T. Luong, 
   "A Robust Technique for Matching Two Uncalibrated Images Through the
   Recovery of the Unknown Epipolar Geometry", INRIA RR-2273, May 1994 

3) Non-linear refinement of the linear homography estimate by minimizing either:
   * the homographic transfer error in the second image
   * the symmetric homographic transfer error between both images
   * the Sampson error (see P.D. Sampson, "Fitting Conic Sections to ``very
     scattered'' Data: An Iterative Refinement of the Bookstein Algorithm",
     CGIP, Vol. 18, Is. 1, pp. 97-108, Jan. 1982.)
   * the reprojection error

   The minimization is performed using the Levenberg-Marquardt algorithm, see
   M.I.A. Lourakis, "levmar: Levenberg-Marquardt Nonlinear Least Squares
   Algorithms in C/C++", http://www.ics.forth.gr/~lourakis/levmar/

Apart from fully projective homographies, homest can also estimate
affine homographies. Affine homographies have their third row fixed to
[0, 0, 1] and arise when the image displacements come from a small
image region or a large focal length lens is being used. When
estimating affine homographies, the third step of the technique
described above is omitted: Non-linear refinement is unnecessary
for them because in that case, the algebraic error equals the
geometric one.

For more details on homographies, projective geometry and such,
refer to R. Hartley and A. Zisserman, "Multiple View Geometry in
Computer Vision", Cambridge University Press, 2000-3


COMPILATION
 - Make sure that levmar and LAPACK are installed on your system. Installation
   details for these can be found at their respective URLs mentioned above.

 - Edit the appropriate makefile to specify the location of your compiled
   levmar and LAPACK libraries.

 - On a Linux/Unix system, typing "make" will build both homest and the demo
   program using gcc.

 - Under Windows and if Visual C is installed & configured for command line
   use, type "nmake /f Makefile.vc" in a cmd window to build homest and the
   demo program. In case of trouble, read the comments on top of Makefile.vc

USE
homest() is the main routine for estimating a (projective) homography. See the
comments in homest.c for an explanation of its arguments; homest_demo.c illustrates
an example of using homest() with matched point pairs that are read from a text
file. An example of such point pairs is included in subdirectory test.
An affine homography can be estimated with the routine homestaff(), which is
employed when homest_demo is invoked with the -a option.

Typing 
./homest_demo  test/matches.txt 

should produce output similar to

./homest_demo: estimated homography [142 outliers out of 933 matches]

-0.4740631 -0.1668685 20.81597 
0.09884363 -0.5038501 -82.95201 
-0.0001062511 1.289454e-05 -0.5399047 

Homography RMS and RMedS errors for input points: 33.3778 0.483469

Send your comments/bug reports to lourakis **at** ics.forth.gr
