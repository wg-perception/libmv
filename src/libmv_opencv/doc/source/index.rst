libmv port
==========

This is a port of libmv to OpenCV

TODO
====

The following has to be achieved as a first step
 * figure out what OpenCV has already
 * replace the image type with the OpenCV one
 * remove Eigen by cv::Matx and make sure that does not impact the performance (Keir agreed that this would be good to remove an LGPL dependency and make it pure BSD)
 * replace hand-written functionalities with OpenCV ones (like convolution)
 * provide a lightweight OpenCV header copy for only the functionalities we need (so that libmv stays lightweight): core, features2d, imgproc.
   We will probably need to parse
   the C++ header and comment out whatever is not needed (because of all macros and stuff, we cannot copy the data, that would be too hard 
   to distinguish the ifdefs and crap). If you have any Clang/gcc-xml background, we could use their parser but that might be overkill. I think a simple
   regex where we comment out unused classes would suffice (there should not be much extra memory used after that, maybe some global variables)

Detailed TODO
=============

As mentioned by Keir last year, SfM should be divided as follows:
 * Level 0: Correspondence finding routines (SIFT/SURF/DAISY, KLT/NCC)
 * Level 1: Core solvers such as fundamental 7/8 point, 5-point euclidean relative pose, 5-point euclidean resection, etc. These involve a small, fixed number of points or cameras.
 * Level 2: Routines that can robustly solve Level 1 problems with noisy data.
 * Level 3: Any algorithm that involves multiple calls to level 2 routines. This includes general reconstruction from e.g. image collections or video sequences. These require e.g. doing a 2-view solve, then resectioning additional views, then triangulating more points, etc.

libmv tackles all levels and we want to split the core out. We could also have applications during our port but that will not be our focus. Maybe some tests/samples
will look like those apps.

More information: `Keir's mails <https://groups.google.com/d/msg/libmv-devel/akyJX4jfIu8/8Y9Q5EeKEnkJ/>`_

Notes on compiling libmv:

.. toctree::
   :maxdepth: 1

   compile_libmv.rst

Docs on what has been done
==========================

Development  ideas:

.. toctree::
   :maxdepth: 1

   opencv_comparison.rst

Guidelines for coding
=====================

Please follow:
http://opencv.willowgarage.com/wiki/CodingStyleGuide
but the Google conventions have priority for libmv
http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

Use 64float (double) for everything to keep high accuracy.

This is a collaborative effort to port libmv to OpenCV.


Datasets to use
===============

http://www.cvpapers.com/datasets.html

Progress
========

Sfm module is divided in the following (low-level) headers:

* **conditioning.hpp**

=============================   ========    ========   ====    =====================
Function                        Wrapper     Port       Test    Comments
=============================   ========    ========   ====    =====================
preconditionerFromPoints        **DONE**    ToDo       --
applyTransformationToPoints     **DONE**    ToDo       --
normalizePoints                 **DONE**    --         Yes*    Port is not needed.
normalizeIsotropicPoints        **DONE**    --         ToDo    Port is not needed.
=============================   ========    ========   ====    =====================

* **fundamental.hpp**

================================= ========    ========   ====    =====================
Function                          Wrapper     Port       Test    Comments
================================= ========    ========   ====    =====================
projectionsFromFundamental        **DONE**    **DONE**   Yes
fundamentalFromProjections        **DONE**    **DONE**   Yes
normalizedEightPointSolver        **DONE**    ToDo       Yes*
relativeCameraMotion              **DONE**    **DONE**   --
motionFromEssential               **DONE**    ToDo       Yes*
MotionFromEssentialChooseSolution **DONE**	  **DONE**	 Fail
fundamentalFromEssential          **DONE**    **DONE**   Yes
essentialFromFundamental          **DONE**    **DONE**   Yes
essentialFromRt                   **DONE**    ToDo       Yes*
normalizeFundamental              **DONE**    **DONE**   --
================================= ========    ========   ====    =====================

* **numeric.hpp**

=============================   ========    ========   ====    =====================
Function                        Wrapper     Port       Test    Comments
=============================   ========    ========   ====    =====================
meanAndVarianceAlongRows        **DONE**    **DONE**   Yes*
skewMat                         **DONE**    **DONE**   Yes
skewMatMinimal                  **DONE**    **DONE**   --
=============================   ========    ========   ====    =====================

* **projection.hpp**

=============================   ========    ========   ====    =====================
Function                        Wrapper     Port       Test    Comments
=============================   ========    ========   ====    =====================
depth							**DONE**	**DONE**   No
euclideanToHomogeneous          **DONE**    **DONE**   Yes
homogeneousToEuclidean          **DONE**    **DONE**   Yes      ToDo: check homogeneousToEuclidean( X, X );
P_From_KRt                      **DONE**    **DONE**   Yes*     P = K * [R t]
KRt_From_P                      **DONE**    ToDo       Yes*     RQ decomposition HZ A4.1.1 pag.579
=============================   ========    ========   ====    =====================

* **robust.hpp**

==========================================  ========    ========   ====    =====================
Function                                    Wrapper     Port       Test    Comments
==========================================  ========    ========   ====    =====================
fundamentalFromCorrespondences8PointRobust  **DONE**    ToDo       Yes     Mail: https://groups.google.com/d/msg/libmv_gsoc/-u3TPnBYmFs/4OnYErjjMSQJ
==========================================  ========    ========   ====    =====================

* **simple_pipeline.hpp**

=============================   ========    ========   ====    =====================
Function                        Wrapper     Port       Test    Comments
=============================   ========    ========   ====    =====================
libmv_solveReconstruction       **DONE**    ToDo       Yes     Slow test, many log messages
=============================   ========    ========   ====    =====================


* **triangulationhpp**

=============================   ========    ========   ====    =====================
Function                        Wrapper     Port       Test    Comments
=============================   ========    ========   ====    =====================
triangulateDLT                  **DONE**    **DONE**   Yes
nViewTriangulate                **DONE**    **DONE**   Yes
=============================   ========    ========   ====    =====================

Note: [*] Test only in double