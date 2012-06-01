libmv port
==========

This is a port of libmv to OpenCV

TODO
====

The following has to be achieved as a first step
 * figure out what OpenCV has already and why it sucks
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


To make sure we keep up to date with libmv SVN:
===============================================

Add to the end of your ~/.subversion/servers file:

.. code-block:: sh

  ssl-trust-default-ca = no

(so that git SVN works)

1) Define the new branch in .git/config :

.. code-block:: sh

  [svn-remote "libmv-upstream"]
          url = https://svn.blender.org/svnroot/bf-blender/trunk/blender/extern/libmv/
          fetch = :refs/remotes/git-svn-libmv-upstream

2) Import the SVN branch (libmv appeared aaround revision 41000)

.. code-block:: sh

  git svn fetch libmv-upstream -r 41000:HEAD

3) Hook up a local Git branch to the remote branch:

.. code-block:: sh

  git branch --track libmv-upstream git-svn-libmv-upstream

5) Checkout and update from SVN once in while:

.. code-block:: sh

  git checkout libmv-upstream
  git svn rebase

6) Whenever changes need to be merged (and therefore the first time):

.. code-block:: sh

  git checkout master
  git merge libmv-upstream
