#include <opencv2/sfm/sfm.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include "recon2v.hpp"

// Used to display 3D structure using Qt widget
#include <QApplication>
#include <QtGui>
#include <QtOpenGL>
#include "glwidget.h"

#ifdef __APPLE__
    #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

using namespace std;
using namespace cv;

static void help() {
	cout
			<< "\n------------------------------------------------------------------\n"
			<< " This program shows the two view reconstruction capabilities in the \n"
			<< " OpenCV Structure From Motion (SFM) module.\n"
			<< " It uses the following data from the VGG datasets at ...\n"
			<< " Usage:\n"
//    << "       reconv2 view1_2D_pts.txt view2_2D_pts.txt\n"
			<< "       reconv2 _pts.txt \n "
			<< " where the first line has the number of points and each subsequent \n"
			<< " line has entries for matched points as: \n"
			<< " x1 y1 x2 y2 \n"
			<< "------------------------------------------------------------------\n\n"
			<< endl;
}

int main(int argc, char** argv) {

	// Do projective reconstruction
	bool is_projective = true;

	// Assume noise free
	bool has_outliers = false;

	// Read 2D points from text file

	Mat_<double> x1, x2;
	int npts;

	if (argc < 2) {
		help();
		exit(0);
	} else {
		ifstream myfile(argv[1]);
		if (!myfile.is_open()) {
			cout << "Unable to read file: " << argv[1] << endl;
			exit(0);

		} else {
			string line;

			// Read number of points
			getline(myfile, line);
			npts = (int) atof(line.c_str());
			x1 = Mat_<double>(2, npts);
			x2 = Mat_<double>(2, npts);

			// Read the point coordinates
			for (int i = 0; i < npts; ++i) {
				getline(myfile, line);
				stringstream s(line);
				string cord;

				s >> cord;
				x1(0, i) = atof(cord.c_str());
				s >> cord;
				x1(1, i) = atof(cord.c_str());

				s >> cord;
				x2(0, i) = atof(cord.c_str());
				s >> cord;
				x2(1, i) = atof(cord.c_str());

			}

			myfile.close();

		}
	}

	// Call the reconstruction function

	vector < Mat_<double> > points2d;
	points2d.push_back(x1);
	points2d.push_back(x2);
	Mat_<double> points3d_estimated;
	vector < Mat > Ps_estimated;

	reconstruct(points2d, Ps_estimated, points3d_estimated, is_projective,
			has_outliers);

	// Print output

	cout << endl;
	cout << "Projection Matrix of View 1: " << endl;
	cout << "============================ " << endl;
	cout << Ps_estimated[0] << endl << endl;
	cout << "Projection Matrix of View 1: " << endl;
	cout << "============================ " << endl;
	cout << Ps_estimated[1] << endl << endl;
	cout << "Reconstructed 3D points: " << endl;
	cout << "======================== " << endl;
	cout << points3d_estimated << endl;

	// Display 3D points using Qt widget

	// Create the structure
	vector<Vec3f> struct_coords;
	for (int i = 0; i < npts; ++i) {
		Vec3f point3d((float) points3d_estimated(0, i),
				(float) points3d_estimated(1, i),
				(float) points3d_estimated(2, i));
		struct_coords.push_back(point3d);
	}

	// Qt stuff
	QApplication app(argc, argv);
	GLWidget window;
	window.AddNewStructure(struct_coords);
	window.resize(800, 600);
	window.show();
	return app.exec();

}
