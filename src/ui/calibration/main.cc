// Copyright (c) 2011 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include "main.h"

#include "opencv2/highgui/highgui.hpp" //cvLoadImage
#include "opencv2/imgproc/imgproc_c.h" //cvFindCornerSubPix
#include "opencv2/calib3d/calib3d.hpp" //cvFindChessboardCorners

#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QCompleter>
#include <QFileSystemModel>
#include <QSettings>
#include <QPainter>
#include <QTimer>

Image::Image(QString path) : path(path), distorted_image(path), correct(false) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

QSize Image::sizeHint() const {
  return distorted_image.size() / (distorted_image.height()>540 ? 2 : 1);
}

void Image::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
  p.drawPixmap(rect(), correct ? corrected_image : distorted_image);
  if (!distorted_corners.isEmpty()) {
    p.scale((float)width()/distorted_image.width(),
            (float)height()/distorted_image.height());
    p.setPen(Qt::green);
    p.drawPolyline( correct ? corrected_corners : distorted_corners);
  }
}

MainWindow::MainWindow() : currentImage(0), current(0) {
  setWindowTitle("Camera Calibration");
  hbox = new QHBoxLayout(this);
  QFormLayout* side = new QFormLayout();
  side->setRowWrapPolicy(QFormLayout::WrapLongRows);
  hbox->addLayout(side);
  side->addRow("Source Image Folder", &source);
  QFileSystemModel* model = new QFileSystemModel;
  model->setRootPath(QDir::rootPath());
  source.setCompleter(new QCompleter(model));
  source.setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::Fixed);
  source.setMinimumWidth(256);
  connect(&source, SIGNAL(textChanged(QString)), SLOT(open(QString)));
  side->addRow("Loaded Images", &list);
  list.setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  connect(&list, SIGNAL(currentRowChanged(int)), SLOT(showImage(int)));
  side->addRow("Grid Width", &width);
  width.setRange(3, 256);
  width.setValue(QSettings().value("width", 3).toInt());
  connect(&width, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  side->addRow("Grid Height", &height);
  height.setRange(3, 256);
  height.setValue(QSettings().value("height", 3).toInt());
  connect(&height, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  side->addRow("Square Size", &size);
  size.setValue(QSettings().value("size", 1).toFloat());
  connect(&size, SIGNAL(valueChanged(double)), SLOT(calibrate()));
  side->addRow("Undistort",&correct);
  correct.setEnabled(false);
  connect(&correct, SIGNAL(stateChanged(int)), SLOT(toggleDistort()));
  side->addRow("Focal Length",&focalLength);
  side->addRow("Principal Point",&principalPoint);
  side->addRow("K1",&radialDistortion[0]);
  side->addRow("K2",&radialDistortion[1]);
  side->addRow("K3",&radialDistortion[2]);
}
MainWindow::~MainWindow() {
  QSettings().setValue("width", width.value());
  QSettings().setValue("height", height.value());
  QSettings().setValue("size", size.value());
}

void MainWindow::open(QString folder) {
  list.clear();
  currentImage=0;
  foreach (Image* image, images) {
    delete image;
  }
  images.clear();
  current=0;
  if (folder.isEmpty() || !QDir(folder).exists()) return;
  foreach (QString file, QDir(folder).entryList(QStringList("*.jpg") << "*.png",
                                                QDir::Files, QDir::Name)) {
    list.addItem( file );
    QString path = QDir(folder).filePath(file);
    Image* image = new Image( path );
    image->hide();
    images << image;
  }
  if(images.isEmpty()) return;
  source.setText(QDir(folder).canonicalPath());
  list.setCurrentRow(0);
  calibrate();
}

void MainWindow::showImage(int index) {
  if (currentImage) {
    currentImage->hide();
    hbox->removeWidget(currentImage);
  }
  if(index < 0) return;
  currentImage = images[index];
  hbox->addWidget(currentImage);
  if (correct.isChecked()) {
    currentImage->correct=true;
    if(currentImage->corrected_image.isNull()) {
      IplImage* image = cvLoadImage(currentImage->path.toAscii(), 0);
      IplImage* correct = cvCloneImage( image );
      CvMat camera_matrix = cvMat( 3, 3, CV_64F, camera );
      CvMat distortion_coefficients = cvMat( 1, 4, CV_64F, coefficients );
      cvUndistort2( image, correct, &camera_matrix, &distortion_coefficients );
      currentImage->corrected_image = QPixmap::fromImage(
            QImage((uchar*)correct->imageData, correct->width, correct->height,
                   correct->widthStep, QImage::Format_Indexed8));
      cvReleaseImage( &image );
      cvReleaseImage( &correct );
    }
  } else {
    currentImage->correct=false;
  }
  currentImage->show();
}

void MainWindow::calibrate() {
  for(int i = 0; i < list.count(); i++) {
    list.item(i)->setForeground(QBrush(Qt::black));
  }
  current = 0;
  QTimer::singleShot(50, this, SLOT(process()));
}

void MainWindow::process() {
  CvSize board = { width.value(), height.value() };
  if (current >= images.count()) {
    int point_count = board.width*board.height;
    int image_count = 0;
    foreach (Image* image, images) {
      if(image->distorted_corners.size() == point_count) {
        image_count++;
      }
    }
    CvMat* point_counts = cvCreateMat( 1, image_count, CV_32SC1 );
    cvSet( point_counts, cvScalar(point_count) );

    CvMat* image_points = cvCreateMat( 1, image_count*point_count, CV_32FC2 );
    CvMat* object_points = cvCreateMat( 1, image_count*point_count, CV_32FC3 );
    CvPoint2D32f* image_point = ((CvPoint2D32f*)image_points->data.fl);
    CvPoint3D32f* object_point = ((CvPoint3D32f*)object_points->data.fl);
    for (int i = 0; i < image_count; i++) {
      if (images[i]->distorted_corners.size() != point_count) continue;
      for (int j = 0; j < board.height; j++) {
        for (int k = 0; k < board.width; k++) {
          *object_point++ = cvPoint3D32f(j*size.value(), k*size.value(), 0);
          QPointF point = images[i]->distorted_corners[j*board.width+k];
          *image_point++ = cvPoint2D32f(point.x(),point.y());
        }
      }
    }

    CvMat camera_matrix = cvMat( 3, 3, CV_64F, camera );
    CvMat distortion_coefficients = cvMat( 1, 4, CV_64F, coefficients );
    cvSetZero( &camera_matrix );
    cvSetZero( &distortion_coefficients );

    CvMat* extrinsics = cvCreateMat( image_count, 6, CV_32FC1 );
    CvMat rotation_vectors, translation_vectors;
    cvGetCols( extrinsics, &rotation_vectors, 0, 3 );
    cvGetCols( extrinsics, &translation_vectors, 3, 6 );

    CvSize size = { images.first()->width(), images.first()->height() };
    cvCalibrateCamera2( object_points, image_points, point_counts,
                        size, &camera_matrix, &distortion_coefficients,
                        &rotation_vectors, &translation_vectors );

    CvMat* correct_points = cvCreateMat( 1, image_count*point_count, CV_32FC2 );
    cvUndistortPoints(image_points, correct_points, &camera_matrix, &distortion_coefficients,0,&camera_matrix);
    CvPoint2D32f* correct_point = ((CvPoint2D32f*)correct_points->data.fl);
    for (int i = 0; i < image_count; i++) {
      if (images[i]->distorted_corners.size() != point_count) continue;
      images[i]->corrected_corners.clear();
      for (int j = 0 ; j < board.width*board.height ; j++) {
        CvPoint2D32f point = *correct_point++;
        images[i]->corrected_corners << QPointF(point.x,point.y);
      }
    }

    cvReleaseMat( &object_points );
    cvReleaseMat( &image_points );
    cvReleaseMat( &point_counts );

    focalLength.setText(QString("%1 x %2").arg(camera[0]).arg(camera[4]));
    principalPoint.setText(QString("%1 x %2").arg(camera[2]).arg(camera[5]));
    radialDistortion[0].setText(QString::number(coefficients[0]));
    radialDistortion[1].setText(QString::number(coefficients[1]));
    radialDistortion[2].setText(QString::number(coefficients[2]));

    QFile target(QDir(source.text()).filePath("camera.xml"));
    target.open(QFile::WriteOnly);
    target.write(QString("<lens FocalLengthX='%1' FocalLengthY='%2'"
                         " PrincipalPointX='%3' PrincipalPointY='%4'"
                         " k1='%5' k2='%6' k3='%7' />").arg(camera[0])
                 .arg(camera[4]).arg(camera[2]).arg(camera[5])
                 .arg(coefficients[0]).arg(coefficients[1]).arg(coefficients[2])
                 .toAscii());
    correct.setEnabled(true);
    return;
  }
  Image* image = images[current];
  image->distorted_corners.clear();
  IplImage* iplImage = cvLoadImage(image->path.toAscii(), 0);
  CvPoint2D32f corners[board.width*board.height];
  bool found = false;
  if (cvCheckChessboard(iplImage, board) == 1) {
    int count = 0;
    found = cvFindChessboardCorners(iplImage, board, corners, &count );
    if (found) {
      cvFindCornerSubPix(iplImage, corners, count, cvSize(11,11), cvSize(-1,-1),
                         cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
      for (int i = 0 ; i < board.width*board.height ; i++) {
        image->distorted_corners << QPointF(corners[i].x, corners[i].y);
      }
      image->update();
    }
  }
  cvReleaseImage( &iplImage );
  list.item(current)->setForeground(QBrush(found ? Qt::green : Qt::red));
  current++;
  QTimer::singleShot(50, this, SLOT(process()));  // Voluntary preemption
}

void MainWindow::toggleDistort() {
  showImage(list.currentRow());
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setOrganizationName("libmv");
  app.setApplicationName("calibration");
  MainWindow window;
  window.show();
  window.open(app.arguments().value(1));
  return app.exec();
}
