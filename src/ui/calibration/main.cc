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
#include <QSettings>
#include <QPainter>
#include <QTimer>
#include <QTime>
#include <QDebug>

Image::Image(QString path) : path(path), pixmap(path) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

QSize Image::sizeHint() const {
  return pixmap.size();
}

void Image::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.drawPixmap(rect(), pixmap);
  p.setPen(Qt::green);
  if (!corners.isEmpty()) {
    p.drawPolyline(corners);
  }
}

MainWindow::MainWindow() : currentImage(0), current(0) {
  setWindowTitle("Camera Calibration");
  hbox = new QHBoxLayout(this);
  QFormLayout* side = new QFormLayout();
  side->setRowWrapPolicy(QFormLayout::WrapAllRows);
  hbox->addLayout(side);
  side->addRow("Source image folder", &source);
  side->addRow("Loaded images", &list);
  list.setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  side->addRow("Number of corners along horizontal", &width);
  width.setRange(3, 256);
  width.setValue(QSettings().value("width", 3).toInt());
  side->addRow("Number of corners along vertical", &height);
  height.setRange(3, 256);
  height.setValue(QSettings().value("height", 3).toInt());
  side->addRow("Real distance between corners", &size);
  size.setValue(QSettings().value("size", 1).toFloat());
  connect(&list, SIGNAL(currentRowChanged(int)), SLOT(showImage(int)));
  connect(&width, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  connect(&height, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  connect(&size, SIGNAL(valueChanged(double)), SLOT(calibrate()));
}
MainWindow::~MainWindow() {
  QSettings().setValue("width", width.value());
  QSettings().setValue("height", height.value());
  QSettings().setValue("size", size.value());
}

void MainWindow::open() {
  open(QFileDialog::getExistingDirectory(this, "Select image folder"));
}

void MainWindow::open(QString folder) {
  foreach (Image* image, images) {
    delete image;
  }
  if (folder.isEmpty() || !QDir(folder).exists()) return;
  source.setText(QDir(folder).canonicalPath());
  foreach (QString file, QDir(folder).entryList(QStringList("*.jpg") << "*.png",
                                                QDir::Files, QDir::Name)) {
    list.addItem( file );
    QString path = QDir(folder).filePath(file);
    Image* image = new Image( path );
    image->hide();
    images << image;
  }
  list.setCurrentRow(0);
  calibrate();
}

void MainWindow::showImage(int index) {
  if (currentImage) {
    currentImage->hide();
    hbox->removeWidget(currentImage);
  }
  currentImage = images[index];
  hbox->addWidget(currentImage);
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
  if (current == images.count()) {
    /*cvReleaseMat( &extr_params );
    cvReleaseMat( &reproj_errs );
    int code = run_calibration( image_points_seq, img_size, board_size,
                                square_size, aspect_ratio, flags, &camera, &dist_coeffs, &extr_params,
                                &reproj_errs, &avg_reproj_err );
    // save camera parameters in any case, to catch Inf's/NaN's
    save_camera_params( out_filename, image_count, img_size,
                        board_size, square_size, aspect_ratio, flags,
                        &camera, &dist_coeffs, write_extrinsics ? extr_params : 0,
                        write_points ? image_points_seq : 0, reproj_errs, avg_reproj_err );*/
    return;
  }
  QTime time; time.start();
  Image* image = images[current];
  image->corners.clear();
  IplImage* iplImage = cvLoadImage(image->path.toAscii(), 0);
  CvSize board = { width.value(), height.value() };
  CvPoint2D32f corners[board.width*board.height];
  bool found = false;
  if (cvCheckChessboard(iplImage, board) == 1) {
    int count = 0;
    found = cvFindChessboardCorners(iplImage, board, corners, &count );
    if(found) {
      cvFindCornerSubPix(iplImage, corners, count, cvSize(11,11), cvSize(-1,-1),
                         cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
      for (int i = 0 ; i < board.width*board.height ; i++) {
        image->corners << QPointF(corners[i].x, corners[i].y);
      }
      image->update();
    }
  }
  qDebug() << image->path << ":" << (found?"board found":"board not found")
           << "in" << time.elapsed() << "ms";
  list.item(current)->setForeground(QBrush(found ? Qt::green : Qt::red));
  current++;
  QTimer::singleShot(50, this, SLOT(process()));  // Voluntary preemption
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

