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

#ifndef UI_CALIBRATION_MAIN_H_
#define UI_CALIBRATION_MAIN_H_

#include <QHBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QListWidget>
#include <QCheckBox>
#include <QSpinBox>
#include <QWidget>
#include <QLabel>

struct Image {
  Image() {}
  Image(QImage image) : image(image), distorted_pixmap(QPixmap::fromImage(image)) {}
  QImage image;
  QVector<QPointF> distorted_corners;
  QVector<QPointF> corrected_corners;
  QPixmap distorted_pixmap;
  QPixmap corrected_pixmap;
};

class View : public QWidget {
public:
  View();
  QSize sizeHint() const;
  void setImage(Image image,bool correct=false);

protected:
  void paintEvent(QPaintEvent*);

private:
  QVector<QPointF> corners;
  QPixmap pixmap;
};

#include <QDialog>
#include <QFileSystemModel>
#include <QTreeView>

class FileDialog : public QDialog {
    Q_OBJECT
public:
    FileDialog(QWidget*,QString);
    QStringList selectedFiles();
private:
    QFileSystemModel fileSystem;
    QTreeView view;
};

class MainWindow : public QWidget {
  Q_OBJECT
 public:
  MainWindow();
  ~MainWindow();

 public slots:
  void open();
  void open(QStringList);
  void addImage();
  void stop();
  void showImage(int);
  void calibrate();
  void process();
  void toggleDistort();

 private:
  QHBoxLayout hbox;
  QFormLayout side;
  QPushButton source;
  QListWidget list;
  QSpinBox width;
  QSpinBox height;
  QDoubleSpinBox size;
  QCheckBox correct;
  QLabel focalLength;
  QLabel principalPoint;
  QLabel radialDistortion[3];
  QLabel tangentialDistortion[2];
  View view;

  QString path;
  QList<Image> images;
  Image preview;
  int current;
  bool play;

  double camera[9];
  double coefficients[5];
};
#endif

