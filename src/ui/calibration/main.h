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

#include <QWidget>
#include <QLineEdit>
#include <QListWidget>
#include <QSpinBox>

class QHBoxLayout;

class Image : public QWidget {
  Q_OBJECT
 public:
  Image(QString path);
  QSize sizeHint() const;

 protected:
  void paintEvent(QPaintEvent*);
 private:
  QPixmap pixmap;
};

class MainWindow : public QWidget {
  Q_OBJECT
 public:
  MainWindow();

 public slots:
  void open();
  void open(QString);
  void showImage(int);

 private:
  QHBoxLayout* hbox;
  QLineEdit source;
  QListWidget list;
  QSpinBox width;
  QSpinBox height;
  QDoubleSpinBox size;
  QList<Image*> images;
  Image* current;
};
#endif

