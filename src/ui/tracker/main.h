/****************************************************************************
**
** Copyright (c) 2011 libmv authors.
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to
** deal in the Software without restriction, including without limitation the
** rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in
** all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
** IN THE SOFTWARE.
**
****************************************************************************/

#ifndef UI_TRACKER_MAIN_H_
#define UI_TRACKER_MAIN_H_

#include <QMainWindow>
#include <QGridLayout>
#include <QSpinBox>
#include <QSlider>
#include <QTimer>

class Clip;
class Calibration;
class Tracker;
class Zoom;
class Scene;

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow();
  ~MainWindow();

 public slots:
  void open();
  void open(QStringList);
  void seek(int);
  void stop();
  void first();
  void previous();
  void next();
  void last();
  void toggleTracking(bool);
  void toggleBackward(bool);
  void toggleForward(bool);
  void toggleUndistort(bool);
  void detect();
#ifdef RECONSTRUCTION
  void solve();
#endif

 private:
  QString path_;
  Clip *clip_;
  Calibration* calibration_;
  Tracker *tracker_;
  Zoom *zoom_;
  Scene *scene_;

  QToolBar* toolbar_;

  //-> Player : Clip
  QAction *backward_action_;
  QAction *forward_action_;
  QSpinBox spinbox_;
  QSlider slider_;
  QTimer previous_timer_;
  QTimer next_timer_;
  int current_frame_;

  QAction *track_action_;
  QAction *zoom_action_;

};
#endif

