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

#ifndef UI_TRACKER_TRACKER_H_
#define UI_TRACKER_TRACKER_H_

#include <QMap>
#include <QGLWidget>
#include "ui/tracker/gl.h"

//TODO: Qt Tracker should be independent from libmv to be able to use new lens distortion API
#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/numeric/numeric.h"

// TODO(MatthiasF): custom pattern/search size
static const double kSigma = 0.9;
static const int kHalfPatternSize = 5;
static const int kPatternSize = kHalfPatternSize * 2;
static const int kHalfSearchSize = 31;
static const int kSearchSize = kHalfSearchSize * 2;
static const int kMaxIterations = 100;
static const int kMinimumCorrelation = 0.76;

// KLT tracker settings
//static const int kPyramidLevelCount = 2;
//static const int kHalfSearchSize = kHalfPatternSize << kPyramidLevelCount;

class Scene;

class Tracker : public QGLWidget {
  Q_OBJECT
 public:
  Tracker(libmv::CameraIntrinsics* intrinsics);

  void Load(QString path);
  void Save(QString path);
  void SetImage(int id, QImage image);
  void SetUndistort(bool undistort);
  void SetOverlay(Scene* scene);
  void AddTrack(float x, float y);
  void Track(int previous, int next, QImage old, QImage search);
  void Render(int x, int y, int w, int h, int image=-1, int track=-1);

 public slots:
  void select(QVector<int>);
  void deleteSelectedMarkers();
  void deleteSelectedTracks();
  void upload();

 signals:
  void trackChanged(QVector<int> tracks);

 protected:
  void paintGL();
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);

 private:
  libmv::CameraIntrinsics* intrinsics_;
  Scene* scene_;
  int last_frame;
  QVector< QVector<vec2> > tracks; //[track][image]

  bool undistort_;
  QImage image_;
  GLTexture texture_;
  mat4 transform_;
  GLBuffer markers_;
  int current_;
  QVector<int> selected_tracks_;
  vec2 delta_;
  int active_track_;
  bool dragged_;
};

#endif
