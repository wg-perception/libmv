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

#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/tracking/klt.h"

// TODO(MatthiasF): custom pattern/search size
static const double kSigma = 0.9;
static const int kHalfPatternSize = 4;
static const int kPyramidLevelCount = 2;
static const int kHalfSearchSize = kHalfPatternSize << kPyramidLevelCount;
static const int kSearchSize = kHalfSearchSize * 2 + 1;

class Scene;

class Tracker : public QGLWidget, public libmv::Tracks {
  Q_OBJECT
 public:
  Tracker(libmv::CameraIntrinsics* intrinsics);

  void Load(QString path);
  void Save(QString path);
  void SetImage(int id, QImage image);
  void SetOverlay(Scene* scene);
  void Track(int previous, int next, QImage old_image, QImage new_image);
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
  void DrawMarker(const libmv::Marker marker, QVector<vec2> *lines);

  libmv::CameraIntrinsics* intrinsics_;
  Scene* scene_;
  int last_frame;
  QMap<int,libmv::Tracker> trackers;

  GLTexture image_;
  mat4 transform_;
  GLBuffer markers_;
  int current_image_;
  QVector<int> selected_tracks_;
  vec2 last_position_;
  int active_track_;
  bool dragged_;
};

#endif
