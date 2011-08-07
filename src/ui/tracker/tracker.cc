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

#include "ui/tracker/tracker.h"
#include "ui/tracker/scene.h"
#include "ui/tracker/gl.h"

#include "libmv/base/vector.h"
#include "libmv/tracking/sad.h"

using libmv::Marker;
using libmv::vector;

#include <QMouseEvent>
#include <QFileInfo>
#include <QDebug>
#include <QTime>

#if (QT_VERSION < QT_VERSION_CHECK(4, 7, 0))
#define constBits bits
#endif

typedef unsigned char ubyte;

Tracker::Tracker(libmv::CameraIntrinsics* intrinsics)
  : intrinsics_(intrinsics), scene_(0), undistort_(false),
    current_image_(0), active_track_(-1), dragged_(false) {
  setMinimumHeight(64);
}

void Tracker::Load(QString path) {
  QFile file(path + (QFileInfo(path).isDir()?"/":".") + "tracks");
  if( file.open(QFile::ReadOnly) ) {
    Marker marker;
    while(file.read((char*)&marker,sizeof(Marker))>0) {
      Insert(marker.image, marker.track, marker.x, marker.y);
      // Select all tracks with markers visible on first frame
      if(marker.image==0) selected_tracks_ << marker.track;
    }
  }
  emit trackChanged(selected_tracks_);
}

void Tracker::Save(QString path) {
  QFile file(path + (QFileInfo(path).isDir()?"/":".") + "tracks");
  if (file.open(QFile::WriteOnly | QIODevice::Truncate)) {
    vector<Marker> markers = AllMarkers();
    file.write(reinterpret_cast<char *>(markers.data()),
               markers.size() * sizeof(Marker));
  }
}

inline float sqr(float x) { return x*x; }
void Tracker::SetImage(int id, QImage image) {
  makeCurrent();
  current_image_ = id;
  if(undistort_) {
    QTime time; time.start();
    QImage correct(image.width(),image.height(),QImage::Format_Indexed8);
    intrinsics_->Undistort(image.constBits(), correct.bits(), image.width(), image.height(), 1);
    qDebug() << QString("%1x%2 image warped in %3 ms")
                .arg(image.width()).arg(image.height()).arg(time.elapsed());
    image_.upload(correct);
  } else {
    image_.upload(image);
  }
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::SetUndistort(bool undistort) {
  undistort_ = undistort;
}

void Tracker::SetOverlay(Scene* scene) {
  scene_ = scene;
}

// Track active trackers from the previous image into next one.
void Tracker::Track(int previous, int next, QImage old_image, QImage new_image) {
  QTime time; time.start();
  Q_ASSERT(old_image.size() == new_image.size() && old_image.depth() == 8);
  int width = old_image.width(), height = old_image.height(),
      stride = old_image.bytesPerLine();
  vector<Marker> previous_markers = MarkersInImage(previous);
  for (int i = 0; i < previous_markers.size(); i++) {
    const Marker &marker = previous_markers[i];
    if (!selected_tracks_.contains(marker.track)) {
      continue;
    }
    float x = marker.x, y = marker.y;
    if( x < kPatternSize || y < kPatternSize ||
        x >= width-kPatternSize || y >= height-kPatternSize ) {
      continue;
    }

    int ix = x, iy = y;
    int x0 = qMax( ix - kSearchSize/2, 0 );
    int y0 = qMax( iy - kSearchSize/2, 0 );
    int x1 = qMin( x0+kSearchSize, width  );
    int y1 = qMin( y0+kSearchSize, height );
    int w = x1-x0, h = y1-y0;

    const ubyte *old_data = old_image.constBits();
    ubyte pattern[16*16];
    libmv::SamplePattern(old_data,stride,x,y,pattern);

    x -= x0, y -= y0;
    libmv::Track(pattern, new_image.constBits()+y0*stride+x0, stride, w, h, &x, &y);
    x += x0, y += y0;
    Insert(next, marker.track, x, y);
  }
  last_frame = next;
  qDebug() << previous_markers.size() <<"markers in" << time.elapsed() << "ms";
}

void Tracker::select(QVector<int> tracks) {
  selected_tracks_ = tracks;
  upload();
}

void Tracker::deleteSelectedMarkers() {
  foreach (int track, selected_tracks_) {
    RemoveMarker(current_image_, track);
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::deleteSelectedTracks() {
  foreach (int track, selected_tracks_) {
    RemoveMarkersForTrack(track);
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::DrawMarker(const libmv::Marker marker, QVector<vec2> *lines) {
  vec2 center = vec2(marker.x, marker.y);
  vec2 quad[] = { vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, 1) };
  for (int i = 0; i < 4; i++) {
    *lines << center+(kSearchSize/2)*quad[i];
    *lines << center+(kSearchSize/2)*quad[(i+1)%4];
  }
  for (int i = 0; i < 4; i++) {
    *lines << center+(kPatternSize/2)*quad[i];
    *lines << center+(kPatternSize/2)*quad[(i+1)%4];
  }
}

bool compare_image(const Marker &a, const Marker &b) {
    return a.image < b.image;
}

void Tracker::upload() {
  makeCurrent();
  vector<Marker> markers = MarkersInImage(current_image_);
  QVector<vec2> lines;
  lines.reserve(markers.size()*8);
  for (int i = 0; i < markers.size(); i++) {
    const Marker &marker = markers[i];
    DrawMarker(marker, &lines);
    vector<Marker> track = MarkersForTrack(marker.track);
    qSort(track.begin(), track.end(), compare_image);
    for (int i = 0; i < track.size()-1; i++) {
      lines << vec2(track[i].x, track[i].y) << vec2(track[i+1].x, track[i+1].y);
    }
  }
  foreach (int track, selected_tracks_) {
    Marker marker = MarkerInImageForTrack(current_image_, track);
    if (marker.image < 0) {
      selected_tracks_.remove(selected_tracks_.indexOf(track));
      continue;
    }
    DrawMarker(marker, &lines);
    DrawMarker(marker, &lines);
    DrawMarker(marker, &lines);
  }
  markers_.primitiveType = 2;
  markers_.upload(lines.constData(), lines.count(), sizeof(vec2));
  update();
}

void Tracker::Render(int x, int y, int w, int h, int image, int track) {
  glBindWindow(x, y, w, h, false);
  glDisableBlend();
  static GLShader image_shader;
  if (!image_shader.id) {
    image_shader.compile(glsl("vertex image"), glsl("fragment image"));
  }
  image_shader.bind();
  image_shader["image"] = 0;
  image_.bind(0);
  mat4 transform;
  if (image >= 0 && track >= 0) {
    Marker marker = MarkerInImageForTrack(image, track);
    vec2 center(marker.x, marker.y);
    vec2 min = (center-kSearchSize/2);
    vec2 max = (center+kSearchSize/2);
    vec2 size( image_.width, image_.height );
    glQuad(vec4(-1, 1, min/size), vec4(1, -1, max/size));
    transform.scale(vec3(1, -1, 0));
    transform.translate(vec3(-1, -1, 0));
    transform.scale(vec3(2/(max-min), 1));
    transform.translate(vec3(-min, 0));
  } else {
    float width = 0, height = 0;
    int W = intrinsics_->image_width(), H = intrinsics_->image_height();
    if (W*h > H*w) {
      width = 1;
      height = static_cast<float>(H*w)/(W*h);
    } else {
      height = 1;
      width = static_cast<float>(W*h)/(H*w);
    }
    glQuad(vec4(-width, -height, 0, 1), vec4(width, height, 1, 0));
    //if (scene_ && scene_->isVisible()) scene_->Render(w, h, current_image_);
    W = image_.width, H = image_.height;
    transform.scale(vec3(2*width/W, -2*height/H, 1));
    transform.translate(vec3(-W/2, -H/2, 0));
    transform_ = transform;
  }
  static GLShader marker_shader;
  if (!marker_shader.id) {
    marker_shader.compile(glsl("vertex transform marker"),
                          glsl("fragment transform marker"));
  }
  marker_shader.bind();
  marker_shader["transform"] = transform;
  markers_.bind();
  markers_.bindAttribute(&marker_shader, "position", 2);
  glAdditiveBlendMode();
  markers_.draw();
}

void Tracker::paintGL() {
  glBindWindow(0, 0, width(), height(), true);
  Render(0, 0, width(), height());
}

void Tracker::mousePressEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,
                                       1-2.0*e->y()/height());
  last_position_ = pos;
  vector<Marker> markers = MarkersInImage(current_image_);
  for (int i = 0; i < markers.size(); i++) {
    const Marker &marker = markers[i];
    vec2 center = vec2(marker.x, marker.y);
    if (pos > center-kSearchSize/2 && pos < center+kSearchSize/2) {
      active_track_ = marker.track;
      return;
    }
  }
  int new_track = MaxTrack() + 1;
  Insert(current_image_, new_track, pos.x, pos.y);
  selected_tracks_ << new_track;
  active_track_ = new_track;
  emit trackChanged(selected_tracks_);
  upload();
}

void Tracker::mouseMoveEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,
                                       1-2.0*e->y()/height());
  vec2 delta = pos-last_position_;
  // FIXME: a reference would avoid duplicate lookup
  Marker marker = MarkerInImageForTrack(current_image_, active_track_);
  marker.x += delta.x;
  marker.y += delta.y;
  Insert(current_image_, active_track_, marker.x, marker.y);
  upload();
  last_position_ = pos;
  dragged_ = true;
  emit trackChanged(selected_tracks_);
}

void Tracker::mouseReleaseEvent(QMouseEvent */*event*/) {
  if (!dragged_ && active_track_ >= 0) {
    if (selected_tracks_.contains(active_track_)) {
      selected_tracks_.remove(selected_tracks_.indexOf(active_track_));
    } else {
      selected_tracks_ << active_track_;
    }
    emit trackChanged(selected_tracks_);
  }
  active_track_ = -1;
  dragged_ = false;
  upload();
}

