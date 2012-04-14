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

#include "libmv/tracking/region_tracker.h"
#include "libmv/tracking/esm_region_tracker.h"
#include "libmv/tracking/brute_region_tracker.h"
#include "libmv/tracking/hybrid_region_tracker.h"

#include <QMouseEvent>
#include <QFileInfo>
#include <QDebug>
#include <QTime>

#if (QT_VERSION < QT_VERSION_CHECK(4, 7, 0))
#define constBits bits
#endif

QDataStream& operator<<(QDataStream& s, const vec2& v) { s.writeRawData((char*)&v,sizeof(v)); return s; }
QDataStream& operator>>(QDataStream& s, vec2& v) { s.readRawData((char*)&v,sizeof(v)); return s; }
QDebug operator <<(QDebug d, vec2& v) {
  return d.nospace()<<v.x<<", "<<v.y;
}

// Copy the region starting at x0, y0 with width w, h into region.
// If the region asked for is outside the image border, the marker is removed.
// Returns false if the region leave the image.
bool CopyRegionFromQImage(QImage image,
                          int w, int h,
                          int x0, int y0,
                          libmv::FloatImage *region) {
  Q_ASSERT(image.depth() == 8);
  const unsigned char *data = image.constBits();
  int width = image.width();
  int height = image.height();

  // Return if the region leave the image.
  if (x0 < 0 || y0 < 0 || x0+w >= width || y0+h >= height) return false;

  // Copy the region.
  region->resize(h, w);
  float* dst = region->Data();
  for (int y = y0; y < y0 + h; ++y) {
    for (int x = x0; x < x0 + w; ++x) {
      *dst++ = (float)data[y * width + x] / 255.0;
    }
  }
  return true;
}

libmv::RegionTracker *NewHybridRegionTracker(int max_iterations, int half_window_size, double minimum_correlation)
{
  libmv::EsmRegionTracker *esm_region_tracker = new libmv::EsmRegionTracker;
  esm_region_tracker->half_window_size = half_window_size;
  esm_region_tracker->max_iterations = max_iterations;
  esm_region_tracker->min_determinant = 1e-4;
  esm_region_tracker->minimum_correlation = minimum_correlation;

  libmv::BruteRegionTracker *brute_region_tracker = new libmv::BruteRegionTracker;
  brute_region_tracker->half_window_size = half_window_size;

  // do not use correlation check for brute checker itself
  // this check will happen in esm tracker
  brute_region_tracker->minimum_correlation = 0.0;

  libmv::HybridRegionTracker *hybrid_region_tracker =
  new libmv::HybridRegionTracker(brute_region_tracker, esm_region_tracker);

  return hybrid_region_tracker;
}

Tracker::Tracker(libmv::CameraIntrinsics* intrinsics) : intrinsics_(intrinsics),
    scene_(0), undistort_(false),
    current_(0), active_track_(-1), dragged_(false) {
  setMinimumHeight(64);
}

void Tracker::Load(QString path) {
  QFile file(path + (QFileInfo(path).isDir()?"/":".") + "tracks");
  if (file.open(QFile::ReadOnly)) {
    QDataStream s(&file);
    s >> tracks;
  }
  emit trackChanged(selected_tracks_);
}

void Tracker::Save(QString path) {
  QFile file(path + (QFileInfo(path).isDir()?"/":".") + "tracks");
  if (file.open(QFile::WriteOnly|QFile::Truncate)) {
    QDataStream s(&file);
    s << tracks;
  }
}

inline float sqr(float x) { return x*x; }
void Tracker::SetImage(int id, QImage image) {
  makeCurrent();
  current_ = id;
  image_ = image;
#ifdef LENS_DISTORTION
  if(undistort_) {
    QTime time; time.start();
    QImage correct(image.width(),image.height(),QImage::Format_Indexed8);
    intrinsics_->Undistort(image.constBits(), correct.bits(), image.width(), image.height(), 1);
    qDebug() << QString("%1x%2 image warped in %3 ms")
                .arg(image.width()).arg(image.height()).arg(time.elapsed());
    image_.upload(correct);
  } else
#endif
  {
    texture_.upload(image);
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

void Tracker::AddTrack(float x, float y) {
  int new_track = tracks.count();
  vec2 marker = vec2(x, y);
  tracks << QVector<vec2>(current_+1,marker);
  selected_tracks_ << new_track;
  active_track_ = new_track;
  emit trackChanged(selected_tracks_);
  upload();
}

void Tracker::Track(int previous, int next, QImage old, QImage search) {
  QTime time; time.start();
  int width = search.width(), height = search.height();
  libmv::RegionTracker *region_tracker;
  region_tracker = NewHybridRegionTracker(kMaxIterations, kHalfPatternSize, kMinimumCorrelation);
  foreach(int index, selected_tracks_) {
    QVector<vec2>& track = tracks[index];
    vec2 marker = track[previous];

    // Stop tracking near borders
    double x = marker.x, y = marker.y;
    if( x <= kHalfPatternSize || y <= kHalfPatternSize ||
        x >= width-kHalfPatternSize-1 || y >= height-kHalfPatternSize-1 ) continue;

    int x0 = marker.x - kHalfSearchSize;
    int y0 = marker.y - kHalfSearchSize;

    if(track.count()<=next) track.resize(next+1);

    libmv::FloatImage old_patch, new_patch;

    if (!CopyRegionFromQImage(old, kSearchSize, kSearchSize, x0, y0, &old_patch) ||
        !CopyRegionFromQImage(search, kSearchSize, kSearchSize, x0, y0, &new_patch)) {
      continue;
    }

    double x1 = marker.x - x0, y1 = marker.y - y0;
    region_tracker->Track(old_patch, new_patch, x1, y1, &x1, &y1);
    marker.x = x1 + x0, marker.y = y1 + y0;

    track[next] = marker;
  }
  delete region_tracker;
  last_frame = next;
  qDebug() << selected_tracks_.size() <<"markers in" << time.elapsed() << "ms";
}

void Tracker::select(QVector<int> tracks) {
  selected_tracks_ = tracks;
  upload();
}

void Tracker::deleteSelectedMarkers() {
  foreach (int track, selected_tracks_) {
    tracks[track][current_] = vec2();
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::deleteSelectedTracks() {
  foreach (int track, selected_tracks_) {
    tracks.remove(track);
  }
  selected_tracks_.clear();
  upload();
  emit trackChanged(selected_tracks_);
}

void Tracker::upload() {
  makeCurrent();
  QVector<vec2> lines;
  static const vec2 quad[] = { vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, 1) };
  foreach(QVector<vec2> track, tracks) {
    if (current_ < track.count()) {
      const vec2 marker = track[current_];
      if(marker.x && marker.y) {
        for (int i = 0; i < 4; i++) {
          lines << marker+((kPatternSize/2)*quad[i]);
          lines << marker+((kPatternSize/2)*quad[(i+1)%4]);
        }
      }
    }
  }
  foreach (int index, selected_tracks_) {
    QVector<vec2> track = tracks[index];
    if (current_ >= track.count()) {
      selected_tracks_.remove(selected_tracks_.indexOf(index));
      continue;
    }
    for (int i = 0; i < track.size()-1; i++) {
      lines << track[i] <<  track[i+1];
    }
    const vec2 marker = track[current_];
    int x = marker.x, y = marker.y;
    int x0 = qMax( x - kSearchSize/2, 0);
    int y0 = qMax( y - kSearchSize/2, 0);
    int x1 = qMin( x + kSearchSize/2, image_.width());
    int y1 = qMin( y + kSearchSize/2, image_.height());
    lines <<vec2(x0,y0)<<vec2(x1,y0) <<vec2(x1,y0)<<vec2(x1,y1) <<vec2(x1,y1)<<vec2(x0,y1) <<vec2(x0,y1)<<vec2(x0,y0);
  }
  markers_.primitiveType = 2;
  markers_.upload(lines.constData(), lines.count(), sizeof(vec2));
  update();
}

void Tracker::Render(int x, int y, int w, int h, int image, int track) {
  glBindWindow(x, y, w, h, false);
  static GLShader image_shader;
  if (!image_shader.id) {
    image_shader.compile(glsl("vertex image"), glsl("fragment image"));
  }
  image_shader.bind();
  image_shader["image"] = 0;
  texture_.bind(0);
  if (image >= 0 && track >= 0) {
    vec2 marker = tracks[track][image]; // reference -> current
    float scale = kPatternSize/2;
    vec2 size( image_.width(), image_.height() );
    vec4 quad[] = { vec4(-1,  1, (marker+(scale*vec2(-1,-1)))/size),
                    vec4( 1,  1, (marker+(scale*vec2(1,-1)))/size),
                    vec4( 1, -1, (marker+(scale*vec2(1,1)))/size),
                    vec4(-1, -1, (marker+(scale*vec2(-1,1)))/size) };
    glQuad(quad);
    return;
  }
  float width = 0, height = 0;
  int W = intrinsics_->image_width(), H = intrinsics_->image_height();
  if (W*h > H*w) {
    width = 1;
    height = float(H*w)/(W*h);
  } else {
    height = 1;
    width = float(W*h)/(H*w);
  }
  vec4 quad[] = { vec4(-width, -height, 0, 1),
                  vec4( width, -height, 1, 1),
                  vec4( width,  height, 1, 0),
                  vec4(-width,  height, 0, 0) };
  glQuad(quad);
  //if (scene_ && scene_->isVisible()) scene_->Render(w, h, current_);
  W = image_.width(), H = image_.height();
  transform_ = mat4();
  transform_.scale(vec3(2*width/W, -2*height/H, 1));
  transform_.translate(vec3(-W/2, -H/2, 0));

  static GLShader marker_shader;
  if (!marker_shader.id) {
    marker_shader.compile(glsl("vertex transform marker"),
                          glsl("fragment transform marker"));
  }
  marker_shader.bind();
  marker_shader["transform"] = transform_;
  markers_.bind();
  markers_.bindAttribute(&marker_shader, "position", 2);
  glSmooth();
  markers_.draw();
  glHard();
}

void Tracker::paintGL() {
  glBindWindow(0, 0, width(), height(), true);
  Render(0, 0, width(), height());
}

void Tracker::mousePressEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,
                                       1-2.0*e->y()/height());
  int i=0;
  foreach(QVector<vec2> track, tracks) {
    vec2 marker = track[current_];
    vec2 center = vec2(marker);
    if (pos > center-kSearchSize/2 && pos < center+kSearchSize/2) {
      active_track_ = i;
      delta_ = pos-vec2(marker);
      return;
    }
    i++;
  }
  AddTrack(pos.x, pos.y);
}

void Tracker::mouseMoveEvent(QMouseEvent* e) {
  vec2 pos = transform_.inverse()*vec2(2.0*e->x()/width()-1,
                                       1-2.0*e->y()/height())+delta_;
  vec2& marker = tracks[active_track_][current_];
  int size = kPatternSize/2;
  marker.x = qBound<float>(size, pos.x, image_.width ()-size-1);
  marker.y = qBound<float>(size, pos.y, image_.height()-size-1);
  upload();
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

