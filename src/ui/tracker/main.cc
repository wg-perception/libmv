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

#include "ui/tracker/main.h"
#include "ui/tracker/tracker.h"
#include "ui/tracker/scene.h"

#include "libmv/simple_pipeline/initialize_reconstruction.h"
#include "libmv/simple_pipeline/bundle.h"
#include "libmv/simple_pipeline/pipeline.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"

// TODO(MatthiasF): better API for blender integration
#include "libmv/image/convolve.h"
#include "libmv/simple_pipeline/detect.h"

#include <QApplication>
#include <QFileDialog>
#include <QFormLayout>
#include <QDockWidget>
#include <QToolButton>
#include <QSettings>
#include <QToolBar>
#include <QAction>
#include <QMenu>
#include <QTime>

#ifdef USE_FFMPEG
extern "C" {
typedef uint64_t UINT64_C;
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
}
#endif

#include <float.h>

void Clip::Open(QString path) {
  images_.clear();
  QString sep = QFileInfo(path).isDir() ? "/" : ".";
  cache_.setFileName(path+sep+"cache");
  if (cache_.open(QFile::ReadOnly)) {
    QFile meta(path+sep+"meta");
    meta.open(QFile::ReadOnly);
    meta.read((char*)&size_, sizeof(size_));
    int image_size = size_.width()*size_.height();
    images_.resize( cache_.size()/image_size );
    uchar* data = cache_.map(0, cache_.size());
    for(int i = 0; i < images_.count(); i++, data+=image_size) {
      images_[i] = QImage(data,size_.width(),size_.height(),QImage::Format_Indexed8);
    }
    return;
  }
#ifdef USE_FFMPEG
  av_register_all();
  if (QFileInfo(path).isFile()) {
    AVFormatContext* file = 0;
    if(avformat_open_input(&file, path.toUtf8(), 0, 0)) return;
    av_find_stream_info(file);
    int video_stream = 0;
    AVCodecContext* video = 0;
    for (int i = 0; i < (int)file->nb_streams; i++ ) {
        if( file->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
          video_stream = i;
          video = file->streams[i]->codec;
          AVCodec* codec = avcodec_find_decoder(video->codec_id);
          if( codec ) avcodec_open(video, codec);
          break;
        }
    }
    for (AVPacket packet; av_read_frame(file, &packet) >= 0; ) {
      if ( packet.stream_index == video_stream ) {
        AVFrame* frame = avcodec_alloc_frame();
        int complete_frame = 0;
        avcodec_decode_video2(video, frame, &complete_frame, &packet);
        if (complete_frame) {
          // FIXME: Assume planar format
          QImage image(frame->width, frame->height, QImage::Format_Indexed8);
          int w = image.width(), h = image.height(), bytesPerLine = frame->linesize[0];
          uchar* dst = image.bits();
          const uchar* src = frame->data[0];
          for(int y = 0; y < h; y++) {
              memcpy(dst, src, w);
              dst += w; src += bytesPerLine;
          }
          av_free(frame);
          images_ << image;
        }
      }
      av_free_packet(&packet);
    }
    avcodec_close(video);
    av_close_input_file(file);
  } else
#endif
  foreach (QString file, QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                              QDir::Files, QDir::Name)) {
    QImage image(QDir(path).filePath(file));
    if(image.depth() != 8) {
      QImage grayscale(image.width(),image.height(),QImage::Format_Indexed8);
      QRgb *src = (QRgb*)image.constBits();
      uchar *dst = grayscale.bits();
      for(int i = 0; i < grayscale.byteCount(); i++) dst[i] = qGray(src[i]);
      image = grayscale;
    }
    images_ << image;
  }
  size_ = QSize(images_[0].size());
  // Create raw on-disk video cache
  if(Size().width()*Size().height()*Count() < 1024*1024*1024) {
    cache_.open(QFile::WriteOnly|QFile::Truncate);
    foreach(const QImage& image, images_) cache_.write((const char*)image.constBits(),image.byteCount());
    cache_.close();
    QFile meta(path+sep+"meta");
    meta.open(QFile::WriteOnly|QFile::Truncate);
    meta.write((char*)&size_,sizeof(size_));
  }
}

int Clip::Count() {
  return images_.count();
}

QSize Clip::Size() {
  return size_;
}

QImage Clip::Image(int i) {
  return images_[i];
}

MainWindow::MainWindow()
  : clip_(new Clip(this)),
    tracks_(new libmv::Tracks()),
    intrinsics_(new libmv::CameraIntrinsics()),
    reconstruction_(new libmv::Reconstruction()),
    scene_(new Scene(intrinsics_, reconstruction_)),
    tracker_(new Tracker(tracks_, scene_, scene_)),
    current_frame_(-1) {
  setWindowTitle("Tracker");
  setCentralWidget(tracker_);
  connect(tracker_, SIGNAL(trackChanged(QVector<int>)),
          this, SLOT(updateZooms(QVector<int>)));

  QToolBar* toolbar = addToolBar("Main Toolbar");
  toolbar->setObjectName("mainToolbar");

  toolbar->addAction(QIcon(":/open"), "Load Footage", this, SLOT(open()));

  QAction* tracker_action_ = toolbar->addAction(QIcon(":/view-image"),
                                                "Tracker View");
  tracker_action_->setCheckable(true);
  tracker_action_->setChecked(true);
  connect(tracker_action_, SIGNAL(triggered(bool)),
          tracker_, SLOT(setVisible(bool)));

  zoom_action_ = toolbar->addAction(QIcon(":/view-zoom"), "Zoom View");
  zoom_action_->setCheckable(true);
  zoom_action_->setChecked(true);
  connect(zoom_action_, SIGNAL(triggered(bool)), this, SLOT(toggleZoom(bool)));

  QDockWidget* scene_dock = new QDockWidget("Scene View");
  scene_dock->setObjectName("sceneDock");
  addDockWidget(Qt::BottomDockWidgetArea, scene_dock);
  scene_dock->setWidget(scene_);
  scene_dock->toggleViewAction()->setIcon(QIcon(":/view-scene"));
  scene_dock->toggleViewAction()->setChecked(false);
  toolbar->addAction(scene_dock->toggleViewAction());
  connect(scene_, SIGNAL(imageChanged(int)), SLOT(seek(int)));
  connect(scene_, SIGNAL(trackChanged(QVector<int>)),
          tracker_, SLOT(select(QVector<int>)));

  toolbar->addSeparator();

  toolbar->addAction(QIcon(":/detect"), "Detect features",this, SLOT(detect()));

  QToolButton* delete_button = new QToolButton();
  toolbar->addWidget(delete_button);
  QMenu* delete_popup = new QMenu();
  delete_popup->addAction(QIcon(":/delete"),
                          "Delete current marker",
                          tracker_, SLOT(deleteSelectedMarkers()));
  QAction* delete_track = delete_popup->addAction(QIcon(":/delete-row"),
                                                "Delete current track",
                                                tracker_,
                                                SLOT(deleteSelectedTracks()));
  delete_button->setMenu(delete_popup);
  delete_button->setDefaultAction(delete_track);
  delete_button->setPopupMode(QToolButton::MenuButtonPopup);
  connect(delete_popup, SIGNAL(triggered(QAction*)),
          delete_button, SLOT(setDefaultAction(QAction*)));

  track_action_ = toolbar->addAction(QIcon(":/record"), "Track selected markers");
  track_action_->setCheckable(true);
  connect(track_action_, SIGNAL(triggered(bool)), SLOT(toggleTracking(bool)));
  connect(tracker_action_, SIGNAL(triggered(bool)),
          track_action_, SLOT(setVisible(bool)));

  toolbar->addAction(QIcon(":/solve"), "Solve reconstruction",
                     this, SLOT(solve()));
  QAction* add_action = toolbar->addAction(QIcon(":/add"), "Add object",
                                           scene_, SLOT(add()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          add_action, SLOT(setVisible(bool)));

  QAction* link_action = toolbar->addAction(
        QIcon(":/link"), "Link active object to selected bundles",
        scene_, SLOT(link()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          link_action, SLOT(setVisible(bool)));

  toolbar->addSeparator();

  toolbar->addAction(QIcon(":/skip-backward"), "Seek to first frame",
                     this, SLOT(first()));
  toolbar->addAction(QIcon(":/step-backward"), "Step to previous frame",
                     this, SLOT(previous()))->setShortcut(QKeySequence("Left"));
  backward_action_ = toolbar->addAction(QIcon(":/play-backward"),
                                        "Play backwards");
  backward_action_->setCheckable(true);
  connect(backward_action_, SIGNAL(triggered(bool)),
          SLOT(toggleBackward(bool)));
  connect(&previous_timer_, SIGNAL(timeout()), SLOT(previous()));

  toolbar->addWidget(&spinbox_);
  connect(&spinbox_, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  toolbar->addWidget(&slider_);
  slider_.setOrientation(Qt::Horizontal);
  connect(&slider_, SIGNAL(valueChanged(int)), SLOT(seek(int)));

  forward_action_ = toolbar->addAction(QIcon(":/play-forward"),
                                       "Play forwards");
  forward_action_->setCheckable(true);
  connect(forward_action_, SIGNAL(triggered(bool)), SLOT(toggleForward(bool)));
  connect(&next_timer_, SIGNAL(timeout()), SLOT(next()));
  toolbar->addAction(QIcon(":/step-forward"), "Next Frame", this, SLOT(next()))
      ->setShortcut(QKeySequence("Right"));
  toolbar->addAction(QIcon(":/skip-forward"), "Last Frame", this, SLOT(last()));

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());
}
void MainWindow::Save(QString name, QByteArray data) {
  if (path_.isEmpty() || data.isEmpty()) return;
  QFile file(QFileInfo(path_).isDir() ? QDir(path_).filePath(name) : path_+"."+name);
  if (file.open(QFile::WriteOnly | QIODevice::Truncate)) {
    file.write(data);
  }
}
MainWindow::~MainWindow() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
  QSettings().setValue("currentFrame", current_frame_);
  Save("tracks", tracker_->Save());
  //Save("cameras", scene_->SaveCameras());
  //Save("bundles", scene_->SaveBundles());
  //Save("objects", scene_->SaveObjects());
  delete reconstruction_;
  delete tracks_;
}

QByteArray MainWindow::Load(QString name) {
  QFile file(QFileInfo(path_).isDir() ? QDir(path_).filePath(name) : path_+"."+name);
  return file.open(QFile::ReadOnly) ? file.readAll() : QByteArray();
}

struct Parameter {
  const char* name;
  const char* suffix;
  double min;
  double max;
  double value;
};

void MainWindow::open() {
  open(QFileDialog::getOpenFileName(this, "Load Footage"));
  QSize size = clip_->Size();
  QDialog dialog(this);
  dialog.setWindowTitle("Camera Parameters");
  QFormLayout layout(&dialog);

  const int kCount = 4;
  const Parameter parameters[kCount] = {
    {"Focal Length",              "px", 0,  FLT_MAX,       size.width()*2  },
    {"Principal Point (X)",       "px", 0,  size.width(),  size.width()/2  },
    {"Principal Point (Y)",       "px", 0,  size.height(), size.height()/2 },
    {"1st Radial Distortion",     "",  -1,  1,             0               },
  };
  QDoubleSpinBox spinbox[kCount];
  QByteArray data = Load("settings");
  data.reserve(kCount*sizeof(float));
  float* values = reinterpret_cast<float*>(data.data());
  for (int i = 0; i < kCount; i++) {
    Parameter parameter = parameters[i];
    spinbox[i].setSuffix(parameter.suffix);
    spinbox[i].setRange(parameter.min, parameter.max);
    spinbox[i].setDecimals(3);
    if (data.isEmpty()) {
      spinbox[i].setValue(parameter.value);
    } else {
      spinbox[i].setValue(values[i]);
    }
    layout.addRow(parameter.name, &spinbox[i]);
  }
  dialog.exec();
  data.resize(kCount*sizeof(float));
  for (int i = 0; i < kCount; i++) {
    values[i] = spinbox[i].value();
  }
  Save("settings", data);
  intrinsics_->SetFocalLength(values[0]);
  intrinsics_->set_principal_point(values[1],values[2]);
  intrinsics_->set_radial_distortion(values[3], 0, 0);
}

void MainWindow::open(QString path) {
  if (path.isEmpty() || !QFileInfo(path).exists()) return;
  path_ = path;
  setWindowTitle(QString("Tracker - %1").arg(QDir(path).dirName()));

  clip_->Open(path);
  tracker_->Load(Load("tracks"));
  /*foreach(QString path, QDir(path_).entryList(QStringList("*.dae"))) {
    QFile file(path);
    scene_->LoadCOLLADA(&file);
  }*/
  //scene_->LoadCameras(Load("cameras"));
  //scene_->LoadBundles(Load("bundles"));
  //scene_->LoadObjects(Load("objects"));
  //scene_->upload();
  spinbox_.setMaximum(clip_->Count() - 1);
  slider_.setMaximum(clip_->Count() - 1);
  int frame = QSettings().value("currentFrame", 0).toInt();
  if(frame >= 0 && frame < clip_->Count()) {
    seek(frame);
  } else {
    seek(0);
  }
}

void MainWindow::seek(int frame) {
  // Bail out if there's nothing to do.
  if (frame == current_frame_) {
    return;
  }
  if (frame < 0 || frame >= clip_->Count()) {
    stop();
    return;
  }
  // Track only if the shift is between consecutive frames.
  if ( frame > current_frame_ + 1 || frame < current_frame_ - 1 ) {
    track_action_->setChecked(false);
  }
  current_frame_ = frame;

  slider_.setValue(current_frame_);
  spinbox_.setValue(current_frame_);
  tracker_->SetImage(current_frame_, clip_->Image(current_frame_),
                     track_action_->isChecked());
}

void MainWindow::stop() {
  backward_action_->setChecked(false);
  previous_timer_.stop();
  forward_action_->setChecked(false);
  next_timer_.stop();
}

void MainWindow::first() {
  seek(0);
}

void MainWindow::previous() {
  seek(current_frame_ - 1);
}

void MainWindow::next() {
  seek(current_frame_ + 1);
}

void MainWindow::last() {
  seek(clip_->Count() - 1);
}

void MainWindow::toggleTracking(bool track) {
  stop();
  if (track) {
    backward_action_->setText("Track backwards");
    forward_action_->setText("Track forwards");
  } else {
    backward_action_->setText("Play backwards");
    forward_action_->setText("Play forwards");
  }
}

void MainWindow::toggleBackward(bool play) {
  if (play) {
    forward_action_->setChecked(false);
    next_timer_.stop();
    previous_timer_.start(0);
  } else {
    stop();
  }
}

void MainWindow::toggleForward(bool play) {
  if (play) {
    backward_action_->setChecked(false);
    previous_timer_.stop();
    next_timer_.start(0);
  } else {
    stop();
  }
}

void MainWindow::toggleZoom(bool zoom) {
  if (zoom) {
    foreach (QDockWidget* dock, zooms_docks_) {
      dock->show();
    }
  } else {
    foreach (QDockWidget* dock, zooms_docks_) {
      dock->hide();
    }
  }
}

void MainWindow::updateZooms(QVector<int> tracks) {
  if (!zoom_action_->isChecked()) return;
  for (int i = tracks.size(); i < zooms_docks_.size(); i++) {
    removeDockWidget(zooms_docks_[i]);
    delete zooms_docks_[i];
    // zoom widget is owned by dock ?
  }
  for (int i = zooms_docks_.size(); i < tracks.size(); i++) {
    QDockWidget* dock = new QDockWidget();
    dock->setObjectName(QString("zoom%1").arg(tracks[i]));
    dock->setTitleBarWidget(new QWidget(dock));
    addDockWidget(Qt::TopDockWidgetArea, dock);
    Zoom* zoom = new Zoom(0, tracker_);
    dock->setWidget(zoom);
    addDockWidget(Qt::TopDockWidgetArea, dock);
    zooms_ << zoom;
    zooms_docks_ << dock;
  }
  zooms_docks_.resize(tracks.size());
  zooms_.resize(tracks.size());
  for (int i = 0; i < tracks.size(); i++) {
    zooms_[i]->SetMarker(current_frame_, tracks[i]);
  }
}

void MainWindow::detect() {
  // TODO(MatthiasF): put filtering in a new file with proper API
  // TODO(MatthiasF): process all image filters in one pass
  // TODO(MatthiasF): it could also be useful for tracker to take filtered
  // regions (to avoid filtering overlapping regions multiple times)
  QImage qimage = clip_->Image(current_frame_);
  libmv::Array3Df image(qimage.height(), qimage.width());
  const uchar* src = qimage.constBits();
  float* dst = image.Data();
  for (int i = 0; i < qimage.byteCount(); ++i) {
    dst[i] = src[i] / 255.0;
  }
  libmv::FloatImage gradient;
  libmv::BlurredImageAndDerivativesChannels(image, 0.9, &gradient);

  // TODO(MatthiasF): Feature count would be a better parameter than min_trackness
  // TODO(MatthiasF): A resolution independent parameter would be better than distance
  // e.g. a coefficient going from 0 (no minimal distance) to 1 (optimal circle packing)
  std::vector<libmv::Feature> features;
  libmv::DetectGoodFeatures(gradient, 4, 0.125, 120, &features);
  QVector<int> tracks;
  foreach (const libmv::Feature& p, features) {
    int track = tracks_->MaxTrack() + 1;
    tracks_->Insert(current_frame_, track, p.x, p.y );
    tracks << track;
  }
  tracker_->select(tracks);
  updateZooms(tracks);
}

void MainWindow::solve() {
  // Invert the camera intrinsics.
  libmv::vector<libmv::Marker> markers = tracks_->AllMarkers();
  for (int i = 0; i < markers.size(); ++i) {
    intrinsics_->InvertIntrinsics(markers[i].x,
                                  markers[i].y,
                                  &(markers[i].x),
                                  &(markers[i].y));
  }
  libmv::Tracks normalized_tracks(markers);

  libmv::vector<libmv::Marker> keyframe_markers =
      normalized_tracks.MarkersForTracksInBothImages(0, clip_->Count()-1);

  libmv::ReconstructTwoFrames(keyframe_markers, reconstruction_);
  libmv::Bundle(normalized_tracks, reconstruction_);
  libmv::CompleteReconstruction(normalized_tracks, reconstruction_);
  libmv::ReprojectionError(*tracks_, *reconstruction_, *intrinsics_);
  scene_->upload();
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setOrganizationName("libmv");
  app.setApplicationName("tracker");
  MainWindow window;
  window.show();
  window.open(app.arguments().value(1));
  return app.exec();
}
