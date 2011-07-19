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

#include "ui/tracker/main.h"
#include "ui/tracker/clip.h"
#include "ui/tracker/calibration.h"
#include "ui/tracker/tracker.h"
#include "ui/tracker/zoom.h"
#include "ui/tracker/scene.h"

#include "libmv/simple_pipeline/detect.h"

// -> reconstruction
#include "libmv/simple_pipeline/initialize_reconstruction.h"
#include "libmv/simple_pipeline/bundle.h"
#include "libmv/simple_pipeline/pipeline.h"

#include <QApplication>
#include <QFileDialog>
#include <QDockWidget>
#include <QToolButton>
#include <QSettings>
#include <QToolBar>
#include <QAction>
#include <QMenu>
#include <QTime>

MainWindow::MainWindow() : clip_(0), calibration_(0), tracker_(0), zoom_(0), scene_(0) {
  setWindowTitle("Tracker");
  toolbar_ = addToolBar("Main Toolbar");
  toolbar_->setObjectName("mainToolbar");
  toolbar_->addAction(QIcon(":/open"), "Load Footage", this, SLOT(open()));
}

MainWindow::~MainWindow() {
  QSettings().setValue("geometry", saveGeometry());
  QSettings().setValue("windowState", saveState());
  QSettings().setValue("currentFrame", current_frame_);
  if(tracker_) tracker_->Save(path_);
  //if(scene_) scene_->Save(path_);
}

void MainWindow::open() {
  FileDialog dialog(this,"Load Footage");
  dialog.exec();
  open(dialog.selectedFiles());
}

void MainWindow::open(QStringList files) {
// Clip
  if (files.isEmpty()) return;
  clip_ = new Clip(files);
  if (clip_->Count() == 0) return;
  path_ = files.count() == 1 ? files.first() : QFileInfo(files.first()).dir().path();
  setWindowTitle(QString("Tracker - %1").arg(QDir(path_).dirName()));
  current_frame_ = -1;

// Calibration
  calibration_ = new Calibration(path_, clip_->Size());
  QDockWidget* calibration_dock = new QDockWidget("Calibration View");
  calibration_dock->setObjectName("calibrationDock");
  addDockWidget(Qt::TopDockWidgetArea, calibration_dock);
  calibration_dock->setWidget(calibration_);
  calibration_dock->toggleViewAction()->setIcon(QIcon(":/view-calibration"));
  calibration_dock->toggleViewAction()->setChecked(false);
  toolbar_->addAction(calibration_dock->toggleViewAction());

// Tracker
  tracker_ = new Tracker(calibration_);
  setCentralWidget(tracker_);
  connect(calibration_, SIGNAL(settingsChanged()), tracker_, SLOT(update()));

  QAction* tracker_action_ = toolbar_->addAction(QIcon(":/view-image"),
                                                "Tracker View");
  tracker_action_->setCheckable(true);
  tracker_action_->setChecked(true);
  connect(tracker_action_, SIGNAL(triggered(bool)),
          tracker_, SLOT(setVisible(bool)));

// Zoom
  zoom_ = new Zoom(tracker_);
  QDockWidget* zoom_dock = new QDockWidget("Zoom Grid");
  zoom_dock->setObjectName("zoomDock");
  addDockWidget(Qt::TopDockWidgetArea, zoom_dock);
  zoom_dock->setWidget(zoom_);
  zoom_dock->toggleViewAction()->setIcon(QIcon(":/view-zoom"));
  zoom_dock->toggleViewAction()->setChecked(false);
  toolbar_->addAction(zoom_dock->toggleViewAction());
  connect(tracker_, SIGNAL(trackChanged(QVector<int>)), zoom_, SLOT(select(QVector<int>)));

// Reconstruction
  scene_ = new Scene(calibration_,tracker_);
  QDockWidget* scene_dock = new QDockWidget("Scene View");
  scene_dock->setObjectName("sceneDock");
  addDockWidget(Qt::BottomDockWidgetArea, scene_dock);
  scene_dock->setWidget(scene_);
  scene_dock->toggleViewAction()->setIcon(QIcon(":/view-scene"));
  scene_dock->toggleViewAction()->setChecked(false);
  toolbar_->addAction(scene_dock->toggleViewAction());
  connect(scene_, SIGNAL(imageChanged(int)), SLOT(seek(int)));
  connect(scene_, SIGNAL(trackChanged(QVector<int>)),
          tracker_, SLOT(select(QVector<int>)));
  connect(scene_, SIGNAL(trackChanged(QVector<int>)),
          zoom_, SLOT(select(QVector<int>)));
  tracker_->SetOverlay(scene_);

  toolbar_->addSeparator();

  toolbar_->addAction(QIcon(":/detect"), "Detect features",this, SLOT(detect()));

  QToolButton* delete_button = new QToolButton();
  toolbar_->addWidget(delete_button);
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

  track_action_ = toolbar_->addAction(QIcon(":/record"), "Track selected markers");
  track_action_->setCheckable(true);
  //track_action_->setChecked(true);
  connect(track_action_, SIGNAL(triggered(bool)), SLOT(toggleTracking(bool)));
  connect(tracker_action_, SIGNAL(triggered(bool)),
          track_action_, SLOT(setVisible(bool)));

  toolbar_->addAction(QIcon(":/solve"), "Solve reconstruction",
                     this, SLOT(solve()));
  QAction* add_action = toolbar_->addAction(QIcon(":/add"), "Add object",
                                           scene_, SLOT(add()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          add_action, SLOT(setVisible(bool)));

  QAction* link_action = toolbar_->addAction(
        QIcon(":/link"), "Link active object to selected bundles",
        scene_, SLOT(link()));
  connect(scene_dock->toggleViewAction(), SIGNAL(triggered(bool)),
          link_action, SLOT(setVisible(bool)));

  toolbar_->addSeparator();

  toolbar_->addAction(QIcon(":/skip-backward"), "Seek to first frame",
                     this, SLOT(first()));
  toolbar_->addAction(QIcon(":/step-backward"), "Step to previous frame",
                     this, SLOT(previous()))->setShortcut(Qt::Key_Left);
  backward_action_ = toolbar_->addAction(QIcon(":/play-backward"),
                                        "Play backwards");
  backward_action_->setCheckable(true);
  connect(backward_action_, SIGNAL(triggered(bool)),
          SLOT(toggleBackward(bool)));
  connect(&previous_timer_, SIGNAL(timeout()), SLOT(previous()));

  toolbar_->addWidget(&spinbox_);
  connect(&spinbox_, SIGNAL(valueChanged(int)), SLOT(seek(int)));
  toolbar_->addWidget(&slider_);
  slider_.setOrientation(Qt::Horizontal);
  connect(&slider_, SIGNAL(valueChanged(int)), SLOT(seek(int)));

  forward_action_ = toolbar_->addAction(QIcon(":/play-forward"),
                                       "Play forwards");
  forward_action_->setCheckable(true);
  connect(forward_action_, SIGNAL(triggered(bool)), SLOT(toggleForward(bool)));
  connect(&next_timer_, SIGNAL(timeout()), SLOT(next()));
  toolbar_->addAction(QIcon(":/step-forward"), "Next Frame", this, SLOT(next()))
      ->setShortcut(Qt::Key_Right);
  toolbar_->addAction(QIcon(":/skip-forward"), "Last Frame", this, SLOT(last()));

  //tracker_->Load(path_);
  scene_->Load(path_);

  spinbox_.setMaximum(clip_->Count() - 1);
  slider_.setMaximum(clip_->Count() - 1);
  int frame = 0; //QSettings().value("currentFrame", 0).toInt();
  if(frame >= 0 && frame < clip_->Count()) {
    seek(frame);
  } else {
    seek(0);
  }

  //detect();

  restoreGeometry(QSettings().value("geometry").toByteArray());
  restoreState(QSettings().value("windowState").toByteArray());
}

void MainWindow::seek(int next) {
  // Bail out if there's nothing to do.
  if (next == current_frame_) {
    return;
  }
  // Stop if we hit one of both clip ends.
  if (next < 0 || next >= clip_->Count()) {
    stop();
    return;
  }
  // Disable tracking when scrubbing
  if ( next > current_frame_ + 1 || next < current_frame_ - 1 ) {
    track_action_->setChecked(false);
  }
  int previous = current_frame_;
  current_frame_ = next;

  slider_.setValue(next);
  spinbox_.setValue(next);
  if(previous >= 0 && track_action_->isChecked()) {
    tracker_->Track(previous, next, clip_->Image(previous), clip_->Image(next));
  }
  tracker_->SetImage(next, clip_->Image(next));
  zoom_->SetImage(next);
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

#if (QT_VERSION < QT_VERSION_CHECK(4, 7, 0))
#define constBits bits
#endif

void MainWindow::detect() {
  QImage image = clip_->Image(current_frame_);
  std::vector<libmv::Corner> corners = libmv::Detect(image.constBits(), image.width(),
                                                     image.height(), image.bytesPerLine(),16,128,120);

  // Insert features
  QVector<int> tracks;
  foreach(libmv::Corner corner, corners) {
    int track = tracker_->MaxTrack() + 1;
    tracker_->Insert(current_frame_, track, corner.x, corner.y );
    tracks << track;
  }
  tracker_->select(tracks);
  zoom_->select(tracks);
}

void MainWindow::solve() {
  // Invert the camera intrinsics.
  // TODO(MatthiasF): handle varying focal lengths
  // TODO(MatthiasF): -> reconstruction
  libmv::vector<libmv::Marker> markers = tracker_->AllMarkers();
  for (int i = 0; i < markers.size(); ++i) {
    calibration_->InvertIntrinsics(markers[i].x,
                                  markers[i].y,
                                  &(markers[i].x),
                                  &(markers[i].y));
  }
  libmv::Tracks normalized_tracks(markers);
  libmv::vector<libmv::Marker> keyframe_markers =
      normalized_tracks.MarkersForTracksInBothImages(0, clip_->Count()-1);
  libmv::ReconstructTwoFrames(keyframe_markers, scene_);
  libmv::Bundle(normalized_tracks, scene_);
  libmv::CompleteReconstruction(normalized_tracks, scene_);
  libmv::ReprojectionError(*tracker_, *scene_, *calibration_);

  scene_->upload();
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setOrganizationName("libmv");
  app.setApplicationName("tracker");
  MainWindow window;
  window.show();
  window.open(app.arguments().mid(1));
  return app.exec();
}
