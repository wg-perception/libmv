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

#include "opencv2/imgproc/imgproc_c.h" //cvFindCornerSubPix
#include "opencv2/calib3d/calib3d.hpp" //cvFindChessboardCorners

#include <QApplication>
#include <QFileDialog>
#include <QSettings>
#include <QPainter>
#include <QTimer>
#include <QTime>

#ifdef USE_FFMPEG
extern "C" {
#include <stdint.h>
typedef uint64_t UINT64_C;
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
}
#endif

#if (QT_VERSION < QT_VERSION_CHECK(4, 7, 0))
#define constBits bits
#endif

View::View() {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

QSize View::sizeHint() const {
  return pixmap.size() / (pixmap.height()>540 ? 2 : 1);
}

void View::setImage(Image image, bool correct) {
  if (correct) {
    corners = image.corrected_corners;
    pixmap = image.corrected_pixmap;
  } else {
    corners = image.distorted_corners;
    pixmap = image.distorted_pixmap;
  }
  updateGeometry();
  update();
}

void View::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHints(QPainter::Antialiasing|QPainter::SmoothPixmapTransform);
  p.drawPixmap(rect(), pixmap);
  if (!corners.isEmpty()) {
    p.scale((float)width()/pixmap.width(),
            (float)height()/pixmap.height());
    p.setPen(Qt::green);
    p.drawPolyline(corners);
  }
}

FileDialog::FileDialog(QWidget* parent,QString caption) : QDialog(parent) {
    setWindowTitle(caption);
    view.setModel(&fileSystem);
    view.setHeaderHidden(true);
    view.setSelectionMode(QAbstractItemView::ExtendedSelection);
    for(int i=1;i<fileSystem.columnCount();i++) view.setColumnHidden(i,true);
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(&view);
    view.setRootIndex(fileSystem.setRootPath(QDir::rootPath()));
    view.resizeColumnToContents(0); setMinimumHeight(600); setMinimumWidth(view.columnWidth(0));
    QPushButton* openButton = new QPushButton(QIcon::fromTheme("document-open"),"Open");
    connect(openButton,SIGNAL(clicked()),this,SLOT(accept()));
    layout->addWidget(openButton);
}

QStringList FileDialog::selectedFiles() {
    QStringList files;
    foreach(QModelIndex index, view.selectionModel()->selectedRows(0) )
        files << index.data(QFileSystemModel::FilePathRole).toString();
    return files;
}


MainWindow::MainWindow() : current(0) {
  setWindowTitle("Camera Calibration");
  side.setRowWrapPolicy(QFormLayout::WrapLongRows);
  side.setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  hbox.addLayout(&side);

  source.setIcon(QIcon(":/open"));
  source.setText("Load Calibration Footage");
  connect(&source,SIGNAL(clicked()),this,SLOT(open()));
  side.addWidget(&source);

  list.setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
  connect(&list, SIGNAL(currentRowChanged(int)), SLOT(showImage(int)));
  side.addRow("Loaded Images", &list);

  width.setRange(3, 256);
  width.setValue(QSettings().value("width", 3).toInt());
  connect(&width, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  side.addRow("Grid Width", &width);

  height.setRange(3, 256);
  height.setValue(QSettings().value("height", 3).toInt());
  connect(&height, SIGNAL(valueChanged(int)), SLOT(calibrate()));
  side.addRow("Grid Height", &height);

  size.setValue(QSettings().value("size", 1).toFloat());
  connect(&size, SIGNAL(valueChanged(double)), SLOT(calibrate()));
  side.addRow("Square Size", &size);

  correct.setEnabled(false);
  connect(&correct, SIGNAL(stateChanged(int)), SLOT(toggleDistort()));
  side.addRow("Undistort",&correct);
  focalLength.setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("Focal Length",&focalLength);
  principalPoint.setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("Principal Point",&principalPoint);
  radialDistortion[0].setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("K1",&radialDistortion[0]);
  radialDistortion[1].setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("K2",&radialDistortion[1]);
  radialDistortion[2].setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("K3",&radialDistortion[2]);
  tangentialDistortion[0].setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("P1",&tangentialDistortion[0]);
  tangentialDistortion[1].setTextInteractionFlags(Qt::TextSelectableByMouse);
  side.addRow("P2",&tangentialDistortion[1]);

  hbox.addWidget(&view);
  setLayout(&hbox);
}
MainWindow::~MainWindow() {
  QSettings().setValue("width", width.value());
  QSettings().setValue("height", height.value());
  QSettings().setValue("size", size.value());
}

void MainWindow::open() {
  FileDialog dialog(this,"Load Calibration Footage");
  if (dialog.exec())
    open(dialog.selectedFiles());
}

void MainWindow::open(QStringList files) {
  list.clear();
  images.clear();
  current=0;
  path = files.count() == 1 ? files.first() : QFileInfo(files.first()).dir().path();
  foreach(QString path, files) {
    if(!QFileInfo(path).exists()) continue;

#ifdef USE_FFMPEG
  av_register_all();
  if (QFileInfo(path).isFile()) {
    AVFormatContext* file = 0;
#if LIBAVFORMAT_VERSION_MAJOR >= 53
    if(avformat_open_input(&file, path.toUtf8(), 0, 0)) return;
#else
    if(av_open_input_file(&file, path.toUtf8(), 0, 0, 0)) return;
#endif
    avformat_find_stream_info(file, NULL);
    int video_stream = 0;
    AVCodecContext* video = 0;
    for (int i = 0; i < (int)file->nb_streams; i++ ) {
        if( file->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
          video_stream = i;
          video = file->streams[i]->codec;
          AVCodec* codec = avcodec_find_decoder(video->codec_id);
          if( codec ) avcodec_open2(video, codec, NULL);
          break;
        }
    }
    source.setIcon(QIcon(":/add"));
    source.setText("Add Current Frame");
    source.disconnect();
    connect(&source,SIGNAL(clicked()),this,SLOT(addImage()));
    QPushButton done;
    done.setIcon(QIcon(":/stop"));
    done.setText("Stop");
    side.insertRow(0,&done);
    connect(&done,SIGNAL(clicked()),SLOT(stop()));
    play = true;
    QTime time; time.start();
    for (AVPacket packet; av_read_frame(file, &packet) >= 0; ) {
      if ( packet.stream_index == video_stream ) {
        AVFrame* frame = avcodec_alloc_frame();
        int complete_frame = 0;
        avcodec_decode_video2(video, frame, &complete_frame, &packet);
        if (complete_frame) {
          // FIXME: Assume planar format
          QImage image(video->width, video->height, QImage::Format_Indexed8);
          int w = image.width(), h = image.height(), bytesPerLine = frame->linesize[0];
          uchar* dst = image.bits();
          const uchar* src = frame->data[0];
          for(int y = 0; y < h; y++) {
              memcpy(dst, src, w);
              dst += w; src += bytesPerLine;
          }
          av_free(frame);
          view.setImage(preview=Image(image));
          // Play quickly the video (min(50fps,decode rate)) while allowing user input
          int wait = 20-time.restart();
          if(wait>0) QApplication::processEvents(QEventLoop::WaitForMoreEvents,wait);
          else QApplication::processEvents();
        }
      }
      av_free_packet(&packet);
      if(!play) break;
    }
    avcodec_close(video);
    avformat_close_input(&file);
    hbox.removeWidget(&done);
    source.setIcon(QIcon(":/open"));
    source.setText("Reset");
    source.disconnect();
    connect(&source,SIGNAL(clicked()),this,SLOT(open()));
  } else
#endif
  foreach (QString file, QDir(path).entryList(QStringList("*.jpg") << "*.png",
                                                QDir::Files, QDir::Name)) {
    list.addItem( file );
    QImage image(QDir(path).filePath(file));
    if(image.depth() != 8) {
      QImage grayscale(image.width(),image.height(),QImage::Format_Indexed8);
      QRgb *src = (QRgb*)image.constBits();
      uchar *dst = grayscale.bits();

      // create grayscale palette
      QVector<QRgb> colors(256);
      for(int i = 0; i < 256; i++)
        colors.replace(i, qRgb(i,i,i));
      grayscale.setColorTable(colors);

      for(int i = 0; i < grayscale.byteCount(); i++)
        dst[i] = qGray(src[i]);

      image = grayscale;
    }
    images << image;
  }
  }
  if(images.isEmpty()) return;
  list.setCurrentRow(0);
  calibrate();
}

void MainWindow::addImage() {
  list.addItem(QString::number(list.count()+1));
  images << preview;
}

void MainWindow::stop() {
  play = false;
}

void MainWindow::showImage(int index) {
  if(index < 0) return;
  Image& image = images[index];
  if (correct.isChecked()) {
    if(image.corrected_pixmap.isNull()) {
      CvSize size = { image.image.width(), image.image.height() };
      IplImage cv_image;
      cvInitImageHeader(&cv_image, size, 8, 1);
      cv_image.imageData = (char*)image.image.constBits();
      cv_image.widthStep = image.image.bytesPerLine();
      IplImage* correct = cvCloneImage( &cv_image );
      CvMat camera_matrix = cvMat( 3, 3, CV_64F, camera );
      CvMat distortion_coefficients = cvMat( 1, 4, CV_64F, coefficients );
      cvUndistort2( &cv_image, correct, &camera_matrix, &distortion_coefficients );
      image.corrected_pixmap = QPixmap::fromImage(
            QImage((uchar*)correct->imageData, correct->width, correct->height,
                   correct->widthStep, QImage::Format_Indexed8));
      cvReleaseImage( &correct );
    }
    view.setImage(image, true);
  } else {
    view.setImage(image, false);
  }
}

void MainWindow::calibrate() {
  for(int i = 0; i < list.count(); i++) {
    list.item(i)->setForeground(QBrush(Qt::black));
  }
  current = 0;
  QTimer::singleShot(50, this, SLOT(process()));
}

void MainWindow::process() {
  CvSize board = { width.value(), height.value() };
  if (current >= images.count()) {
    int point_count = board.width*board.height;
    int image_count = 0;
    foreach (const Image& image, images) {
      if(image.distorted_corners.size() == point_count) {
        image_count++;
      }
    }
    if(image_count <= 4) return;
    CvMat* point_counts = cvCreateMat( 1, image_count, CV_32SC1 );
    cvSet( point_counts, cvScalar(point_count) );

    CvMat* image_points = cvCreateMat( 1, image_count*point_count, CV_32FC2 );
    CvMat* object_points = cvCreateMat( 1, image_count*point_count, CV_32FC3 );
    CvPoint2D32f* image_point = ((CvPoint2D32f*)image_points->data.fl);
    CvPoint3D32f* object_point = ((CvPoint3D32f*)object_points->data.fl);
    for (int i = 0; i < image_count; i++) {
      if (images[i].distorted_corners.size() != point_count) continue;
      for (int j = 0; j < board.height; j++) {
        for (int k = 0; k < board.width; k++) {
          *object_point++ = cvPoint3D32f(j*size.value(), k*size.value(), 0);
          QPointF point = images[i].distorted_corners[j*board.width+k];
          *image_point++ = cvPoint2D32f(point.x(),point.y());
        }
      }
    }

    CvMat camera_matrix = cvMat( 3, 3, CV_64F, camera );
    CvMat distortion_coefficients = cvMat( 1, 5, CV_64F, coefficients );
    cvSetZero( &camera_matrix );
    cvSetZero( &distortion_coefficients );

    CvMat* extrinsics = cvCreateMat( image_count, 6, CV_32FC1 );
    CvMat rotation_vectors, translation_vectors;
    cvGetCols( extrinsics, &rotation_vectors, 0, 3 );
    cvGetCols( extrinsics, &translation_vectors, 3, 6 );

    CvSize size = { images.first().image.width(), images.first().image.height() };
    cvCalibrateCamera2( object_points, image_points, point_counts,
                        size, &camera_matrix, &distortion_coefficients,
                        &rotation_vectors, &translation_vectors );

    CvMat* correct_points = cvCreateMat( 1, image_count*point_count, CV_32FC2 );
    cvUndistortPoints(image_points, correct_points, &camera_matrix, &distortion_coefficients,0,&camera_matrix);
    CvPoint2D32f* correct_point = ((CvPoint2D32f*)correct_points->data.fl);
    for (int i = 0; i < image_count; i++) {
      if (images[i].distorted_corners.size() != point_count) continue;
      images[i].corrected_corners.clear();
      for (int j = 0 ; j < board.width*board.height ; j++) {
        CvPoint2D32f point = *correct_point++;
        images[i].corrected_corners << QPointF(point.x,point.y);
      }
    }

    cvReleaseMat( &object_points );
    cvReleaseMat( &image_points );
    cvReleaseMat( &point_counts );

    focalLength.setText(QString("%1 x %2").arg(camera[0]).arg(camera[4]));
    principalPoint.setText(QString("%1 x %2").arg(camera[2]).arg(camera[5]));
    radialDistortion[0].setText(QString::number(coefficients[0]));
    radialDistortion[1].setText(QString::number(coefficients[1]));
    radialDistortion[2].setText(QString::number(coefficients[4]));
    tangentialDistortion[0].setText(QString::number(coefficients[2]));
    tangentialDistortion[1].setText(QString::number(coefficients[3]));

    QFile file(path + (QFileInfo(path).isDir()?"/":".") + "camera.xml");
    if( file.open(QFile::WriteOnly | QFile::Truncate) ) {
      file.write(QString("<lens FocalLengthX='%1' FocalLengthY='%2'"
                         " PrincipalPointX='%3' PrincipalPointY='%4'"
                         " k1='%5' k2='%6' k3='%7' p1='%8' p2='%9'/>")
                 .arg(camera[0]).arg(camera[4])
                 .arg(camera[2]).arg(camera[5])
                 .arg(coefficients[0]).arg(coefficients[1]).arg(coefficients[4])
                 .arg(coefficients[2]).arg(coefficients[3])
                 .toAscii());
    }
    correct.setEnabled(true);
    return;
  }
  Image& image = images[current];
  image.distorted_corners.clear();
  CvSize size = { image.image.width(), image.image.height() };
  IplImage cv_image;
  cvInitImageHeader(&cv_image, size, 8, 1);
  cv_image.imageData = (char*)image.image.constBits();
  cv_image.widthStep = image.image.bytesPerLine();
  CvPoint2D32f corners[board.width*board.height];
  bool found = false;
  if (cvCheckChessboard(&cv_image, board) == 1) {
    int count = 0;
    found = cvFindChessboardCorners(&cv_image, board, corners, &count );
    if (found) {
      cvFindCornerSubPix(&cv_image, corners, count, cvSize(11,11), cvSize(-1,-1),
                         cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
      for (int i = 0 ; i < board.width*board.height ; i++) {
        image.distorted_corners << QPointF(corners[i].x, corners[i].y);
      }
    }
  }
  list.item(current)->setForeground(QBrush(found ? Qt::green : Qt::red));
  list.setCurrentRow(current);
  current++;
  QTimer::singleShot(50, this, SLOT(process()));  // Voluntary preemption
}

void MainWindow::toggleDistort() {
  showImage(list.currentRow());
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  app.setOrganizationName("libmv");
  app.setApplicationName("calibration");
  MainWindow window;
  window.show();
  if (app.arguments().size() == 2)
    window.open(app.arguments().mid(1));
  return app.exec();
}
