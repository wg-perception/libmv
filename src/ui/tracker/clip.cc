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

#include "ui/tracker/clip.h"

#ifdef USE_FFMPEG
extern "C" {
#include <stdint.h>
typedef uint64_t UINT64_C;
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
}
#endif

#include <QFileInfo>
#include <QDir>
#include <QVBoxLayout>
#include <QPushButton>

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

Clip::Clip(QStringList files) {
  images_.clear();
  if ( files.count() == 1 ) {
    if (QFileInfo(files.first()).isDir()) {
      QString folder = files.takeFirst();
      cache_.setFileName(folder+"/cache");
      foreach (QString file, QDir(folder,"*.jpg *.png")
               .entryList(QDir::Files, QDir::Name)) {
        files << QDir(folder).filePath(file);
      }
    } else {
      cache_.setFileName(files.first()+".cache");
    }
  } else {
    cache_.setFileName(QFileInfo(files.first()).dir().path()+"/cache");
  }
  if(!cache_.exists()) {
    // Decode to raw on-disk video cache
    cache_.open(QFile::WriteOnly|QFile::Truncate);
    if (files.count() == 1) {
      DecodeVideo(files.first());
    } else {
      DecodeSequence(files);
    }
    cache_.close();
    QFile meta(cache_.fileName()+".meta");
    meta.open(QFile::WriteOnly|QFile::Truncate);
    meta.write((char*)&size_,sizeof(size_));
  }
  QFile meta(cache_.fileName()+".meta");
  meta.open(QFile::ReadOnly);
  meta.read((char*)&size_, sizeof(size_));
  cache_.open(QFile::ReadOnly);
  int image_size = size_.width()*size_.height();
  images_.resize( cache_.size()/image_size );
  uchar* data = cache_.map(0, cache_.size());
  for(int i = 0; i < images_.count(); i++, data+=image_size) {
    images_[i] = QImage(data,size_.width(),size_.height(),QImage::Format_Indexed8);
  }
}

void Clip::DecodeSequence(QStringList files) {
  foreach (QString file, files) {
    QImage image(file);
    if(image.depth() != 8) {
      QImage grayscale(image.width(),image.height(),QImage::Format_Indexed8);
      QRgb *src = (QRgb*)image.constBits();
      uchar *dst = grayscale.bits();
      for(int i = 0; i < grayscale.byteCount(); i++) dst[i] = qGray(src[i]);
      image = grayscale;
      cache_.write((const char*)image.constBits(),image.byteCount());
    }
  }
}

void Clip::DecodeVideo(QString path) {
#ifdef USE_FFMPEG
  av_register_all();
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
        size_ = QSize(frame->width,frame->height);
        // FIXME: Assume planar format
        const uchar* data = frame->data[0];
        if( frame->linesize[0] != frame->width ) {
          for(int y = 0; y < frame->height; y++) {
            cache_.write((const char*)data,frame->width);
            data += frame->linesize[0];
          }
        } else {
          cache_.write((const char*)data,frame->width*frame->height);
        }
        av_free(frame);
      }
    }
    av_free_packet(&packet);
  }
  avcodec_close(video);
  av_close_input_file(file);
#endif
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
