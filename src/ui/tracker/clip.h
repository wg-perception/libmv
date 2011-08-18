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

#ifndef UI_TRACKER_CLIP_H_
#define UI_TRACKER_CLIP_H_

#include <QImage>
#include <QFile>
#include <QDialog>
#include <QFileSystemModel>
#include <QTreeView>

class FileDialog : public QDialog {
    Q_OBJECT
public:
    FileDialog(QWidget*,QString);
    QStringList selectedFiles();
private:
    QFileSystemModel fileSystem;
    QTreeView view;
};

#ifdef USE_FFMPEG
class AVFormatContext;
class AVCodecContext;
#endif

class Clip {
 public:
  Clip(QStringList files);

  void DecodeSequence(QStringList files);
  void DecodeVideo(QString path);

  int Count();
  QSize Size();
  QImage Image(int);

 private:
  QSize size_;
  QFile cache_;
  QVector<QImage> images_;
};
#endif
