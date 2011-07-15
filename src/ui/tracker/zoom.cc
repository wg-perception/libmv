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
#include "ui/tracker/zoom.h"
#include "ui/tracker/gl.h"

Zoom::Zoom(Tracker *tracker) : QGLWidget(QGLFormat(QGL::SampleBuffers), 0, tracker),
    tracker_(tracker) {}

void Zoom::SetImage(int image) {
  current_image_ = image;
  update();
}

void Zoom::select(QVector<int> tracks) {
  tracks_ = tracks;
  update();
}

void Zoom::paintGL() {
  glBindWindow(0, 0, width(), height(), true);
  if (tracks_.count() == 0) return;
  // FIXME: There are probably better way to do this.
  // Initial estimate using available area
  int size = ceil(sqrt(width()*height()/tracks_.count()));
  int columns = width()/size, rows = height()/size;
  // Tweak size to fit to widget
  if (columns*rows < tracks_.count()) {
    size = qMax( width()/(columns+1), height()/(rows+1) );
    columns = width()/size, rows = height()/size;
    if (columns*rows < tracks_.count()) {
      size = qMin( width()/(columns+1), height()/(rows+1) );
      columns = width()/size, rows = height()/size;
      Q_ASSERT(columns*rows >= tracks_.count());
    }
  }
  for (int i = 0; i < tracks_.count(); i++) {
    int y = i/columns, x = i%columns;
    tracker_->Render(x*size, y*size, size, size, current_image_, tracks_[i]);
  }
}
