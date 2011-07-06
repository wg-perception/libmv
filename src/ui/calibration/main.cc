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

#include "opencv2/calib3d/calib3d.hpp"

#include <QApplication>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPainter>

Image::Image(QString path) : pixmap(path) {
    setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
}

QSize Image::sizeHint() const {
    return pixmap.size();
}

void Image::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.drawPixmap(rect(),pixmap);
}

MainWindow::MainWindow() : current(0) {
    setWindowTitle("Camera Calibration");
    hbox = new QHBoxLayout(this);
    QFormLayout* side = new QFormLayout();
    side->setRowWrapPolicy(QFormLayout::WrapAllRows);
    hbox->addLayout(side);
    side->addRow("Source image folder",&source);
    side->addRow("Loaded images",&list);
    side->addRow("Number of corners along horizontal",&width);
    side->addRow("Number of corners along vertical",&height);
    side->addRow("Real distance between corners",&size);

    connect(&list,SIGNAL(currentRowChanged(int)),SLOT(showImage(int)));
}

void MainWindow::open() {
    open(QFileDialog::getExistingDirectory(this, "Select image folder"));
}

void MainWindow::open(QString folder) {
    foreach (Image* image, images) {
        delete image;
    }
    if (folder.isEmpty() || !QDir(folder).exists()) return;
    source.setText(QDir(folder).canonicalPath());
    foreach (QString file, QDir(folder).entryList(QStringList("*.jpg") << "*.png",
                                                QDir::Files, QDir::Name)) {
        list.addItem( file );
        Image* image = new Image( QDir(folder).filePath(file) );
        image->hide();
        images << image;
    }
    list.setCurrentRow(0);
}

void MainWindow::showImage(int index) {
    if (current) { current->hide(); hbox->removeWidget(current); }
    current = images[index];
    hbox->addWidget(current); current->show();
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setOrganizationName("libmv");
    app.setApplicationName("calibration");
    MainWindow window;
    window.show();
    window.open(app.arguments().value(1));
    return app.exec();
}

