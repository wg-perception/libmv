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

#include "ui/tracker/calibration.h"

#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>
#include <QFormLayout>
#include <QDoubleSpinBox>

struct Parameter {
  const char* id;
  const char* name;
  const char* suffix;
  double min;
  double max;
  double value;
};

Calibration::Calibration(QString path, QSize size) : CameraIntrinsics(), path_(path) {
  setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Maximum);

  const int kCount = 11;
  const Parameter parameters[kCount] = {
    {"ImageWidth",      "Real Image Width",          "px", 0,  size.width()*2.0,      (double)size.width()  },
    {"ImageHeight",     "Real Image Height",         "px", 0,  size.height()*2.0,     (double)size.height() },
    {"FocalLengthX",    "Focal Length (X)",          "px", 0,  size.width()*4.0,      size.width()*2.0      },
    {"FocalLengthY",    "Focal Length (Y)",          "px", 0,  size.width()*4.0,      size.width()*2.0      },
    {"PrincipalPointX", "Principal Point (X)",       "px", 0,  (double)size.width(),  size.width()/2.0      },
    {"PrincipalPointY", "Principal Point (Y)",       "px", 0,  (double)size.height(), size.height()/2.0     },
    {"k1",              "1st Radial Distortion",     "",  -1,  1,                     0                     },
    {"k2",              "2nd Radial Distortion",     "",  -1,  1,                     0                     },
    {"k3",              "3rd Radial Distortion",     "",  -1,  1,                     0                     },
    {"p1",              "1st Tangential Distortion", "",  -1,  1,                     0                     },
    {"p2",              "2nd Tangential Distortion", "",  -1,  1,                     0                     },
  };
  QFormLayout* layout = new QFormLayout(this);
  for (int i = 0; i < kCount; i++) {
    const Parameter& parameter = parameters[i];
    QDoubleSpinBox* spinbox = spinbox_[i] = new QDoubleSpinBox(this);
    spinbox->setSuffix(parameter.suffix);
    spinbox->setRange(parameter.min, parameter.max);
    spinbox->setDecimals(3);
    spinbox->setValue(parameter.value);
    layout->addRow(parameter.name, spinbox);
    connect(spinbox,SIGNAL(valueChanged(double)),SLOT(updateSettings()));
  }

  QFile file(path + (QFileInfo(path).isDir()?"/":".") + "camera.xml");
  if( file.open(QFile::ReadOnly) ) {
    QXmlStreamReader xml(&file);
    for(;!xml.atEnd();) {
      QXmlStreamReader::TokenType token = xml.readNext();
      if(token == QXmlStreamReader::StartElement) {
        QXmlStreamAttributes lens = xml.attributes();
        for (int i = 0; i < kCount; i++) {
          const Parameter& parameter = parameters[i];
          if(lens.hasAttribute(parameter.id)) {
            spinbox_[i]->setValue(lens.value(parameter.id).toString().toFloat());
          }
        }
      }
    }
  }

  updateSettings();
}

double Calibration::Value(int i) { return spinbox_[i]->value(); }

void Calibration::updateSettings() {
  SetImageSize(Value(0),Value(1));
  SetFocalLength(Value(2),Value(3));
  SetPrincipalPoint(Value(4),Value(5));
  SetRadialDistortion(Value(6), Value(7), Value(8));
  SetTangentialDistortion(Value(9),Value(10));

  QFile file(path_ + (QFileInfo(path_).isDir()?"/":".") + "camera.xml");
  if( file.open(QFile::WriteOnly | QFile::Truncate) ) {
    file.write(QString("<lens ImageWidth='%1' ImageHeight='%2' "
                       "FocalLengthX='%3' FocalLengthY='%4' "
                       "PrincipalPointX='%5' PrincipalPointY='%6' "
                       "k1='%7' k2='%8' k3='%9' p1='%10' p2='%11' />")
               .arg(Value(0)).arg(Value(1))
               .arg(Value(2)).arg(Value(3))
               .arg(Value(4)).arg(Value(5))
               .arg(Value(6)).arg(Value(7))
               .arg(Value(8)).arg(Value(9)).arg(Value(10)) .toAscii());
  }

  emit settingsChanged();
}
