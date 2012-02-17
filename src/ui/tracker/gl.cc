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

#include "ui/tracker/gl.h"

#include <QDebug>
#include <QImage>
#include <QFile>
#include <QStringList>

#ifdef GLEW
# include "GL/glew.h"
#else
# define GL_GLEXT_PROTOTYPES
# ifdef __APPLE__
#  include "OpenGL/gl.h"
# else
#  include "GL/gl.h"
# endif
#endif

#ifdef WIN32
#include "GL/wglew.h"
#else
#ifdef GLEW
#include "GL/glxew.h"
#else
extern "C" int glXSwapIntervalSGI(int interval);
#endif
#endif

void glInitialize() {
#ifdef GLEW
  glewInit();
#endif
#ifdef WIN32
  wglSwapIntervalEXT(1);
#else
#if 0
  glXSwapIntervalSGI(1);
#endif
#endif
}

void GLUniform::operator=(int v) {
  if (id >= 0) glUniform1i(id, v);
}
void GLUniform::operator=(double v) {
  if (id >= 0) glUniform1f(id, v);
}
void GLUniform::operator=(vec2 v) {
  if (id >= 0) glUniform2f(id, v.x, v.y);
}
void GLUniform::operator=(vec3 v) {
  if (id >= 0) glUniform3f(id, v.x, v.y, v.z);
}
void GLUniform::operator=(vec4 v) {
  if (id >= 0) glUniform4f(id, v.x, v.y, v.z, v.w);
}
void GLUniform::operator=(mat4 m) {
  if (id >= 0) glUniformMatrix4fv(id, 1, 0, m.data);
}
void GLUniform::set(vec3* data, int size) {
  if (id >= 0) glUniform3fv(id, size, reinterpret_cast<float*>(data));
}
void GLUniform::set(vec4* data, int size) {
  if (id >= 0) glUniform4fv(id, size, reinterpret_cast<float*>(data));
}

bool compileShader(int program, int type, QByteArray source) {
  int shader = glCreateShader(type);
  const char* string = source.constData(); glShaderSource(shader,1,&string,0);
  glCompileShader(shader); glAttachShader(program,shader);
  int status; glGetShaderiv(shader,GL_COMPILE_STATUS,&status);
  if(status==GL_TRUE) return true;
  int l=0; glGetShaderiv(shader,GL_INFO_LOG_LENGTH,&l);
  QByteArray msg(l,0); glGetShaderInfoLog(shader,l,0,msg.data());
  qDebug()<<(type==GL_VERTEX_SHADER?"Vertex:\n":"Fragment:\n")<<msg;
  return false;
}
bool GLShader::compile(QString vertex, QString fragment) {
    if(!id) id = glCreateProgram();
    compileShader(id,GL_VERTEX_SHADER,vertex.toAscii());
    compileShader(id,GL_FRAGMENT_SHADER,fragment.toAscii());
    glLinkProgram(id);
    int status; glGetProgramiv(id,GL_LINK_STATUS,&status);
    if(status==GL_TRUE) return true;
    int l=0; glGetProgramiv(id,GL_INFO_LOG_LENGTH,&l);
    QByteArray msg(l,0); glGetProgramInfoLog(id,l,0,msg.data());
    qDebug()<<"Program:\n"<<msg;
    return false;
}

void GLShader::bind() {
  glUseProgram(id);
}

int GLShader::attribLocation(const char* name ) {
  int location = attribLocations.value(name, -1);
  if (location < 0) {
    attribLocations[name] = location = glGetAttribLocation(id, name);
  }
  return location;
}

GLUniform GLShader::operator[](const char* name) {
  int location = uniformLocations.value(name, -1);
  if (location < 0) {
    uniformLocations[name] = location = glGetUniformLocation(id, name);
  }
  return GLUniform(location);
}


void GLBuffer::upload(const void* data, int count) {
  if (!indexBuffer) glGenBuffers(1, &indexBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, count*sizeof(uint), data,
               GL_STATIC_DRAW);
  indexCount = count;
}

void GLBuffer::upload(const void* data, int count, int size) {
  if (!vertexBuffer) glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, count*size, data, GL_STATIC_DRAW);
  vertexCount = count;
  vertexSize = size;
}

void GLBuffer::bind() {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
}

void GLBuffer::bindAttribute(GLShader* program, const char* name,
                             int elementSize, size_t offset) {
  int location = program->attribLocation(name);
  if (location < 0) qWarning() << "unused attrib" << name;
  glVertexAttribPointer(location, elementSize, GL_FLOAT, 0, vertexSize,
                        reinterpret_cast<void*>(offset));
  glEnableVertexAttribArray(location);
}

void GLBuffer::draw() {
  uint mode[] = { 0, GL_POINTS, GL_LINES, GL_TRIANGLES, GL_QUADS };
  if (primitiveType == 1) {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);
  }
  if (indexBuffer) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    glDrawElements(mode[primitiveType], indexCount, GL_UNSIGNED_INT, 0);
  } else {
    glDrawArrays(mode[primitiveType], 0, vertexCount);
  }
}

#if (QT_VERSION < QT_VERSION_CHECK(4, 7, 0))
#define constBits bits
#endif

void GLTexture::upload(QImage image) {
  width = image.width();
  height = image.height();
  depth = image.depth();
  int stride = image.bytesPerLine();
#ifdef USE_PBO
  GLuint pbo; glGenBuffers(1,&pbo);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo );
  glBufferData(GL_PIXEL_UNPACK_BUFFER, width*height, 0, GL_STREAM_DRAW);
  uchar* dst = (uchar*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER,GL_WRITE_ONLY);
  Q_ASSERT(dst);
  const uchar* src = image.constBits();
  for(int y = 0; y < height; y++) {
      memcpy(dst, src, width);
      dst += width; src += stride;
  }
  glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
#else
  if (stride != width*image.depth()) {
    glPixelStorei(GL_UNPACK_ROW_LENGTH, stride);
  }
#endif
  if (!id) glGenTextures(1, &id);
  glBindTexture(GL_TEXTURE_2D, id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, depth/8, width, height, 0,
               depth == 8 ? GL_LUMINANCE : GL_BGRA, GL_UNSIGNED_BYTE,
#ifdef USE_PBO
               0);
  glDeleteBuffers(1, &pbo);
#else
               image.constBits());
#endif
}

void GLTexture::bind(int sampler) {
  glActiveTexture(GL_TEXTURE0+sampler);
  glBindTexture(GL_TEXTURE_2D, id);
}

void glQuad(vec4 quad[4]) {
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glVertexAttribPointer(0, 4, GL_FLOAT, 0, 0, quad);
  glEnableVertexAttribArray(0);
  glDrawArrays(GL_QUADS, 0, 4);
}

void glBindWindow(int x, int y, int w, int h, bool clear) {
  glViewport(x, y, w, h);
  if(clear) glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
}

void glSmooth()  {
  glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
  glEnable(GL_LINE_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
}
void glHard() {
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

static QString filterGLSL(QFile* file, QStringList tags) {
  QRegExp filter("(\\w*) \\{");
  QString l = file->readLine().simplified();
  if (!filter.exactMatch(l)) return l;
  bool enabled = tags.contains(filter.cap(1));
  QString out;
  for (int nest = 0; (l = filterGLSL(file, tags)) != "}" || nest > 0;) {
    if (l.isEmpty()) continue;
    if (enabled) out+=l+"\n";
    nest += l.count("{")-l.count("}");
  }
  return out;
}

QString glsl(QString tags) {
  QFile file(":/shader.glsl");
  file.open(QFile::ReadOnly);
  QString global, main;
  while (!file.atEnd()) {
    foreach (QString line, filterGLSL(&file, tags.split(" ")).split("\n")) {
      if (!line.indexOf(QRegExp("uniform|attribute|varying|in|out|(float|vec[12"
                                "34]|mat[234]) [a-zA-Z0-9]*\\("))) {
        global+=line+"\n";
      } else {
        main+=line+"\n";
      }
    }
  }
  return "#version 120\n"+global+"\nvoid main() {\n"+main+"\n}\n";
}
