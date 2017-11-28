#ifndef GLWIDGET_H
#define GLWIDGET_H


#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QDebug>
#include <QOpenGLTexture>
#include <GL/gl.h>
#include <QGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>



class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0);

    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void LoadGLTextures();

private :

    QOpenGLShaderProgram *program;
    GLuint tex;


public slots:



private slots:



};
#endif // GLWIDGET_H
