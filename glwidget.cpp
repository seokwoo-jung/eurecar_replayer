#include "glwidget.h"
#include "ui_glwidget.h"


GLWidget::GLWidget(QWidget *parent) :
        QOpenGLWidget(parent)
{

}

void GLWidget::LoadGLTextures(){

    QImage img;

    if(!img.load("/home/usrg_eurecar_stu/Documents/1507702514030235.jpg")){

        qDebug()<<"Image loading failed";
    }

    QImage t = (img.convertToFormat(QImage::Format_RGBA8888)).mirrored();

    glGenTextures(1, &tex);

    glBindTexture(GL_TEXTURE_2D, tex);

        glTexImage2D(GL_TEXTURE_2D, 0, 3, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture( GL_TEXTURE_2D, 0);



}

void GLWidget::initializeGL(){

    initializeOpenGLFunctions();

    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_TEXTURE_2D);
    LoadGLTextures();

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
        "#version 150 core\n"
        "in vec2 in_Vertex;\n"
        "in vec2 vertTexCoord;\n"
        "out vec2 fragTexCoord;\n"
        "void main(void)\n"
        "{\n"
        "    gl_Position = vec4(in_Vertex, 0.0, 1.0);\n"
        "    fragTexCoord = vertTexCoord;\n"
        "}\n";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
            "#version 150 core\n"
            "uniform sampler2D tex;\n"
            "in vec2 fragTexCoord;\n"
            "void main(void)\n"
            "{\n"
            "    gl_FragColor = texture2D(tex,fragTexCoord);\n"
            "}\n";
    fshader->compileSourceCode(fsrc);

    program = new QOpenGLShaderProgram;
    program->addShader(vshader);
    program->addShader(fshader);

    program->link();
    program->bind();
    glActiveTexture(GL_TEXTURE0);
    program->setUniformValue("tex", 0);

}

void GLWidget::paintGL(){

    glClear(GL_COLOR_BUFFER_BIT);

    program->bind();
    {
        float vertices[] = {-1.0,-1.0,  1.0,-1.0,  1.0,1.0,  -1.0,1.0};

        float coordTexture[] = {0.0,0.0,  1.0,0.0,  1.0,1.0,  0.0,1.0};

        GLint vertexLocation = glGetAttribLocation(program->programId(), "in_Vertex" );
        GLint texcoordLocation = glGetAttribLocation( program->programId(), "vertTexCoord" );

        glVertexAttribPointer(vertexLocation, 2, GL_FLOAT, GL_FALSE, 0, vertices);
        glEnableVertexAttribArray(texcoordLocation);

        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, coordTexture);
        glEnableVertexAttribArray(texcoordLocation);

        glBindTexture(GL_TEXTURE_2D, tex);

//        glDrawArrays(GL_QUADS, vertexLocation, 4);

        glBindTexture(GL_TEXTURE_2D, 0);

        glDisableVertexAttribArray(2);
        glDisableVertexAttribArray(0);


    }
    program->release();

}

void GLWidget::resizeGL(int w, int h){

    glViewport(0, 0, (GLint)w, (GLint)h);

}
