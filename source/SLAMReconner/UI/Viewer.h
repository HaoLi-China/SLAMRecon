// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#ifndef _VIEWER_H
#define _VIEWER_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_2_Core>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <QMatrix4x4>

#include "Define.h"

class Viewer : public QOpenGLWidget, public QOpenGLFunctions_3_2_Core
{
public:
    Viewer();
    void initializeGL();
	QOpenGLShaderProgram* genShaderProgram(QString shadername);

    QMap<QString, QOpenGLShaderProgram*> shaderPrograms;

    // Active camera properties
    QMatrix4x4 pvm;
    QVector3D eyePos;

    // Draw primitives
	void drawPoints(const QVector<QVector3D> &points, float& pointSize, QColor color, QMatrix4x4 camera, bool isConnected = false);
	void drawOrientedPoints(const QVector< QVector3D > & points, const QVector< QVector3D > & normals, QColor color, QMatrix4x4 camera);
	void drawOrientedPoints(const QVector< QVector3D > & points, const QVector< QVector3D > & normals, const QVector<QVector3D> & colors, QMatrix4x4 camera);
	void drawLines(const QVector<QVector3D> &lines, QColor color, QMatrix4x4 camera, QString shaderName);
    void drawBox(double width, double length, double height, QMatrix4x4 camera);
	void drawBox(const QVector<QVector3D> verts, const QColor &color, QMatrix4x4 camera);
    void drawQuad(const QImage &img);
    void drawPlane(QVector3D normal, QVector3D origin, QMatrix4x4 camera);
	void drawTexturedPlane(const QImage &img, const QVector<QVector3D> verts, QMatrix4x4 camera);
    void drawTriangles(QColor useColor, const QVector<QVector3D> &points, const QVector<QVector3D> &normals, QMatrix4x4 camera);
	void drawRenderingImgs(Vector4u* freeRenderImg, Vector2i freeRenderImgSize, unsigned int id1, 
		Vector4u* renderImg, Vector2i renderImgSize, unsigned int id2,
		Vector4u* rgbImg, Vector2i rgbImgSize, unsigned int id3,
		Vector4u* depthImg, Vector2i depthImgSize, unsigned int id4);
};

#endif //_VIEWER_H