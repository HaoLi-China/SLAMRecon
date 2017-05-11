// Copyright 2016-2017 Interdisciplinary Research Center in Shandong University and the authors of SLAMRecon.
#include "Viewer.h"

#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include <QPainter>

Viewer::Viewer()
{
    QSurfaceFormat format;
    format.setSamples(4);
    setFormat(format);

    setMouseTracking(true);
}

void Viewer::initializeGL()
{
    initializeOpenGLFunctions();

    /// Antialiasing and alpha blending:
    glEnable(GL_MULTISAMPLE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

	//glRenderMode(GL_SELECT);

    /// Prepare shaders
	QVector<QString> shadernames;
	shadernames << "points" << "lines" << "grid_lines" 
		<< "translucent" << "box" << "texturedQuad" << "mesh" << "texturedPlane";
	for (auto sname : shadernames) {
		shaderPrograms.insert(sname, genShaderProgram(sname));
	}
}

QOpenGLShaderProgram* Viewer::genShaderProgram(QString shadername)
{
	auto program = new QOpenGLShaderProgram(context());

	// Basic points shader
	if (shadername == "points")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"uniform highp mat4 matrix;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform mediump vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"   gl_FragColor = color; \n"
			"}");
	}
	// Basic lines shader
	else if (shadername == "lines")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"uniform highp mat4 matrix;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform mediump vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"   gl_FragColor = color; \n"
			"}");
	}
	// Grid-Lines shader
	else if (shadername == "grid_lines")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"uniform highp mat4 matrix;\n"
			"varying vec3 pos;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"   pos = vertex.xyz;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform mediump vec4 color;\n"
			"varying vec3 pos;\n"
			"void main(void)\n"
			"{\n"
			"   color.a = clamp(0.5 - color.a * length(pos),0,1);\n"
			"   gl_FragColor = color; \n"
			"   if(pos.x + pos.y == 0) gl_FragColor.a *= 3.0; \n"
			"   if(pos.z + pos.y == 0) gl_FragColor.a *= 3.0; \n"
			"   if(pos.z + pos.x == 0) gl_FragColor.a *= 3.0; \n"
			"   if (gl_FragColor.a < 0.01)discard;"
			"}");
	}
	// Translucent shader
	else if (shadername == "translucent")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"uniform highp mat4 matrix;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform mediump vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"   gl_FragColor = color; \n"
			"}");
	}
	// Box shader
	else if (shadername == "box")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"attribute highp vec4 color;\n"
			"uniform highp mat4 matrix;\n"
			"varying vec4 vColor;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"   vColor = color;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"varying vec4 vColor;\n"
			"void main(void)\n"
			"{\n"
			"   gl_FragColor = vColor;\n"
			"}");
	}
	// Textured quad
	else if (shadername == "texturedQuad")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"attribute highp vec2 texcoord;\n"
			"varying vec2 v_texCoords;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = vertex;\n"
			"   v_texCoords = texcoord;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform sampler2D texture; \n"
			"varying vec2 v_texCoords;\n"
			"void main() \n"
			"{ \n"
			"    gl_FragColor = texture2D(texture, v_texCoords); \n"
			"}\n");
	}
	// Mesh
	else if (shadername == "mesh")
	{
		//auto program = new QOpenGLShaderProgram(context());
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"#version 330 core\n"
			"layout (location = 0) in vec4 vertex;\n"
			"layout (location = 1) in vec4 normal;\n"
			"layout (location = 2) in vec4 color;\n"
			"uniform mat4 matrix;\n"
			"out vec3 FragPos;\n"
			"out vec3 Normal;\n"
			"out vec4 Color;\n"
			"void main(void)\n"
			"{\n"
			"   gl_Position = matrix * vertex;\n"
			"   FragPos = vertex.xyz;\n"
			"   Normal = normal.xyz;\n"
			"   Color = color;\n"
			"}");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"#version 330 core\n"
			"out vec4 color;\n"
			"in vec3 FragPos;\n"
			"in vec3 Normal;\n"
			"in vec4 Color;\n"
			"uniform vec3 lightPos;\n"
			"uniform vec3 viewPos;\n"
			"uniform vec3 lightColor;\n"
			"void main(void)\n"
			"{\n"
			"    // Ambient \n"
			"    float ambientStrength = 0.2f; \n"
			"    vec3 ambient = ambientStrength * lightColor; \n"
			"    \n"
			"    // Diffuse \n"
			"    vec3 norm = normalize(Normal); \n"
			"    vec3 lightDir = normalize(lightPos - FragPos); \n"
			"    float diff = max(dot(norm, lightDir), 0.0); \n"
			"    vec3 diffuse = diff * lightColor; \n"
			"    \n"
			"    // Specular \n"
			"    vec3 fakeLightDir = normalize(vec3(0.5,0.5,1));\n"
			"    float specularStrength = 1.0f; \n"
			"    vec3 viewDir = normalize(viewPos - FragPos); \n"
			"    vec3 reflectDir = reflect(-fakeLightDir, norm);  \n"
			"    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64); \n"
			"    vec3 specular = specularStrength * spec * lightColor;   \n"
			"    \n"
			"    vec3 result = (ambient + diffuse + specular) * Color.xyz; \n"
			"    color = vec4(result, Color.w); \n"
			"}");
	}
	else if (shadername == "texturedPlane")
	{
		program->addShaderFromSourceCode(QOpenGLShader::Vertex,
			"attribute highp vec4 vertex;\n"
			"attribute mediump vec4 texCoord;\n"
			"varying mediump vec4 texc;\n"
			"uniform mediump mat4 matrix;\n"
			"void main(void)\n"
			"{\n"
			"    gl_Position = matrix * vertex;\n"
			"    texc = texCoord;\n"
			"}\n");
		program->addShaderFromSourceCode(QOpenGLShader::Fragment,
			"uniform sampler2D texture;\n"
			"varying mediump vec4 texc;\n"
			"void main(void)\n"
			"{\n"
			"    gl_FragColor = texture2D(texture, texc.st);\n"
			"}\n");
	}
	else
	{
		delete program;
		program = nullptr;
	}

	// link shaders
	if (program) program->link();

	return program;
}

void Viewer::drawPoints(const QVector< QVector3D > & points, float& pointSize, QColor color, QMatrix4x4 camera, bool isConnected)
{
    if(points.empty()) return;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_POINT_SMOOTH);
	glPointSize(pointSize);

    auto & program = *shaderPrograms["points"];

    // Activate shader
    program.bind();

    int vertexLocation = program.attributeLocation("vertex");
    int matrixLocation = program.uniformLocation("matrix");
    int colorLocation = program.uniformLocation("color");

    // Shader data
    program.enableAttributeArray(vertexLocation);
	program.setAttributeArray(vertexLocation, points.data());
    program.setUniformValue(matrixLocation, camera);
    program.setUniformValue(colorLocation, color);

    // Draw points
    if(isConnected) glDrawArrays(GL_LINE_STRIP, 0, points.size());
    glDrawArrays(GL_POINTS, 0, points.size());

    program.disableAttributeArray(vertexLocation);
    program.release();

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);

    glDisable(GL_POINT_SMOOTH);
    glPointSize(1.0f);
}

void Viewer::drawOrientedPoints(const QVector< QVector3D > & points, 
	const QVector< QVector3D > & normals, QColor useColor, QMatrix4x4 camera)
{
	if (points.empty() || normals.empty()) return;

	// Draw meshes:
	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_BACK);

	// Activate shader
	auto & program = *shaderPrograms["mesh"];
	program.bind();

	// Attributes
	int vertexLocation = program.attributeLocation("vertex");
	int normalLocation = program.attributeLocation("normal");
	int colorLocation = program.attributeLocation("color");

	program.enableAttributeArray(vertexLocation);
	program.enableAttributeArray(normalLocation);
	program.enableAttributeArray(colorLocation);

	// Pack geometry, normals, and colors
	QVector<GLfloat> vertex, normal, color;

	for (int v = 0; v < points.size(); v++){
		for (int i = 0; i < 3; i++){
			vertex << points[v][i];
			normal << normals[v][i];
		}
		color << useColor.redF() << useColor.greenF() << useColor.blueF() << useColor.alphaF();
	}

	// Shader data
	program.setAttributeArray(vertexLocation, &vertex[0], 3);
	program.setAttributeArray(normalLocation, &normal[0], 3);
	program.setAttributeArray(colorLocation, &color[0], 4);

	// Uniforms
	int matrixLocation = program.uniformLocation("matrix");
	int lightPosLocation = program.uniformLocation("lightPos");
	int viewPosLocation = program.uniformLocation("viewPos");
	int lightColorLocation = program.uniformLocation("lightColor");

    program.setUniformValue(matrixLocation, camera);
	//program.setUniformValue(lightPosLocation, eyePos);
	//program.setUniformValue(viewPosLocation, eyePos);
	program.setUniformValue(lightPosLocation, QVector3D(0, 0, 0));
	program.setUniformValue(viewPosLocation, QVector3D(0, 0, 0));
	program.setUniformValue(lightColorLocation, QVector3D(1, 1, 1));

	// Draw
	glDrawArrays(GL_POINTS, 0, points.size());

	program.disableAttributeArray(vertexLocation);
	program.disableAttributeArray(normalLocation);
	program.disableAttributeArray(colorLocation);

	program.release();

	glDisable(GL_DEPTH_TEST);
}

void Viewer::drawOrientedPoints(const QVector< QVector3D > & points,
	const QVector< QVector3D > & normals, const QVector<QVector3D> & colors, QMatrix4x4 camera)
{
	if (points.empty() || normals.empty() || colors.empty()) return;
	if (points.size() != normals.size() || points.size() != colors.size()) return;

	// Draw meshes:
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glCullFace(GL_BACK);

	// Activate shader
	auto & program = *shaderPrograms["mesh"];
	program.bind();

	// Attributes
	int vertexLocation = program.attributeLocation("vertex");
	int normalLocation = program.attributeLocation("normal");
	int colorLocation = program.attributeLocation("color");

	program.enableAttributeArray(vertexLocation);
	program.enableAttributeArray(normalLocation);
	program.enableAttributeArray(colorLocation);

	// Pack geometry, normals, and colors
	QVector<GLfloat> vertex, normal, color;

	for (int v = 0; v < points.size(); v++){
		for (int i = 0; i < 3; i++){
			vertex << points[v][i];
			normal << normals[v][i];
			color << colors[v][i];
		}
	}

	// Shader data
	program.setAttributeArray(vertexLocation, &vertex[0], 3);
	program.setAttributeArray(normalLocation, &normal[0], 3);
	program.setAttributeArray(colorLocation, &color[0], 3);

	// Uniforms
	int matrixLocation = program.uniformLocation("matrix");
	int lightPosLocation = program.uniformLocation("lightPos");
	int viewPosLocation = program.uniformLocation("viewPos");
	int lightColorLocation = program.uniformLocation("lightColor");

	program.setUniformValue(matrixLocation, camera);
	//program.setUniformValue(lightPosLocation, eyePos);
	//program.setUniformValue(viewPosLocation, eyePos);
	program.setUniformValue(lightPosLocation, QVector3D(0, 0, 0));
	program.setUniformValue(viewPosLocation, QVector3D(0, 0, 0));
	program.setUniformValue(lightColorLocation, QVector3D(1, 1, 1));

	// Draw
	glDrawArrays(GL_POINTS, 0, points.size());

	program.disableAttributeArray(vertexLocation);
	program.disableAttributeArray(normalLocation);
	program.disableAttributeArray(colorLocation);

	program.release();

	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
}

void Viewer::drawLines(const QVector< QVector3D > &lines, QColor color, QMatrix4x4 camera, QString shaderName)
{
    if(lines.empty()) return;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    auto & program = *shaderPrograms[shaderName];

    // Activate shader
    program.bind();

    int vertexLocation = program.attributeLocation("vertex");
    int matrixLocation = program.uniformLocation("matrix");
    int colorLocation = program.uniformLocation("color");

    // Pack geometry
    QVector<GLfloat> vertices;
    for(auto p : lines) {vertices.push_back(p.x()); vertices.push_back(p.y()); vertices.push_back(p.z());}

    // Shader data
    program.enableAttributeArray(vertexLocation);
    program.setAttributeArray(vertexLocation, &vertices[0], 3);
    program.setUniformValue(matrixLocation, camera);
    program.setUniformValue(colorLocation, color);

    // Draw lines
    glDrawArrays(GL_LINES, 0, vertices.size() / 3);

    program.disableAttributeArray(vertexLocation);

    program.release();

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}

void Viewer::drawBox(double width, double length, double height, QMatrix4x4 camera)
{
    if(width == 0 || length == 0 || height == 0) return;

    glEnable( GL_DEPTH_TEST );

    // Define the 8 vertices of a unit cube
    GLfloat g_Vertices[24] = {
          0.5,  0.5,  1,
         -0.5,  0.5,  1,
         -0.5, -0.5,  1,
          0.5, -0.5,  1,
          0.5, -0.5, 0,
         -0.5, -0.5, 0,
         -0.5,  0.5, 0,
          0.5,  0.5, 0
    };

    //GLfloat g_Colors[24] = {
    //     1, 1, 1,
    //     0, 1, 1,
    //     0, 0, 1,
    //     1, 0, 1,
    //     1, 0, 0,
    //     0, 0, 0,
    //     0, 1, 0,
    //     1, 1, 0,
    //};

	GLfloat g_Colors[24] = {
		1, 1, 0,
		1, 1, 0,
		1, 1, 0,
		1, 1, 0,
		1, 1, 0,
		1, 1, 0,
		1, 1, 0,
		1, 1, 0
	};

    // Define the vertex indices for the cube.
    GLuint g_Indices[24] = {
        0, 1, 2, 3,                 // Front face
        7, 4, 5, 6,                 // Back face
        6, 5, 2, 1,                 // Left face
        7, 0, 3, 4,                 // Right face
        7, 6, 1, 0,                 // Top face
        3, 2, 5, 4,                 // Bottom face
    };

    //glDrawElements( GL_QUADS, 24, GL_UNSIGNED_INT, &g_Indices[0] );

    // Pack geometry & colors
    QVector<GLfloat> vertices, colors;
    for(int i = 0; i < 24; i++)
    {
        int v = g_Indices[i] * 3;

        vertices.push_back(width * g_Vertices[v+0]);
        vertices.push_back(length * g_Vertices[v+1]);
        vertices.push_back(height * g_Vertices[v+2]);

        colors.push_back(g_Colors[v+0]);
        colors.push_back(g_Colors[v+1]);
        colors.push_back(g_Colors[v+2]);
        colors.push_back(0.2);
    }

    auto & program = *shaderPrograms["box"];

    // Activate shader
    program.bind();

    int vertexLocation = program.attributeLocation("vertex");
    int colorLocation = program.attributeLocation("color");
    int matrixLocation = program.uniformLocation("matrix");

    // Shader data
    program.enableAttributeArray(vertexLocation);
    program.setAttributeArray(vertexLocation, &vertices[0], 3);
    program.enableAttributeArray(colorLocation);
    program.setAttributeArray(colorLocation, &colors[0], 4);

    program.setUniformValue(matrixLocation, camera);

    // Draw faces
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDrawArrays(GL_QUADS, 0, 24);

    program.disableAttributeArray(vertexLocation);
    program.disableAttributeArray(colorLocation);

    program.release();

    glDisable( GL_DEPTH_TEST );
}

void Viewer::drawBox(const QVector<QVector3D> verts, const QColor &color, QMatrix4x4 camera)
{
	if (verts.size() != 8) return;

	glEnable(GL_DEPTH_TEST);

	// Define the vertex indices for the cube.
	GLuint g_Indices[24] = {
		0, 1, 2, 3,                 // Front face
		7, 4, 5, 6,                 // Back face
		6, 5, 2, 1,                 // Left face
		7, 0, 3, 4,                 // Right face
		7, 6, 1, 0,                 // Top face
		3, 2, 5, 4,                 // Bottom face
	};

	// Pack geometry & colors
	QVector<GLfloat> vertices, colors;
	for (int i = 0; i < 24; i++)
	{
		int v = g_Indices[i];

		vertices.push_back(verts[v].x());
		vertices.push_back(verts[v].y());
		vertices.push_back(verts[v].z());

		colors.push_back(color.red());
		colors.push_back(color.green());
		colors.push_back(color.blue());
		colors.push_back(0.2);
	}

	auto & program = *shaderPrograms["box"];

	// Activate shader
	program.bind();

	int vertexLocation = program.attributeLocation("vertex");
	int colorLocation = program.attributeLocation("color");
	int matrixLocation = program.uniformLocation("matrix");

	// Shader data
	program.enableAttributeArray(vertexLocation);
	program.setAttributeArray(vertexLocation, &vertices[0], 3);
	program.enableAttributeArray(colorLocation);
	program.setAttributeArray(colorLocation, &colors[0], 4);

	program.setUniformValue(matrixLocation, camera);

	// Draw faces
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawArrays(GL_QUADS, 0, 24);

	program.disableAttributeArray(vertexLocation);
	program.disableAttributeArray(colorLocation);

	program.release();

	glDisable(GL_DEPTH_TEST);
}

void Viewer::drawQuad(const QImage & img)
{
    if(img.isNull() || img.width() == 0 || img.height() == 0) return;

    QOpenGLTexture texture(img.mirrored());
    texture.setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
    texture.setMagnificationFilter(QOpenGLTexture::Linear);
    texture.bind();

    auto & program = *shaderPrograms["texturedQuad"];

    // Activate shader
    program.bind();

    // Draw quad
    float f = 1.0;
    GLfloat g_Vertices[] = {-f,  f ,0,
                            -f, -f ,0,
                             f,  f ,0,
                             f, -f ,0};

    GLfloat g_Texcoord[] = {0, 1,
                            0, 0,
                            1, 1,
                            1, 0};

    GLubyte g_Indices[] = {0,1,2, // first triangle (bottom left - top left - top right)
                           2,1,3}; // second triangle (bottom left - top right - bottom right)

    QVector<GLfloat> vertices, texcoord;
    for(int i = 0; i < 6; i++)
    {
        int v = g_Indices[i] * 3;
        vertices.push_back(g_Vertices[v+0]);
        vertices.push_back(g_Vertices[v+1]);
        vertices.push_back(g_Vertices[v+2]);

        int vt = g_Indices[i] * 2;
        texcoord.push_back(g_Texcoord[vt+0]);
        texcoord.push_back(g_Texcoord[vt+1]);
    }

    int vertexLocation = program.attributeLocation("vertex");
    int textureLocation = program.attributeLocation("texcoord");

    // Shader data
    program.enableAttributeArray(vertexLocation);
    program.setAttributeArray(vertexLocation, &vertices[0], 3);
    program.enableAttributeArray(textureLocation);
    program.setAttributeArray(textureLocation, &texcoord[0], 2);


    // Draw quad
    glDrawArrays(GL_TRIANGLES, 0, 6);

    program.release();

    texture.release();
}

void Viewer::drawPlane(QVector3D normal, QVector3D origin, QMatrix4x4 camera)
{
    if(normal.lengthSquared() == 0) return;

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    // Compute two arbitrary othogonal vectors
    auto orthogonalVector = [](const QVector3D& n) {
        if ((abs(n.y()) >= 0.9 * abs(n.x())) && abs(n.z()) >= 0.9 * abs(n.x())) return QVector3D(0.0, -n.z(), n.y());
        else if ( abs(n.x()) >= 0.9 * abs(n.y()) && abs(n.z()) >= 0.9 * abs(n.y()) ) return QVector3D(-n.z(), 0.0, n.x());
        else return QVector3D(-n.y(), n.x(), 0.0);
    };
    auto u = orthogonalVector(normal);
    auto v = QVector3D::crossProduct(normal, u);
    float scale = 0.75;

    auto & program = *shaderPrograms["translucent"];

    // Activate shader
    program.bind();
    int vertexLocation = program.attributeLocation("vertex");
    int matrixLocation = program.uniformLocation("matrix");
    int colorLocation = program.uniformLocation("color");
    program.enableAttributeArray(vertexLocation);
    program.setUniformValue(matrixLocation, camera);

    // Ray from origin
    {
        QColor color = Qt::green;
        QVector< QVector3D > points;
        points << origin << (origin + (normal * scale));
        QVector<GLfloat> vertices;
        for(auto p : points) {vertices.push_back(p.x()); vertices.push_back(p.y()); vertices.push_back(p.z());}
        program.enableAttributeArray(vertexLocation);
        program.setAttributeArray(vertexLocation, &vertices[0], 3);
        program.setUniformValue(colorLocation, color);
        glLineWidth(5);
        glDrawArrays(GL_LINES, 0, vertices.size() / 3);
    }

    // Rectangle
    {
        QVector< QVector3D > points;
        points << QVector3D(origin + (-u + v) * scale) <<
                  QVector3D(origin + (u + v) * scale) <<
                  QVector3D(origin + (-v + u) * scale) <<
                  QVector3D(origin + (-v + -u) * scale);

        // Pack geometry
        QVector<GLfloat> vertices;
        for(auto p : points) {
            vertices.push_back(p.x()); vertices.push_back(p.y()); vertices.push_back(p.z());
        }
        glLineWidth(5);
        program.setAttributeArray(vertexLocation, &vertices[0], 3);
        program.setUniformValue(colorLocation, QColor (0,0,255,255));
        glDrawArrays(GL_LINE_LOOP, 0, 4);

        program.setAttributeArray(vertexLocation, &vertices[0], 3);
        program.setUniformValue(colorLocation, QColor (0,0,255,80));
        glDrawArrays(GL_QUADS, 0, 4);
    }

    program.release();

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}

#include <QOpenGLBuffer>

void Viewer::drawTexturedPlane(const QImage &img, const QVector<QVector3D> verts, QMatrix4x4 camera) {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	QOpenGLTexture texture(img.mirrored());


	auto & program = *shaderPrograms["texturedPlane"];

	// Activate shader
	program.bindAttributeLocation("vertex", 0);
	program.bindAttributeLocation("texCoord", 1);
	program.link();
	program.bind();

	program.setUniformValue("texture", 0);

	QVector<GLfloat> vertices;

	for (int i = 0; i < 4; ++i) {
		// vertex position
		vertices.append(verts[i].x());
		vertices.append(verts[i].y());
		vertices.append(verts[i].z());

		// texture coordinate
		vertices.append(i == 0 || i == 3);
		vertices.append(i == 0 || i == 1);
	}

	for (int i = 0; i < 4; ++i) {
		// vertex position
		vertices.append(verts[4-1-i].x());
		vertices.append(verts[4 - 1 - i].y());
		vertices.append(verts[4 - 1 - i].z());

		// texture coordinate
		vertices.append(4 - 1 - i == 0 || 4 - 1 - i == 3);
		vertices.append(4 - 1 - i == 0 || 4 - 1 - i == 1);
	}

	QOpenGLBuffer vbo;
	vbo.create();
	vbo.bind();
	vbo.allocate(vertices.constData(), vertices.count() * sizeof(GLfloat));

	program.setUniformValue("matrix", camera);
	program.enableAttributeArray(0);
	program.enableAttributeArray(1);
	program.setAttributeBuffer(0, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
	program.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

	texture.bind();
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	texture.bind();
	glDrawArrays(GL_TRIANGLE_FAN, 4, 4);

	//int vertexLocation = program.attributeLocation("vertex");
	//int matrixLocation = program.uniformLocation("matrix");
	////int colorLocation = program.uniformLocation("color");
	//int textureLocation = program.attributeLocation("texture");


	//// Pack geometry
	//QVector<GLfloat> vertices, textures;

	//for (int i = 0; i < 4; ++i) {
	//	// vertex position
	//	vertices.append(verts[i].x());
	//	vertices.append(verts[i].y());
	//	vertices.append(verts[i].z());

	//	// texture coordinate
	//	textures.append(i == 0 || i == 3);
	//	textures.append(i == 0 || i == 1);
	//}

	//// Shader data
	//program.enableAttributeArray(vertexLocation);
	//program.setAttributeArray(vertexLocation, &vertices[0], 3);
	//program.enableAttributeArray(textureLocation);
	//program.setAttributeArray(textureLocation, &textures[0], 2);
	//program.setUniformValue(matrixLocation, camera);
	////program.setUniformValue(colorLocation, color);

	//// Draw plane
	//texture.bind();
	//glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//program.disableAttributeArray(vertexLocation);

	program.release();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
}

void Viewer::drawTriangles(QColor useColor, const QVector<QVector3D> &points,
                           const QVector<QVector3D> &normals, QMatrix4x4 pvm)
{
    if(points.size() < 3 || normals.size() < 3) return;

    // Draw meshes:
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);

    // Activate shader
    auto & program = *shaderPrograms["mesh"];
    program.bind();

    // Attributes
    int vertexLocation = program.attributeLocation("vertex");
    int normalLocation = program.attributeLocation("normal");
    int colorLocation = program.attributeLocation("color");

    program.enableAttributeArray(vertexLocation);
    program.enableAttributeArray(normalLocation);
    program.enableAttributeArray(colorLocation);

    // Uniforms
    int matrixLocation = program.uniformLocation("matrix");
    int lightPosLocation = program.uniformLocation("lightPos");
    int viewPosLocation = program.uniformLocation("viewPos");
    int lightColorLocation = program.uniformLocation("lightColor");

    program.setUniformValue(matrixLocation, pvm);
    program.setUniformValue(lightPosLocation, eyePos);
    program.setUniformValue(viewPosLocation, eyePos);
    program.setUniformValue(lightColorLocation, QVector3D(1,1,1));

    // Pack geometry, normals, and colors
    QVector<GLfloat> vertex, normal, color;

    for(int vf = 0; vf < points.size(); vf++){
        for(int i = 0; i < 3; i++){
            vertex << points[vf][i];
            normal << normals[vf][i];
        }
        color << useColor.redF() << useColor.greenF() << useColor.blueF() << useColor.alphaF();
    }

    // Shader data
    program.setAttributeArray(vertexLocation, &vertex[0], 3);
    program.setAttributeArray(normalLocation, &normal[0], 3);
    program.setAttributeArray(colorLocation, &color[0], 4);

    // Draw
    glDrawArrays(GL_TRIANGLES, 0, points.size());

    program.disableAttributeArray(vertexLocation);
    program.disableAttributeArray(normalLocation);
    program.disableAttributeArray(colorLocation);

    program.release();

    glDisable(GL_DEPTH_TEST);
}

void Viewer::drawRenderingImgs(Vector4u* freeRenderImg, Vector2i freeRenderImgSize, unsigned int id1,
	Vector4u* renderImg, Vector2i renderImgSize, unsigned int id2,
	Vector4u* rgbImg, Vector2i rgbImgSize, unsigned int id3,
	Vector4u* depthImg, Vector2i depthImgSize, unsigned int id4)
{
	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			//free rendering image
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, id1);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, freeRenderImgSize.x, freeRenderImgSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, freeRenderImg);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glBegin(GL_QUADS); {
				glTexCoord2f(0, 1); glVertex2f(0, 0); // glVertex2f(0, 0);
				glTexCoord2f(1, 1); glVertex2f(1, 0); // glVertex2f(1, 0);
				glTexCoord2f(1, 0); glVertex2f(1, 0.6); // glVertex2f(1, 1);
				glTexCoord2f(0, 0); glVertex2f(0, 0.6); // glVertex2f(0, 1);
			}

			glEnd();
			glDisable(GL_TEXTURE_2D);

			//rendering image
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, id1);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, renderImgSize.x, renderImgSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, renderImg);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glBegin(GL_QUADS); {
				glTexCoord2f(0, 1); glVertex2f(0, 0.6); // glVertex2f(0, 0);
				glTexCoord2f(1, 1); glVertex2f(0.7, 0.6); // glVertex2f(1, 0);
				glTexCoord2f(1, 0); glVertex2f(0.7, 1); // glVertex2f(1, 1);
				glTexCoord2f(0, 0); glVertex2f(0, 1); // glVertex2f(0, 1);
			}

			glEnd();
			glDisable(GL_TEXTURE_2D);


			//rgb image
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, id1);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImgSize.x, rgbImgSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbImg);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glBegin(GL_QUADS); {
				glTexCoord2f(0, 1); glVertex2f(0.7, 0.8); // glVertex2f(0, 0);
				glTexCoord2f(1, 1); glVertex2f(1, 0.8); // glVertex2f(1, 0);
				glTexCoord2f(1, 0); glVertex2f(1, 1); // glVertex2f(1, 1);
				glTexCoord2f(0, 0); glVertex2f(0.7, 1); // glVertex2f(0, 1);
			}

			glEnd();
			glDisable(GL_TEXTURE_2D);

			//depth image
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, id1);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, depthImgSize.x, depthImgSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, depthImg);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glBegin(GL_QUADS); {
				glTexCoord2f(0, 1); glVertex2f(0.7, 0.6); // glVertex2f(0, 0);
				glTexCoord2f(1, 1); glVertex2f(1, 0.6); // glVertex2f(1, 0);
				glTexCoord2f(1, 0); glVertex2f(1, 0.8); // glVertex2f(1, 1);
				glTexCoord2f(0, 0); glVertex2f(0.7, 0.8); // glVertex2f(0, 1);
			}

			glEnd();
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glFlush();
}

