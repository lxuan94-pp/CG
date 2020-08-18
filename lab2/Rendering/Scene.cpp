#include "Rendering/Scene.h"
#include "Rendering/ShadeAndShapes.h"
#include "Rendering/Shading.h"
#include "Rendering/Geometry.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <FL/gl.h>

using namespace std;

Vec3 readVec3(istream* stream) {
	Vec3 ret(0, 0, 0, 0);
	(*stream)>>ret[0]>>ret[1]>>ret[2];
	return ret;
}

Pt3 readPt3(istream* stream) {
	Pt3 ret(0, 0, 0);
	(*stream)>>ret[0]>>ret[1]>>ret[2];
	return ret;
}

double readDouble(istream* stream) {
	double ret;
	(*stream)>>ret;
	return ret;
}

int readInt(istream* stream) {
	int ret;
	(*stream)>>ret;
	return ret;
}

string readString(istream* stream) {
	string ret;
	(*stream)>>ret;
	return ret;
}

void writeVec3(ostream* stream, const Vec3& v) {
	(*stream) << v[0] << " " << v[1] << " " << v[2];
}

void writePt3(ostream* stream, const Pt3& v) {
	(*stream) << v[0] << " " << v[1] << " " << v[2];
}

void writeDouble(ostream* stream, double d) {
	(*stream) << d;
}

void writeInt(ostream* stream, int d) {
	(*stream) << d;
}

void writeString(ostream* stream, const string& s) {
	(*stream) << s;
}


bool SceneUtils::writeScene(const std::string& fname, Scene* scene) {
	std::ofstream fout(fname.c_str());
	fout << setprecision(12); // up to 12 decimal places

	if(scene && fout.good()) {
		int nlights = scene->getNumLights();
		if(nlights <= 0) return false;

		WriteSceneObjectVisitor writer;
		writer.setStream(&fout);

		Color amb = scene->getLight(0)->getAmbient();
		writePt3(&fout, amb);
		fout << endl; // this is for reading purposes

		writeInt(&fout, nlights);
		fout << endl;
		for(int j = 0; j < nlights; j++)
			writer.visit(scene->getLight(j), NULL);
		fout << endl;

		writeInt(&fout, scene->getNumObjects());
		fout << endl << endl;
		for(int j = 0; j < scene->getNumObjects(); j++) {
			Geometry* geom = scene->getObject(j);
			Material* material = scene->getMaterial(geom);
			geom->accept(&writer, NULL);
			material->accept(&writer, NULL);
			fout << endl;
		}
	}
	fout << endl;

	Mat4* rot = scene->getRotate();
	Mat4* trans = scene->getTranslate();

	for(int j = 0; j < 4; j++)
		for(int k = 0; k < 4; k++)
			fout << (*rot)[j][k] << " ";

	for(int i = 0; i < 3; i++) fout << (*trans)[3][i] << ' ';
	fout << " // camera description" << endl;

	return false;
}

// no error handling at all.  pretty unsafe.  please make sure your file is well-formed.
Scene* SceneUtils::readScene(const std::string& fname) {
	Scene* ret = NULL;

	std::fstream fin(fname.c_str());
	std::stringstream ss;
	std::string line;

	if(fin.good()) {
		ret = new Scene();

		while(!fin.eof()) {
			getline(fin, line);
			size_t pos = line.find("//");
			if(pos == string::npos)
				ss << line;
			else
				ss << line.substr(0, pos);
			ss << endl;
		}

		ReadSceneObjectVisitor reader;
		reader.setStream(&ss);

		Color amb = readPt3(&ss);

		int numLights = readInt(&ss);
		for(int j = 0; j < numLights; j++) {
			Light* l = new Light();
			l->setId(GL_LIGHT0+j);
			reader.visit(l, NULL);
			l->setAmbient(amb);
			ret->addLight(l);
		}

		int numObjs = readInt(&ss);
		for(int j = 0; j < numObjs; j++) {
			string type = readString(&ss);
			Geometry* geom = NULL;
			if(type == "sphere")
				geom = new Sphere();
			else if(type == "ellipsoid")
				geom = new Ellipsoid();
			else if(type == "box")
				geom = new Box();
			else if(type == "cylinder")
				geom = new Cylinder();
			else if(type == "cone")
				geom = new Cone();

			geom->accept(&reader, NULL);

			Material* mat = new Material();
			mat->accept(&reader, NULL);

			ret->addObject(geom);
			ret->attachMaterial(geom, mat);
		}

		Mat4* rot = ret->getRotate();
		Mat4* trans = ret->getTranslate();

		for(int j = 0; j < 4; j++)
			for(int k = 0; k < 4; k++)
				(*rot)[j][k] = readDouble(&ss);

		for(int j = 0; j < 3; j++)
			(*trans)[3][j] = readDouble(&ss);
	}

	return ret;
}


//========================================================================
// ReadSceneObjectVisitors
//========================================================================

void ReadSceneObjectVisitor::visit(Material* mat, void* ret) {
	mat->setAmbient(readPt3(_stream));
	mat->setDiffuse(readPt3(_stream));
	mat->setSpecular(readPt3(_stream));
	mat->setSpecExponent(readDouble(_stream));
	mat->setReflective(readDouble(_stream));
	mat->setTransparency(readDouble(_stream));
	mat->setRefractIndex(readDouble(_stream));
}

void ReadSceneObjectVisitor::visit(Light* light, void* ret) {
	Pt3 pos = readPt3(_stream);
	Color color = readPt3(_stream);
	light->setPos(pos);
	light->setColor(color);
}

void ReadSceneObjectVisitor::visit(Box* box, void* ret) {
	box->setCorner(readPt3(_stream)); //corner
	box->setLengthVec(readVec3(_stream)); //v1
	box->setWidthVec(readVec3(_stream)); //v2
	box->setHeightVec(readVec3(_stream)); //v3
	box->setLength(readDouble(_stream)); //l1
	box->setWidth(readDouble(_stream)); //l2
	box->setHeight(readDouble(_stream)); //l3
	box->updateTransform();
}

void ReadSceneObjectVisitor::visit(Sphere* sphere, void* ret) {
	sphere->setCenter(readPt3(_stream));
	sphere->setRadius(readDouble(_stream));
}

void ReadSceneObjectVisitor::visit(Ellipsoid* ellipsoid, void* ret) {
	ellipsoid->setCenter(readPt3(_stream)); //center
	ellipsoid->setLengthVec(readVec3(_stream)); //v1
	ellipsoid->setWidthVec(readVec3(_stream)); //v2
	ellipsoid->setHeightVec(readVec3(_stream)); //v3
	ellipsoid->setLength(readDouble(_stream)); //l1
	ellipsoid->setWidth(readDouble(_stream)); //l2
	ellipsoid->setHeight(readDouble(_stream)); //l3
	ellipsoid->updateTransform();
}

void ReadSceneObjectVisitor::visit(Cylinder* cylinder, void* ret) {
	cylinder->setCenter(readPt3(_stream)); //center
	cylinder->setLengthVec(readVec3(_stream)); //v1
	cylinder->setWidthVec(readVec3(_stream)); //v2
	cylinder->setHeightVec(readVec3(_stream)); //v3
	cylinder->setLength(readDouble(_stream)); //l1
	cylinder->setWidth(readDouble(_stream)); //l2
	cylinder->setHeight(readDouble(_stream)); //l3
	cylinder->updateTransform();
}

void ReadSceneObjectVisitor::visit(Cone* cone, void* ret) {
	cone->setCenter(readPt3(_stream)); //center
	cone->setLengthVec(readVec3(_stream)); //v1
	cone->setWidthVec(readVec3(_stream)); //v2
	cone->setHeightVec(readVec3(_stream)); //v3
	cone->setLength(readDouble(_stream)); //l1
	cone->setWidth(readDouble(_stream)); //l2
	cone->setHeight(readDouble(_stream)); //l3
	cone->updateTransform();
}


//========================================================================
// WriteSceneObjectVisitors
//========================================================================

void WriteSceneObjectVisitor::visit(Material* mat, void* ret) {
	writePt3(_stream, mat->getAmbient());
	(*_stream) << endl;
	writePt3(_stream, mat->getDiffuse());
	(*_stream) << endl;
	writePt3(_stream, mat->getSpecular());
	(*_stream) << endl;
	writeDouble(_stream, mat->getSpecExponent());
	(*_stream) << endl;
	writeDouble(_stream, mat->getReflective());
	(*_stream) << endl;
	writeDouble(_stream, mat->getTransparency());
	(*_stream) << endl;
	writeDouble(_stream, mat->getRefractIndex());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Light* light, void* ret) {
	writePt3(_stream, light->getPos());
	(*_stream) << endl;
	writePt3(_stream, light->getColor());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Box* op, void* ret) {
	writeString(_stream, "box"); (*_stream) << endl;
	writePt3(_stream, op->getCorner());
	(*_stream) << endl;
	writeVec3(_stream, op->getLengthVec());
	(*_stream) << endl;
	writeVec3(_stream, op->getWidthVec());
	(*_stream) << endl;
	writeVec3(_stream, op->getHeightVec());
	(*_stream) << endl;
	writeDouble(_stream, op->getLength());
	(*_stream) << endl;
	writeDouble(_stream, op->getWidth());
	(*_stream) << endl;
	writeDouble(_stream, op->getHeight());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Sphere* sphere, void* ret) {
	writeString(_stream, "sphere"); (*_stream) << endl;
	writePt3(_stream, sphere->getCenter());
	(*_stream) << endl;
	writeDouble(_stream, sphere->getRadius());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Ellipsoid* ellipsoid, void* ret) {
	writeString(_stream, "ellipsoid"); (*_stream) << endl;
	writePt3(_stream, ellipsoid->getCenter());
	(*_stream) << endl;
	writeVec3(_stream, ellipsoid->getLengthVec());
	(*_stream) << endl;
	writeVec3(_stream, ellipsoid->getWidthVec());
	(*_stream) << endl;
	writeVec3(_stream, ellipsoid->getHeightVec());
	(*_stream) << endl;
	writeDouble(_stream, ellipsoid->getLength());
	(*_stream) << endl;
	writeDouble(_stream, ellipsoid->getWidth());
	(*_stream) << endl;
	writeDouble(_stream, ellipsoid->getHeight());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Cylinder* cylinder, void* ret) {
	writeString(_stream, "cylinder"); (*_stream) << endl;
	writePt3(_stream, cylinder->getCenter());
	(*_stream) << endl;
	writeVec3(_stream, cylinder->getLengthVec());
	(*_stream) << endl;
	writeVec3(_stream, cylinder->getWidthVec());
	(*_stream) << endl;
	writeVec3(_stream, cylinder->getHeightVec());
	(*_stream) << endl;
	writeDouble(_stream, cylinder->getLength());
	(*_stream) << endl;
	writeDouble(_stream, cylinder->getWidth());
	(*_stream) << endl;
	writeDouble(_stream, cylinder->getHeight());
	(*_stream) << endl;
}

void WriteSceneObjectVisitor::visit(Cone* cone, void* ret) {
	writeString(_stream, "cone"); (*_stream) << endl;
	writePt3(_stream, cone->getCenter());
	(*_stream) << endl;
	writeVec3(_stream, cone->getLengthVec());
	(*_stream) << endl;
	writeVec3(_stream, cone->getWidthVec());
	(*_stream) << endl;
	writeVec3(_stream, cone->getHeightVec());
	(*_stream) << endl;
	writeDouble(_stream, cone->getLength());
	(*_stream) << endl;
	writeDouble(_stream, cone->getWidth());
	(*_stream) << endl;
	writeDouble(_stream, cone->getHeight());
	(*_stream) << endl;
}
