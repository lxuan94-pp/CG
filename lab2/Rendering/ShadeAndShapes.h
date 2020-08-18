#ifndef SHADE_AND_SHAPES_H
#define SHADE_AND_SHAPES_H

#include "Rendering/Operator.h"
#include "Rendering/Geometry.h"
#include "Rendering/Scene.h"

#include "Common/Common.h"
#include "Common/Matrix.h"
#include "Rendering/Scene.h"

class Material : public SceneObject {
protected:
	Color _ambient, _diffuse, _specular;
	double _specExp, _reflective, _transp, _refractInd;
public:
	inline Material() {};

	inline const Color& getAmbient() const { return _ambient; }
	inline const Color& getDiffuse() const { return _diffuse; }
	inline const Color& getSpecular() const { return _specular; }
	inline double getSpecExponent() const { return _specExp; }
	inline double getReflective() const { return _reflective; }
	inline double getTransparency() const { return _transp; }
	inline double getRefractIndex() const { return _refractInd; }

	inline void setAmbient(const Color& amb) { _ambient = amb; }
	inline void setDiffuse(const Color& diff) { _diffuse = diff; }
	inline void setSpecular(const Color& spec) { _specular = spec; }
	inline void setSpecExponent(double s) { _specExp = s; }
	inline void setReflective(double r) { _reflective = r; }
	inline void setTransparency(double t) { _transp = t; }
	inline void setRefractIndex(double r) { _refractInd = r; }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Light : public SceneObject {
protected:
	unsigned int _id;
	Pt3 _pos;
	Color _color;
	Color _ambient; // Keeping ambient here makes it easier to code than using a global ambient
public:
	inline Light() : _color(Color(1, 1, 1)) {} // Default: white
	inline Light(const Pt3& pos, const Color& color) : _pos(pos), _color(color) {}

	inline const Pt3& getPos() const { return _pos; }
	inline const Color& getColor() const { return _color; }
	inline const Color& getAmbient() const { return _ambient; }
	inline unsigned int getId() { return _id; }

	inline void setPos(const Pt3& p) { _pos = p; }
	inline void setColor(const Color& c) { _color = c; }
	inline void setAmbient(const Color& c) { _ambient = c; }
	inline void setId(unsigned int id) { _id = id; }

	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};


//========================================================================
// Shapes
//========================================================================

class Sphere : public Geometry, public Operand {
protected:
	Pt3 _center;
	double _rad;
public:
	Sphere() {};
	Sphere(const Pt3& c, double r) : _center(c), _rad(r) {}

	// set/get methods
	double getRadius() { return _rad; }
	Pt3 getCenter() { return _center; }

	void setRadius(double r) { _rad = r; }
	void setCenter(const Pt3& c) { _center = c; }

	void translate(const Vec3& trans);
	void rotate(double d, int axis);
	void updateTransform();

	virtual void accept(GeometryVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Box : public Geometry, public Operand {
protected:
	// NOTE: The corner and center for boxes are defined for your convenience but may be redundant.
	// You are free to implement the box or any other shape however you want. Just make sure that it works.

	Pt3 _corner;
	Pt3 _center;
	Vec3 _lengthv;
	Vec3 _widthv;
	Vec3 _heightv;

	double _length;
	double _width;
	double _height;

public:
	Box() {};
	Box(
		const Pt3& corner,
		const Vec3& lv,
		const Vec3& wv,
		const Vec3& hv,
		double l,
		double w,
		double h
	) {
		_corner = corner;
		_lengthv = lv;
		_widthv = wv;
		_heightv = hv;
		_length = l;
		_width = w;
		_height = h;
		updateTransform();
	}

	// set/get methods
	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	Pt3 getCorner() { return _corner; }

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCorner(const Pt3& v) { _corner = v; }

	double getLength() { return _length; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }

	void setLength(double l) { _length = l; }
	void setWidth(double w) { _width = w; }
	void setHeight(double h) { _height = h; }

	Pt3 getCenter() { return _center; }
	void translate(const Vec3& trans);
	void rotate(double d, int axis);
	void updateTransform();

	virtual void accept(GeometryVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Ellipsoid : public Geometry, public Operand {
protected:
	Pt3 _center;
	Vec3 _lengthv;
	Vec3 _widthv;
	Vec3 _heightv;

	double _length;
	double _width;
	double _height;

public:
	Ellipsoid() {};
	Ellipsoid(
		const Pt3& center,
		const Vec3& lv,
		const Vec3& wv,
		const Vec3& hv,
		double l,
		double w,
		double h
	) {
		_center = center;
		_lengthv = lv;
		_widthv = wv;
		_heightv = hv;
		_length = l;
		_width = w;
		_height = h;
		updateTransform();
	}

	// set/get methods
	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	Pt3 getCenter() { return _center; }

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCenter(const Pt3& v) { _center = v; }

	double getLength() { return _length; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }

	void setLength(double l) { _length = l; }
	void setWidth(double w) { _width = w; }
	void setHeight(double h) { _height = h; }

	void translate(const Vec3& trans);
	void rotate(double d, int axis);
	void updateTransform();

	virtual void accept(GeometryVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Cylinder : public Geometry, public Operand {
protected:
	Pt3 _center;
	Vec3 _lengthv;
	Vec3 _widthv;
	Vec3 _heightv;

	double _length;
	double _width;
	double _height;

public:
	Cylinder() {};
	Cylinder(
		const Pt3& center,
		const Vec3& lv,
		const Vec3& wv,
		const Vec3& hv,
		double l,
		double w,
		double h
	) {
		_center = center;
		_lengthv = lv;
		_widthv = wv;
		_heightv = hv;
		_length = l;
		_width = w;
		_height = h;
		updateTransform();
	}

	// set/get method
	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	Pt3 getCenter() { return _center; }

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCenter(const Pt3& v) { _center = v; }

	double getLength() { return _length; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }

	void setLength(double l) { _length = l; }
	void setWidth(double w) { _width = w; }
	void setHeight(double h) { _height = h; }

	void translate(const Vec3& trans);
	void rotate(double d, int axis);
	void updateTransform();

	virtual void accept(GeometryVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Cone : public Geometry, public Operand {
protected:
	Pt3 _center;
	Vec3 _lengthv;
	Vec3 _widthv;
	Vec3 _heightv;

	double _length;
	double _width;
	double _height;

public:
	Cone() {};
	Cone(
		const Pt3& center,
		const Vec3& lv,
		const Vec3& wv,
		const Vec3& hv,
		double l,
		double w,
		double h
	) {
		_center = center;
		_lengthv = lv;
		_widthv = wv;
		_heightv = hv;
		_length = l;
		_width = w;
		_height = h;
		updateTransform();
	}

	// set/get methods
	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	Pt3 getCenter() { return _center; }

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCenter(const Pt3& v) { _center = v; }

	double getLength() { return _length; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }

	void setLength(double l) { _length = l; }
	void setWidth(double w) { _width = w; }
	void setHeight(double h) { _height = h; }

	void translate(const Vec3& trans);
	void rotate(double d, int axis);
	void updateTransform();

	virtual void accept(GeometryVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};


//========================================================================
// END OF Shapes
//========================================================================


struct IsectData {
	bool hit; /* Intersection happened */
	double t; /* Parameter */
	Vec3 normal; /* Normal vector at intersection */
	IsectData() : hit(false), t(DINF) {}
};

struct IsectAxisData {
	bool hit;
	int axis;
	IsectAxisData() : hit(false) {}
};

class Intersector : public GeometryVisitor {
protected:
	Ray _ray;
public:
	Intersector() : _ray(Pt3(0, 0, 0), Vec3(1, 0, 0, 0)) {}
	void setRay(const Ray& r) { _ray = r; }
	virtual void visit(Sphere* sphere, void* ret);
	virtual void visit(Box* op, void* ret);
	virtual void visit(Ellipsoid* op, void* ret);
	virtual void visit(Cylinder* op, void* ret);
	virtual void visit(Cone* op, void* ret);
	virtual void visit(Operator* op, void* ret);
};

#endif
