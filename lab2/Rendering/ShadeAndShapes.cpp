#include "Common/Common.h"
#include "Rendering/ShadeAndShapes.h"

// Returns the rotation matrix for rotation around the "axis" by "d" radians
Mat4 axisRotationMatrix(double d, int axis) {
	int id0, id1;
	switch(axis) {
		case OP_XAXIS: id0 = 1; id1 = 2; break; // X -> Y,Z
		case OP_YAXIS: id0 = 2; id1 = 0; break; // Y -> Z,X
		case OP_ZAXIS: id0 = 0; id1 = 1; break; // Z -> X,Y
		default: assert(false);
	}

	Mat4 m; // identity
	double c = cos(d);
	double s = sin(d);
	m[id0][id0] = c;
	m[id0][id1] = -s;
	m[id1][id0] = s;
	m[id1][id1] = c;
	return move(m);
}

//========================================================================
// translate() and rotate()
//========================================================================

void Sphere::translate(const Vec3& trans) {
	_center += trans;
	// No need to update the matrices for spheres
}
// Rotation doesn't affect a sphere
void Sphere::rotate(double r, int axis) {}

void Box::translate(const Vec3& trans) {
	_corner += trans;
	updateTransform(); // Must call this to update the inner matrices
}

void Box::rotate(double d, int axis) {
	// NOTE: The rotation code for the other objects should look very similar.
	Mat4 m = axisRotationMatrix(d, axis);
	Vec3 cv = _corner - _center;

	// rotate the three linearly independent vectors that determine the shape
	_lengthv = _lengthv * m;
	_widthv = _widthv * m;
	_heightv = _heightv * m;

	// rotation is about the center, so points that are not the center will also be rotated;
	_corner = _center + (cv * m);

	updateTransform();
}

void Ellipsoid::translate(const Vec3& trans) {
	_center += trans;
	updateTransform();
}

void Ellipsoid::rotate(double d, int axis) {
	Mat4 m = axisRotationMatrix(d, axis);

	_lengthv = _lengthv * m;
	_widthv = _widthv * m;
	_heightv = _heightv * m;

	updateTransform();
}

void Cylinder::translate(const Vec3& trans) {
	_center += trans;
	updateTransform();
}

void Cylinder::rotate(double d, int axis) {
	Mat4 m = axisRotationMatrix(d, axis);

	_lengthv = _lengthv * m;
	_widthv = _widthv * m;
	_heightv = _heightv * m;

	updateTransform();
}

void Cone::translate(const Vec3& trans) {
	_center += trans;
	updateTransform();
}

void Cone::rotate(double d, int axis) {
	Mat4 m = axisRotationMatrix(d, axis);

	_lengthv = _lengthv * m;
	_widthv = _widthv * m;
	_heightv = _heightv * m;

	updateTransform();
}

//========================================================================
// updateTransform() and Intersector::visit()
//========================================================================

// The operator is the widget that allows you to translate and rotate a geometric object
// It is colored as red/green/blue.  When one of the axis is highlighted, it becomes yellow.
void Intersector::visit(Operator* op, void* ret) {
	IsectAxisData* iret = (IsectAxisData*) ret;
	Pt3 center = op->getPrimaryOp()->getCenter();
	iret->hit = false;
	const int axes[3] = { OP_XAXIS, OP_YAXIS, OP_ZAXIS };

	if(op->getState() == OP_TRANSLATE) {
		const Ray rays[3] = {
			Ray(center, op->getDirX()), // X
			Ray(center, op->getDirY()), // Y
			Ray(center, op->getDirZ()) // Z
		};

		double bestTime = 0.1;
		for(int i = 0; i < 3; i++) {
			double hitTime = GeometryUtils::pointRayDist(rays[i].p + 0.5*rays[i].dir, _ray);
			if(bestTime > hitTime) {
				bestTime = hitTime;
				iret->hit = true;
				iret->axis = axes[i];
			}
		}
	} else if(op->getState() == OP_ROTATE) {
		const Plane planes[3] = {
			Plane(center, Vec3(1, 0, 0, 0)), // X: YZ-plane
			Plane(center, Vec3(0, 1, 0, 0)), // Y: XZ-plane
			Plane(center, Vec3(0, 0, 1, 0)) // Z: XY-plane
		};

		double bestTime = 0.1;
		for(int i = 0; i < 3; i++) {
			double timeTemp = GeometryUtils::planeRay(planes[i], _ray);
			double hitTime = abs(mag(_ray.at(timeTemp)-center) - OP_STEP);
			if(bestTime > hitTime) {
				bestTime = hitTime;
				iret->hit = true;
				iret->axis = axes[i];
			}
		}
	}
}


// The definition of a sphere can be pretty sparse,
// so you don't need to define the transform associated with a sphere.
void Sphere::updateTransform() {}

void Intersector::visit(Sphere* sphere, void* ret) {
	IsectData* iret = (IsectData*) ret;

	Pt3 center = sphere->getCenter();
	double radius = sphere->getRadius();

	// If Dist > R, fast reject
	if (GeometryUtils::pointRayDist(center, _ray) > radius + EPS) {
		iret->hit = false;
		iret->t = 0;
		return;
	}

	// w = (P - C) - ((P - C)•v)v
	// Q = C + w
	// x = sqrt(R2 - |w|²)
	// A = Q - xv
	// t = (A - P)•v / v•v
	double closest = GeometryUtils::pointRayClosest(center, _ray);
	Pt3 CircleQ = _ray.at(closest);
	Vec3 w = CircleQ - center;
	double x = sqrt(radius * radius - (w * w));
	Vec3 A = CircleQ - (x * _ray.dir);
	double t = ((A - _ray.p) * _ray.dir) / (_ray.dir * _ray.dir);

	iret->hit = true;
	iret->t = t;
	iret->normal = A - center;
	iret->normal.normalize();
}


// The updateTransform functions should properly set up the transformation of this geometry.
// The transformation takes a unit shape into the shape described by the parameters.
// This function also computes the inverse of the transformation.
void Box::updateTransform() {
	Vec3 ncenter = _corner;
	ncenter += _length/2 * _lengthv;
	ncenter += _width/2 * _widthv;
	ncenter += _height/2 * _heightv;
	_center = ncenter;

	_mat[0][0] = _lengthv[0]*_length;
	_mat[0][1] = _lengthv[1]*_length;
	_mat[0][2] = _lengthv[2]*_length;
	_mat[0][3] = 0;
	_mat[1][0] = _widthv[0]*_width;
	_mat[1][1] = _widthv[1]*_width;
	_mat[1][2] = _widthv[2]*_width;
	_mat[1][3] = 0;
	_mat[2][0] = _heightv[0]*_height;
	_mat[2][1] = _heightv[1]*_height;
	_mat[2][2] = _heightv[2]*_height;
	_mat[2][3] = 0;
	_mat[3][0] = _corner[0];
	_mat[3][1] = _corner[1];
	_mat[3][2] = _corner[2];
	_mat[3][3] = 1;

	// NOTE: These two lines are required at the end
	_imat = !_mat;
	Geometry::updateTransform();
}

// A box has six faces, which are basically six planes with rectangular boundaries.
void Intersector::visit(Box* op, void* ret) {

	// Initialization
	IsectData* iret = (IsectData*) ret;
	iret->hit = false; // no collision
	iret->t = DINF;

	// Convert (original ray, current box) to (converted ray, canonical box)
	Mat4 invMat = op->getInverseMat();
	Pt3 newPoint = _ray.p * invMat; newPoint /= newPoint[3];
	// NOTE: Do not normalize this vector, or hit time will be wrong
	Vec3 newDir = _ray.dir * invMat; newDir[3] = 0;
	Ray ray(newPoint, newDir);

	// Canonical box: unit cube, axis-aligned, all coordinates within [0,1]
	const Plane planes[6] = {
		Plane(Pt3(.5, .5, 0), Vec3(0, 0, -1, 0)), // Bottom (-z)
		Plane(Pt3(.5, .5, 1), Vec3(0, 0, 1, 0)), // Top (+z)
		Plane(Pt3(.5, 0, .5), Vec3(0, -1, 0, 0)), // Left (-y)
		Plane(Pt3(.5, 1, .5), Vec3(0, 1, 0, 0)), // Right (+y)
		Plane(Pt3(0, .5, .5), Vec3(-1, 0, 0, 0)), // Back (-x)
		Plane(Pt3(1, .5, .5), Vec3(1, 0, 0, 0)) // Front (+x)
	};
	// Non-constant axes: {x,y} for z, {x,z} for y, and {y,z} for x
	const int Axis0[6] = { 0, 0, 0, 0, 1, 1 };
	const int Axis1[6] = { 1, 1, 2, 2, 2, 2 };

	// Ray-intersection with the canonical box
	for(int i = 0; i < 6; i++) {
		const Plane &pl = planes[i];
		double hitTime = GeometryUtils::planeRay(pl, ray);
		Pt3 hitPoint = ray.at(hitTime);
		// positive hit time, closer to the "eye point", and hit point within the unit square
		if(hitTime > EPS && iret->t > hitTime &&
			hitPoint[Axis0[i]] >= 0 && hitPoint[Axis0[i]] <= 1 &&
			hitPoint[Axis1[i]] >= 0 && hitPoint[Axis1[i]] <= 1)
		{
			iret->hit = true;
			iret->t = hitTime;
			iret->normal = pl.n; // Canonical-box normal
		}
	}

	if(iret->hit) {
		// TODO: Compute the face normal for the hit plane (canonical box -> current box)
		iret->normal = iret->normal * op->getForwardMat();
		iret->normal.normalize();
	}
}

void Ellipsoid::updateTransform() {
	_mat[0][0] = _lengthv[0] * _length;
	_mat[0][1] = _lengthv[1] * _length;
	_mat[0][2] = _lengthv[2] * _length;
	_mat[0][3] = 0;
	_mat[1][0] = _widthv[0] * _width;
	_mat[1][1] = _widthv[1] * _width;
	_mat[1][2] = _widthv[2] * _width;
	_mat[1][3] = 0;
	_mat[2][0] = _heightv[0] * _height;
	_mat[2][1] = _heightv[1] * _height;
	_mat[2][2] = _heightv[2] * _height;
	_mat[2][3] = 0;
	_mat[3][0] = _center[0];
	_mat[3][1] = _center[1];
	_mat[3][2] = _center[2];
	_mat[3][3] = 1;

	_imat = !_mat;
	Geometry::updateTransform();
}

void Intersector::visit(Ellipsoid* ellipsoid, void* ret) {
	// Initialization
	IsectData* iret = (IsectData*) ret;
	iret->hit = false; // no collision
	iret->t = DINF;

	Mat4 invMat = ellipsoid->getInverseMat();
	Pt3 newPoint = _ray.p * invMat; newPoint /= newPoint[3];
	Vec3 newDir = _ray.dir * invMat; newDir[3] = 0;
	// newDir.normalize();
	Ray ray(newPoint, newDir);

	// Canonical sphere
	Pt3 center = Pt3(0, 0, 0);
	double radius = 1;

	Vec3 v = ray.dir;
	v.normalize();

	const Vec3 C2P = ray.p - center;
	const double P2Q = C2P * v;
	const double P2Q2 = (P2Q * P2Q) / (v * v);
	const double D2 = (C2P * C2P) - P2Q2;
	const double R2 = 1;

	// If Dist² > R², fast reject
	if (D2 > R2 + EPS) {
		iret->hit = false;
		iret->t = 0;
	} else {
		Vec3 w = C2P - (P2Q * v);
		Vec3 Q = center + w;
		double x = sqrt(R2 - (w * w));
		Vec3 A = Q - (x * v);
		double t = ((A - ray.p) * ray.dir) / (ray.dir * ray.dir);

		iret->hit = true;
		iret->t = t;
		iret->normal = (A - center) * ellipsoid->getForwardMat();
		iret->normal.normalize();
	}
}

void Cylinder::updateTransform() {
	double radiusX = _length / 2;
	double radiusY = _width / 2;
	double height = _height;
	_mat[0][0] = _lengthv[0] * radiusX;
	_mat[0][1] = _lengthv[1] * radiusX;
	_mat[0][2] = _lengthv[2] * radiusX;
	_mat[0][3] = 0;
	_mat[1][0] = _widthv[0] * radiusY;
	_mat[1][1] = _widthv[1] * radiusY;
	_mat[1][2] = _widthv[2] * radiusY;
	_mat[1][3] = 0;
	_mat[2][0] = _heightv[0] * height;
	_mat[2][1] = _heightv[1] * height;
	_mat[2][2] = _heightv[2] * height;
	_mat[2][3] = 0;
	_mat[3][0] = _center[0];
	_mat[3][1] = _center[1];
	_mat[3][2] = _center[2];
	_mat[3][3] = 1;

	_imat = !_mat;
	Geometry::updateTransform();
}

void Intersector::visit(Cylinder* cylinder, void* ret) {
	IsectData* iret = (IsectData*)ret;
	iret->hit = false;
	iret->t = DINF;

	Mat4 invMat = cylinder->getInverseMat();
	Pt3 newPoint = _ray.p * invMat; newPoint /= newPoint[3];
	Vec3 newDir = _ray.dir * invMat; newDir[3] = 0;

	/* L * M^-1 */
	Ray ray(newPoint, newDir);

	/* Unit cylinder */
	const Vec3 A = Vec3(0, 0, 1, 0);
	const double H = 1.0;
	const Pt3 Q = Pt3(0, 0, 0, 1);
	const Pt3 QHA = Q + H * A;
	const double radius = 1.0;
	const double rad2 = radius * radius;
	Pt3 P = ray.p;
	Vec3 v = ray.dir;

	/*
	 * First check if line L is parallel to disk of cylinder.
	 */
	if (abs(A * v) < 0.01)
	{
		/*
		 * Check distance from line to the plane defined by
		 * Q + (H/2)A. No intersection if greater than H/2
		 */
		Pt3 halfCenter = Q + (H / 2) * A;
		if (abs((P - halfCenter) * A) > (H / 2))
		{
			iret->hit = false;
			iret->t = 0;
			return;
		}
	}

	/*
	 * Intersect line L with planes of the two disks
	 */
	const Plane botPlane = Plane(Q, -A);
	const Plane topPlane = Plane(QHA, A);
	double botHitTime = GeometryUtils::planeRay(botPlane, ray);
	double topHitTime = GeometryUtils::planeRay(topPlane, ray);

	Pt3 E1 = ray.at(botHitTime);
	Pt3 E2 = ray.at(topHitTime);

	const double botdist2 = (Q - E1) * (Q - E1);
	const double topdist2 = (QHA - E2) * (QHA - E2);

	bool insideBotDisk = botdist2 < (rad2 + EPS);
	bool insideTopDisk = topdist2 < (rad2 + EPS);

	/*
	 * When ray crosses both top and bot plane,
	 * then it definitely crosses with disk.
	 */
	if (insideBotDisk && insideTopDisk) {
		iret->hit = true;
		if (botHitTime < topHitTime) {
			iret->t = botHitTime;
			iret->normal = botPlane.n;
		} else {
			iret->t = topHitTime;
			iret->normal = topPlane.n;
		}

		iret->normal = iret->normal * cylinder->getForwardMat();
		iret->normal.normalize();
		return;
	}

	/*
	 * At most one or no intersection with disk.
	 * Intersect E(t) with infinite cylinder
	 */
	Ray ERay(E1, E2 - E1);

	/*
	 * Distance between axis and line is greater than radius
	 */
	if (GeometryUtils::rayRayDist(Ray(Q, A), ERay) > radius + EPS) {
		iret->hit = false;
		iret->t = 0;
		return;
	}

	/* v⊥ and P⊥ */
	Pt3 PPP = ERay.p - ((ERay.p - Q) * A) * A;
	Vec3 VPP = ERay.dir - (ERay.dir * A) * A;
	Vec3 VPPUnit = VPP;
	VPPUnit.normalize();

	// L⊥(t) must be normalized before line-circle intersection
	Ray PPRay(PPP, VPP);

	/*
	 * Apply line-circle intersection to find
	 * intersection points D1 and D2.
	 */
	double closest = GeometryUtils::pointRayClosest(Q, PPRay);
	Pt3 CircleQ = PPRay.at(closest);
	Vec3 w = CircleQ - Q;
	double x = sqrt(rad2 - (w * w));
	Pt3 D1 = CircleQ - (x * VPPUnit);
	Pt3 D2 = CircleQ + (x * VPPUnit);

	/*
	 * Intersection is within cylinder only if
	 * t1/t2 lies within 0 and 1.
	 * This t1/t2 represents parameters for both
	 * Projection Ray and E Ray.
	 */
	double t1 = ((D1 - PPRay.p) * PPRay.dir) / (PPRay.dir * PPRay.dir);
	double t2 = ((D2 - PPRay.p) * PPRay.dir) / (PPRay.dir * PPRay.dir);
	bool d1WithinRange = t1 >= 0.0 && t1 <= 1.0;
	bool d2WithinRange = t2 >= 0.0 && t2 <= 1.0;

	/*
	 * Substitute t1 and t2 to ERay to find
	 * the actual intersection point R1 and R2.
	 * R1 and R2 are on the same coordinate as L * M^-1.
	 */
	Pt3 R1 = ERay.at(t1);
	Pt3 R2 = ERay.at(t2);

	double r1HitTime = ((R1 - ray.p) * ray.dir) / (ray.dir * ray.dir);
	double r2HitTime = ((R2 - ray.p) * ray.dir) / (ray.dir * ray.dir);

	if (insideBotDisk && botHitTime < iret->t) {
		iret->hit = true;
		iret->t = botHitTime;
		iret->normal = botPlane.n;
	}

	if (insideTopDisk && topHitTime < iret->t) {
		iret->hit = true;
		iret->t = topHitTime;
		iret->normal = topPlane.n;
	}

	/*
	 * Determine best time between two intersect points
	 */
	Pt3 bestPoint;
	if (d1WithinRange && r1HitTime < iret->t) {
		iret->hit = true;
		iret->t = r1HitTime;
		bestPoint = ray.at(r1HitTime);
		iret->normal = (bestPoint - Q) - ((bestPoint - Q) * A) * A;
	}

	if (d2WithinRange && r2HitTime < iret->t) {
		iret->hit = true;
		iret->t = r2HitTime;
		bestPoint = ray.at(r2HitTime);
		iret->normal = (bestPoint - Q) - ((bestPoint - Q) * A) * A;
	}

	if (iret->hit) {
		iret->normal = iret->normal * cylinder->getForwardMat();
		iret->normal.normalize();
	}
}

void Cone::updateTransform() {
	double radiusX = _length / 2;
	double radiusY = _width / 2;
	double height = _height;
	_mat[0][0] = _lengthv[0] * radiusX;
	_mat[0][1] = _lengthv[1] * radiusX;
	_mat[0][2] = _lengthv[2] * radiusX;
	_mat[0][3] = 0;
	_mat[1][0] = _widthv[0] * radiusY;
	_mat[1][1] = _widthv[1] * radiusY;
	_mat[1][2] = _widthv[2] * radiusY;
	_mat[1][3] = 0;
	_mat[2][0] = _heightv[0] * height;
	_mat[2][1] = _heightv[1] * height;
	_mat[2][2] = _heightv[2] * height;
	_mat[2][3] = 0;
	_mat[3][0] = _center[0];
	_mat[3][1] = _center[1];
	_mat[3][2] = _center[2];
	_mat[3][3] = 1;

	_imat = !_mat;
	Geometry::updateTransform();
}

void Intersector::visit(Cone* cone, void* ret) {
	IsectData* iret = (IsectData*) ret;
	iret->hit = false;
	iret->t = DINF;

	Mat4 invMat = cone->getInverseMat();
	Pt3 newPoint = _ray.p * invMat;
	Vec3 newDir = _ray.dir * invMat; newDir[3] = 0;
	double lenofnewDir = mag(newDir);
	newDir.normalize();

	Ray ray(newPoint, newDir);

	/*
	 * Unit Cone
	 * V = Cone vertex
	 * A = Unit direction vector parallel to the cone axis6 yy
	 * Q = Center of the projected circle
	 * P = Point on the ray
	 * a = cpne angle. Since using 1/1 cone, it is 4 / PI
	 * u = unit vector of ray
	 */
	const Pt3 V = Pt3(0, 0, 1);
	const Vec3 A = Vec3(0, 0, -1);
	const Pt3 Q = Pt3(0, 0, 0);
	// const Pt3 P = ray.p;
	// const double a = M_PI / 4;
	// const Vec3 u = ray.dir;
	const double radius = 1.0;
	const double rad2 = radius * radius;

	/*
	 * Intersect line L with planes of the base
	 */
	const Plane botPlane = Plane(Q, A);
	double botHitTime = GeometryUtils::planeRay(botPlane, ray);

	Pt3 E1 = ray.at(botHitTime);

	const double botdist2 = (Q - E1) * (Q - E1);
	bool insideBotDisk = botdist2 < (rad2 + EPS);

	/*
	 * If you stare directly from the top/bottom of the cone.
	 * This rarely happens and only affects few pixels, so
     * no need for extensive check.
	 */

	// intersections on the base circle
	if (botHitTime > EPS && insideBotDisk) {
		iret->hit = true;
		iret->t = botHitTime;
		iret->normal = botPlane.n;
	}

	/*
   * Project Ray into the plane determined by the point Q and vector A
	 * u is the normal vector on the segment u = segRay.dir
   * L*(t) = (X0 + tX1) / ( m0 + tm1)
   * X0 = -P + ((Q - P) * A)V
   * m0 = (Q - P)*A - 1
   * X1 = -(u + (u * A)	V)
   * m1 = -u * A
   */
	Pt3 P = ray.p;
	Vec3 u = ray.dir;
	Pt3 X0 = -P + ((Q - P) * A) * V;
	Pt3 X1 = -(u + (u * A) * V);
	double m0 = (Q - P) * A - 1;
	double m1 = -u * A;

	if (m0 == 0  && m1 == 0) {
		iret->hit = false;
		iret->t = 0;
		return;
	}

	Pt3 projctedPoint;
	Vec3 projectedDir;
	if ((m0 > 0 || m0 < 0) && (m1 > 0 || m1 < 0)) {
		projctedPoint = X0 / m0;
		projectedDir = X0 / m0 - X1 / m1;
	} else if ((m0 > 0 || m0 < 0) && abs(m1) < EPS) {
		projctedPoint = X0 / m0;
		projectedDir = X1;
	} else if (abs(m0) < EPS && (m1 > 0 || m1 < 0)) {
		projctedPoint = X1 / m1;
		projectedDir = X0;
	}

	projctedPoint[3] = 1;
	projectedDir[3] = 0;

	/* L*(t) */
	Vec3 VUnit = projectedDir;
	VUnit.normalize();
	Ray PRay(projctedPoint, VUnit);

	/*
	 * Apply line-circle intersection to find
	 * intersection points D1 and D2
	 */
	double closest = GeometryUtils::pointRayClosest(Q, PRay);
	Pt3 CircleQ = PRay.at(closest);
	Vec3 w = CircleQ - Q;
	double hitTime;

	if (w * w < 1.0) {
		double x = sqrt(rad2 - (w * w));
		Pt3 D1 = CircleQ - (x * VUnit);
		Pt3 D2 = CircleQ + (x * VUnit);

		double t1 = (m0 * D1 - X0) * (X1 - m1 * D1) / ((X1 - m1 * D1) * (X1 - m1 * D1));
		double t2 = (m0 * D2 - X0) * (X1 - m1 * D2) / ((X1 - m1 * D2) * (X1 - m1 * D2));

		hitTime = t1 < t2 ? t1 : t2;
		Pt3 intersection = ray.at(hitTime);
		double intersectionZ = intersection[2];

		if (hitTime > EPS && hitTime < iret->t && intersectionZ < 1 + EPS && intersectionZ > EPS) {
			iret->hit = true;
			iret->t = hitTime;
			iret->normal = (intersection - V) - (mag(intersection - V) / sqrt(1.0 / 2.0)) * A;
		}
	}

  if (iret->hit) {
		iret->normal = iret->normal * transpose(invMat);
		iret->normal[3] = 0;
		iret->normal.normalize();
		iret->t = iret->t / lenofnewDir;
  }
}
