#include "Rendering/Raytracer.h"
#include "Rendering/Shading.h"
#include <FL/glu.h>
#include "Common/Common.h"

Raytracer::Raytracer() {
	_pixels = NULL;
}

void Raytracer::drawInit(GLdouble modelview[16], GLdouble proj[16], GLint view[4]) {
	_width = view[2];
	_height = view[3];

	if(_pixels) delete [] _pixels;
	_pixels = new float[_width*_height*4];

	memcpy(_modelview, modelview, 16*sizeof(modelview[0]));
	memcpy(_proj, proj, 16*sizeof(proj[0]));
	memcpy(_view, view, 4*sizeof(view[0]));

	Mat4 mv, pr;
	for(int j = 0; j < 16; j++) {
		mv[j/4][j%4] = modelview[j];
		pr[j/4][j%4] = proj[j];
	}

	_final = mv*pr;
	_invFinal = !_final;
	_last = 0;
}

Pt3 Raytracer::unproject(const Pt3& p) {
	Pt3	np = p;
	np[0] = (p[0]-_view[0])/_view[2];
	np[1] = (p[1]-_view[1])/_view[3];

	np[0] = np[0]*2-1;
	np[1] = np[1]*2-1;
	np[2] = np[2]*2-1;

	Pt3 ret = np*_invFinal;
	ret[0] /= ret[3];
	ret[1] /= ret[3];
	ret[2] /= ret[3];
	ret[3] = 1;
	return move(ret);
}

bool Raytracer::draw(int step) {
	int size = _width*_height;

	int j;
	for(j = _last; j < size && j < _last+step; j++) {
		int x = j % _width;
		int y = j / _width;

		Pt3 rst = unproject(Pt3(x, y, 0));
		Pt3 red = unproject(Pt3(x, y, 1));

		Ray r(rst, red-rst);
		r.dir.normalize();

		TraceResult res;
		if(_scene)
			res = trace(r, 0);

		res.color[3] = 1;
		int offset = (x + y*_width) * 4;
		for(int i = 0; i < 4; i++)
			_pixels[offset + i] = res.color[i];
	}

	_last = j;
	return (_last >= size);
}

TraceResult Raytracer::trace(const Ray& ray, int depth, double c) {
	TraceResult res;
	IsectData data;
	Geometry* geom;
	Material* mat;

	if (depth > 5) {
		res.color = Color(0, 0, 0);
		return res;
	}

	const Color ambientI = _scene->getLight(0)->getAmbient();
	double bestTime = DINF;
	Material* bestMat = NULL;
	Vec3 bestNormal;
	Pt3 bestPoint;

	/* Find best intersection for this ray */
	_intersector.setRay(ray);
	for (int j = 0; j < _scene->getNumObjects(); j++) {
		data.hit = false;
		geom = _scene->getObject(j);
		mat = _scene->getMaterial(geom);
		geom->accept(&_intersector, &data);

		if (data.hit && data.t > EPS) {
			if (bestTime > data.t) {
				bestTime = data.t;
				bestNormal = data.normal;
				bestMat = mat;
			}
		}
	}

	Pt3 hitPoint;
	if (bestTime < DINF) { // hit
		const Color ambientK = bestMat->getAmbient();
		const Color diffuseK = bestMat->getDiffuse();
		const Color specularK = bestMat->getSpecular();
		const double reflectivity = bestMat->getReflective();
		const double transparency = bestMat->getTransparency();
		const double refractIndex = c == 1 ? bestMat->getRefractIndex() : 1;
		const double specExponent = bestMat->getSpecExponent();

		// P2V is a unit vector from hitPoint to viewer
		hitPoint = ray.at(bestTime);
		Vec3 P2V = ray.p - hitPoint;
		P2V.normalize();

		// Ambient Intensity = kaIa
		res.color[0] = ambientI[0] * ambientK[0];
		res.color[1] = ambientI[1] * ambientK[1];
		res.color[2] = ambientI[2] * ambientK[2];

		for (int i = 0; i < _scene->getNumLights(); i++) {
			Light* light = _scene->getLight(i);
			Color colorLight = light->getColor();

			// P2L is a unit vector from light to hitPoint
			Vec3 P2L = light->getPos() - hitPoint;
			double dlight = sqrt(P2L * P2L);
			P2L.normalize();

			Color diffuseI = Color(0, 0, 0);
			Color specularI = Color(0, 0, 0);
			double shadow = 1.0;

			/* If (L•N) is 0 or negative, the light has not effect on diffuse or specular */
			double LXN = P2L * bestNormal;
			if (LXN > 0) {
				// Diffuse Reflection = Kd (L•N) Ip
				// L = Point of Intersection to Light source
				// pointLight refers to the color of the light
				diffuseI = Color(
					diffuseK[0] * LXN * colorLight[0],
					diffuseK[1] * LXN * colorLight[1],
					diffuseK[2] * LXN * colorLight[2]
				);

				// Specular Reflection = Ks (R•V)^n Ip
				// V = Point on the surface to the viewer
				Vec3 R = 2 * LXN * bestNormal - P2L;
				double RXVN = pow(R * P2V, specExponent);
				specularI = Color(
					specularK[0] * RXVN * colorLight[0],
					specularK[1] * RXVN * colorLight[1],
					specularK[2] * RXVN * colorLight[2]
				);

				Pt3 hitPoint1 = hitPoint + P2L * EPS;
				Ray surfaceRay = Ray(hitPoint1, P2L);
				_intersector.setRay(surfaceRay);

				for (int j = 0; j < _scene->getNumObjects(); j++) {
					data.hit = false;
					geom = _scene->getObject(j);
					geom->accept(&_intersector, &data);

					if (data.hit && data.t > 0.0001) {
						Pt3 newHitPoint = surfaceRay.at(data.t);
						if (abs(data.t) < dlight) {
							mat = _scene->getMaterial(geom);
							shadow = mat->getTransparency() * shadow;
						}
					}
				}

				res.color[0] += shadow * (diffuseI[0] + specularI[0]);
				res.color[1] += shadow * (diffuseI[1] + specularI[1]);
				res.color[2] += shadow * (diffuseI[2] + specularI[2]);
			}
		}

		Color reflect = Color(0, 0, 0);
		Color refract = Color(0, 0, 0);

		// Reflection Itensity = ksIreflected
		// Reflected Vector W = 2(V•N)N - V
		if (reflectivity > 0) {
			Vec3 W = 2 * (P2V * bestNormal) * bestNormal - P2V;
			W.normalize();
			reflect = trace(Ray(hitPoint, W), depth + 1).color;
		}

		if (transparency > 0) {
			// Total Reflection Check
			// (N•V)^2 + (c1 / c2)^2 < 1
			// c1 = index of refraction from outside
			// c2 = index of refraction of material
			double c1 = c;
			double c2 = refractIndex;
			double NXV = bestNormal * P2V;
			double NXV2 = NXV * NXV;
			double totalReflection = NXV2 + (c1 / c2) * (c1 / c2);
			if (totalReflection >= 1) {
				/*
				 * If NXV < 0, it's crossing face from inside to outside.
				 * If you don't flip normal here, this ray is trapped inside.
				 */
				Vec3 refractNormal = bestNormal;
				if (NXV < 0) refractNormal = -bestNormal;

				/*
				 * cos(Θ₂) = sqrt(1 - (c2 / c1)^2 (1 - (N•V)^2))
				 * W = ((c2/c1)(N•V) - cos(Θ₂))N - (c2/c1)V
				 */
				double refractRatio = c2 / c1;
				double cosin = sqrt(1 - (refractRatio * refractRatio) * (1 - NXV2));
				Vec3 W = (refractRatio * (refractNormal * P2V) - cosin) * refractNormal - refractRatio * P2V;
				W.normalize();

				if (c == 1) // If currently in air traveling into material
					refract = trace(Ray(hitPoint, W), depth + 1, bestMat->getRefractIndex()).color;
				else // If currently in material and coming out into air
					refract = trace(Ray(hitPoint, W), depth + 1, 1.0).color;
			}
		}

		res.color[0] += reflect[0] * reflectivity + refract[0] * transparency;
		res.color[1] += reflect[1] * reflectivity + refract[1] * transparency;
		res.color[2] += reflect[2] * reflectivity + refract[2] * transparency;
	}
	else { // miss
	 	// Default background color if missing all objects (default)
		res.color = Color(0, 0, 0);
	}

	return res;
}
