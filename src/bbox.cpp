#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
  // TODO (PathTracer):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	double txmin = (min.x - r.o.x) / r.d.x;
	double txmax = (max.x - r.o.x) / r.d.x;
	double tymin = (min.y - r.o.y) / r.d.y;
	double tymax = (max.y - r.o.y) / r.d.y;
	double tzmin = (min.z - r.o.z) / r.d.z;
	double tzmax = (max.z - r.o.z) / r.d.z;

	if (txmin > txmax) {
		std::swap(txmin, txmax);
	}
	if (tymin > tymax) {
		std::swap(tymin, tymax);
	}
	if (tzmin > tzmax) {
		std::swap(tzmin, tzmax);
	}

	double tmin = txmin;
	double tmax = txmax;
	if ((tmin > tymax) || (tymin > tmax)) {
		return false;
	}
	tmin = std::max(tmin, tymin);
	tmax = std::min(tmax, tymax);

	if ((tmin > tzmax) || (tzmin > tmax)) {
		return false;
	}
	t0 = std::max(tmin, tzmin);
	t1 = std::min(tmax, tzmax);

	return true;
}

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();
}

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
