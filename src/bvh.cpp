#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 {
namespace StaticScene {


BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  this->primitives = _primitives;

  // TODO (PathTracer):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bb;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb.expand(primitives[i]->get_bbox());
  }
  root = new BVHNode(bb, 0, primitives.size());
  build_BVH_recursive(root, max_leaf_size);
}


BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate
	deconstruct_BVH_recursive(root);
}
// helper function that deconstructs BVH recursively
void BVHAccel::deconstruct_BVH_recursive(BVHNode* node) {
	if (node->isLeaf()) {
		delete node;
	}
	else {
		deconstruct_BVH_recursive(node->l);
		deconstruct_BVH_recursive(node->r);
		delete node;
	}
}

// helper function that constructs BVH recursively
// the following implementation try all x/y/z axis and divide the best
void BVHAccel::build_BVH_recursive(BVHNode* node, const size_t max_leaf_size) {
	if (node->range <= max_leaf_size) {
		return;
	}
	Vector3D extent = node->bb.extent;
	Vector3D bbmin = node->bb.min;
	size_t bucket_total_num = 20;
	int flag;
	double step_legth;
	double best_SAH_all = DBL_MAX;
	int best_flag = -1;
	size_t best_count1 = 0;
	size_t best_count2 = 0;
	BBox best_box1, best_box2;

	vector<BBox> mybucket;
	vector<size_t> mybucket_count;
	
	for (flag = 0; flag < 3; flag++) { // try x/y/z axis
		if (flag == 0) {
			step_legth = extent.x / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().x < rhs->get_bbox().centroid().x; });
		}
		else if (flag == 1) {
			step_legth = extent.y / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().y < rhs->get_bbox().centroid().y; });
		}
		else {
			step_legth = extent.z / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().z < rhs->get_bbox().centroid().z; });
		}
		// create bucket
		mybucket.clear();
		mybucket_count.clear();
		for (int k = 0; k < bucket_total_num; k++) {
			BBox newbox;
			mybucket.push_back(newbox);
			mybucket_count.push_back(0);
		}
		for (size_t i = node->start; i < node->start + node->range; ++i) {
			size_t b;
			if (flag == 0) {
				b = floor((primitives[i]->get_bbox().centroid().x - bbmin.x) / (step_legth));
			}
			else if (flag == 1) {
				b = floor((primitives[i]->get_bbox().centroid().y - bbmin.y) / (step_legth));
			}
			else {
				b = floor((primitives[i]->get_bbox().centroid().z - bbmin.z) / (step_legth));
			}

			mybucket[b].expand(primitives[i]->get_bbox());
			mybucket_count[b]++;
		}
		double best_SAH = DBL_MAX;
		double tmp_SAH;
		size_t best_partition = 0;
		// find best partition
		for (size_t i = 1; i < bucket_total_num; ++i) {
			BBox box1, box2;
			size_t count1 = 0;
			size_t count2 = 0;
			for (size_t j = 0; j < i; ++j) {
				box1.expand(mybucket[j]);
				count1 += mybucket_count[j];
			}
			for (size_t j = i; j < bucket_total_num; ++j) {
				box2.expand(mybucket[j]);
				count2 += mybucket_count[j];
			}
			if (count1 == 0 || count2 == 0) {
				continue;
			}
			tmp_SAH = count1 * box1.surface_area() / node->bb.surface_area() + count2 * box2.surface_area() / node->bb.surface_area();
			if (tmp_SAH < best_SAH) {
				best_partition = i;
				best_SAH = tmp_SAH;
			}
		}
		// find possible best partition
		BBox box1, box2;
		size_t count1 = 0;
		size_t count2 = 0;
		if (best_partition != 0 && best_SAH < best_SAH_all) { // if all centroids fall into at least two buckets and it is the best so far along x/y/z
			for (size_t j = 0; j < best_partition; ++j) {
				box1.expand(mybucket[j]);
				count1 += mybucket_count[j];
			}
			for (size_t j = best_partition; j < bucket_total_num; ++j) {
				box2.expand(mybucket[j]);
				count2 += mybucket_count[j];
			}
			best_box1 = box1;
			best_box2 = box2;
			best_SAH_all = best_SAH;
			best_flag = flag;
			best_count1 = count1;
			best_count2 = count2;
		}
	}
	
	if (best_flag==-1) { // all fall into one bucket, then divide along z axis
		for (size_t j = node->start; j < node->start + size_t(node->range / 2); ++j) {
			best_box1.expand(primitives[j]->get_bbox());
			best_count1++;
		}
		for (size_t j = node->start + size_t(node->range / 2); j < node->start + node->range; ++j) {
			best_box2.expand(primitives[j]->get_bbox());
			best_count2++;
		}
	}
	else { // general case
		if (best_flag == 0) {
			step_legth = extent.x / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().x < rhs->get_bbox().centroid().x; });
		}
		else if (best_flag == 1) {
			step_legth = extent.y / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().y < rhs->get_bbox().centroid().y; });
		}
		else {
			step_legth = extent.z / bucket_total_num;
			sort(primitives.begin() + node->start, primitives.begin() + node->start + node->range,
				[](const Primitive* lhs, const Primitive* rhs) {  return lhs->get_bbox().centroid().z < rhs->get_bbox().centroid().z; });
		}
	}

	// construct children node recursively
	BVHNode* lc = new BVHNode(best_box1, node->start, best_count1);
	BVHNode* rc = new BVHNode(best_box2, node->start + best_count1, best_count2);

	node->l = lc;
	node->r = rc;
	build_BVH_recursive(lc, max_leaf_size);
	build_BVH_recursive(rc, max_leaf_size);
}


BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.
	double t0, t1;
	if (root->bb.intersect(ray, t0, t1)) {
		if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
			return find_intersection_recursive(ray, root);
		}
	}
	return false;
	//bool hit = false;
	//for (size_t p = 0; p < primitives.size(); ++p) {
	//	if (primitives[p]->intersect(ray)) hit = true;
	//}

	//return hit;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.
	double t0, t1;
	if (root->bb.intersect(ray, t0, t1)) {
		if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
			return find_intersection_recursive(ray, root, isect);
		}
	}
	return false;
	//bool hit = false;
	//for (size_t p = 0; p < primitives.size(); ++p) {
	//	if (primitives[p]->intersect(ray, isect)) hit = true;
	//}

	//return hit;
}

bool BVHAccel::find_intersection_recursive(const Ray &ray, BVHNode* node) const {
	if (node->isLeaf()) {
		for (size_t p = node->start; p < node->start+node->range; ++p) {
			if (primitives[p]->intersect(ray)) {
				return true;
			}
		}
	}
	else {
		double t0, t1, t2, t3, tbest1, tbest2;
		
		bool flag1, flag2;
		node->l->bb.intersect(ray, t0, t1);
		node->r->bb.intersect(ray, t2, t3);
		if (t0 >= ray.min_t&&t0 <= ray.max_t) {
			tbest1 = t0;
		}
		else {
			tbest1 = t1;
		}
		if (t2 >= ray.min_t&&t2 <= ray.max_t) {
			tbest2 = t2;
		}
		else {
			tbest2 = t3;
		}
		BVHNode *first = (tbest1 <= tbest2) ? node->l : node->r;
		BVHNode *second = (tbest1 <= tbest2) ? node->r : node->l;


		if (first->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
				if (find_intersection_recursive(ray, first)) {
					return true;
				}
			}			
		}
		if (second->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
				if (find_intersection_recursive(ray, second)) {
					return true;
				}
			}
		}	
	}
	return false;
}

bool BVHAccel::find_intersection_recursive(const Ray &ray, BVHNode* node, Intersection *isect) const {
	bool hit = false;
	if (node->isLeaf()) {
		for (size_t p = node->start; p < node->start + node->range; ++p) {
			if (primitives[p]->intersect(ray, isect)) {
				hit = true;
			}
		}
		return hit;
	}
	else {
		double t0, t1, t2, t3, tbest1, tbest2;

		bool flag1, flag2;
		node->l->bb.intersect(ray, t0, t1);
		node->r->bb.intersect(ray, t2, t3);
		if (t0 >= ray.min_t&&t0 <= ray.max_t) {
			tbest1 = t0;
		}
		else {
			tbest1 = t1;
		}
		if (t2 >= ray.min_t&&t2 <= ray.max_t) {
			tbest2 = t2;
		}
		else {
			tbest2 = t3;
		}
		BVHNode *first = (tbest1 <= tbest2) ? node->l : node->r;
		BVHNode *second = (tbest1 <= tbest2) ? node->r : node->l;

		if (first->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
				if (find_intersection_recursive(ray, first, isect)) {
					hit = true;
				}
			}
			
		}
		if (second->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
				if (find_intersection_recursive(ray, second, isect)) {
					hit = true;
				}
			}
		}
		return hit;
	}
}

}  // namespace StaticScene
}  // namespace CMU462
