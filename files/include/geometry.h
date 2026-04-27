#ifndef GEOMETRY_H
#define GEOMETRY_H
#include "types.h"

Vec3 bezier_eval(BezierCubic b, float t);
Vec3 bezier_tangent(BezierCubic b, float t);
AABB bezier_bounds(BezierCubic b, float radius);
void bezier_split(BezierCubic b, BezierCubic *left, BezierCubic *right);;
AABB bezier_bounds_adaptive(BezierCubic b, float radius, AABB *segment_bounds, int *segment_count);
#endif