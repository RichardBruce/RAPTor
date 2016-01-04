#pragma once

/* Standard headers */
#include <list>
#include <vector>

/* Common headers */
#include "common.h"

/* Forward declarations */
class point_t;

namespace raptor_physics
{
/* The acceleration due to gravity */
/* Technically this varies, even at sea level, so 9.8 m/s/s should do */
const float ACCELERATION_UNDER_GRAVITY = 9.8f;

/* Welding distance */
/* Objects within this distance of eachother will be considered in contact */
const float WELD_DISTANCE = 1.0e-3f;

/* Epsilon */
/* Numbers less different to this are considered the same */
const float EPSILON = 1.0e-5f;

/* Too small force */
/* Forces smaller than this are considered negligable and ignored */
const float TOO_SMALL_F = 1.0e-6f;

/* Too small velocity */
/* Velocities smaller than this are considered negligable and ignored */
const float TOO_SMALL_V = 1.0e-6f;

/* Contact resolution residual */
/* The percent amount of change that can occur during contact resolution before it is considered done */
const float CONTACT_RESOLUTION_RESIDUAL = 0.0005f;

/* Shared funcitons */
/* Find the component of v that projects on to n */
point_t project_vector(const point_t &v, const point_t &n);

/* Find the most extreme vertex in direction d */
int find_support_vertex(const std::vector<point_t> &m, const point_t &d);

/* Find the most extreme vertex in direction d and its projection */
int find_support_vertex(const std::vector<point_t> &m, const point_t &d, float *const val);

int find_support_vertex(const std::vector<point_t> &m, const point_t &w, const point_t &c, const point_t &p, 
    const point_t &n, float *const val);

/* Find the time that a translating point passes through a plane */
float find_exact_none_translating_collision_time(const point_t &pa, const point_t &pb, const point_t &nb, const point_t &x0, const point_t &q0, 
    const point_t &q1, const float r0, const float r1);

/* Find the time that a translating and rotating point passes through a plane */
float find_exact_collision_time(const point_t &pa, const point_t &pb, const point_t &nb, const point_t &x0, const point_t &x1,
    const point_t &q0, const point_t &q1, const float r0, const float r1);

/* Find the time that a rotating edge passes through an edge */
float find_exact_none_translating_collision_time(const point_t &pa, const point_t &pb, const point_t &ea, const point_t &eb, const point_t &x0, 
    const point_t &q0, const point_t &q1, const float r0, const float r1);

/* Find the time that a translating and rotating edge passes through an edge */
float find_exact_collision_time(const point_t &pa, const point_t &pb, const point_t &ea, const point_t &eb, const point_t &x0, 
    const point_t &x1, const point_t &q0, const point_t &q1, const float r0, const float r1);
}; /* namespace raptor_physics */
