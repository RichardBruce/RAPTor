/* Standard headers */
#include <algorithm>

/* Boost headers */
#include "boost/multiprecision/cpp_int.hpp"

/* Convex decomposition headers */
#include "dac_convex_hull.h"


namespace raptor_convex_decomposition
{
int dac_convex_hull::Rational64::compare(const Rational64& b) const
{
    using boost::multiprecision::int128_t;
    if (sign != b.sign)
    {
        return sign - b.sign;
    }
    else if (sign == 0)
    {
        return 0;
    }

    /* return (numerator * b.denominator > b.numerator * denominator) ? sign : (numerator * b.denominator < b.numerator * denominator) ? -sign : 0; */
    const int128_t lhs = static_cast<int128_t>(m_numerator) * static_cast<int128_t>(b.m_denominator);
    const int128_t rhs = static_cast<int128_t>(m_denominator) * static_cast<int128_t>(b.m_numerator);
    return (lhs > rhs) ? sign : ((lhs < rhs) ? -sign : 0);
}

dac_convex_hull::edge* dac_convex_hull::new_edge_pair(vertex *const from, vertex *const to)
{
    assert((from != nullptr) && (to != nullptr));
    edge *const e = _edge_pool.malloc();
    edge *const r = _edge_pool.malloc();
    e->reverse = r;
    r->reverse = e;
    e->copy = _merge_stamp;
    r->copy = _merge_stamp;
    e->target = to;
    r->target = from;

    return e;
}

void dac_convex_hull::compute_internal(intermediate_hull *const result, const int start, const int end)
{
    const int n = end - start;
    switch (n)
    {
        case 0 :
            return;
        case 2 :
        {
            vertex *v = &_original_vertices[start];
            vertex *w = &_original_vertices[start + 1];
            if (v->point != w->point)
            {
                const std::int32_t dx = v->point.x - w->point.x;
                const std::int32_t dy = v->point.y - w->point.y;
                if ((dx == 0) && (dy == 0))
                {
                    if (v->point.z > w->point.z)
                    {
                        std::swap(v, w);
                    }

                    assert(v->point.z < w->point.z);
                    v->next = v;
                    v->prev = v;
                    result->minXy = v;
                    result->maxXy = v;
                    result->minYx = v;
                    result->maxYx = v;
                }
                else
                {
                    v->next = w;
                    v->prev = w;
                    w->next = v;
                    w->prev = v;
                    if ((dx < 0) || ((dx == 0) && (dy < 0)))
                    {
                        result->minXy = v;
                        result->maxXy = w;
                    }
                    else
                    {
                        result->minXy = w;
                        result->maxXy = v;
                    }

                    if ((dy < 0) || ((dy == 0) && (dx < 0)))
                    {
                        result->minYx = v;
                        result->maxYx = w;
                    }
                    else
                    {
                        result->minYx = w;
                        result->maxYx = v;
                    }
                }

                edge *e = new_edge_pair(v, w);
                e->link(e);
                v->edges = e;

                e = e->reverse;
                e->link(e);
                w->edges = e;

                return;
            }
        }
        case 1 :
        {
            vertex *const v = &_original_vertices[start];
            v->edges = nullptr;
            v->next = v;
            v->prev = v;

            result->minXy = v;
            result->maxXy = v;
            result->minYx = v;
            result->maxYx = v;
            return;
        }
    }

    /* Median object split */
    const int split0 = start + (n >> 1);

    /* Drop co-incident points */
    int split1  = split0;
    while ((split1 < end) && (_original_vertices[split1].point == _original_vertices[split0 - 1].point))
    {
        ++split1;
    }

    /* Recurse left */
    compute_internal(result, start, split0);

    /* Recurse right */
    intermediate_hull hull1;
    compute_internal(&hull1, split1, end);

    /* Merge hulls */
    merge(result, &hull1);
}

dac_convex_hull::orientation_t dac_convex_hull::get_orientation(const edge &prev, const edge *const next, const point_ti<std::int64_t> &s, const point_ti<std::int64_t> &t)
{
    assert(prev.reverse->target == next->reverse->target);
    if (prev.next == next)
    {
        if (prev.prev == next)
        {
            const point_ti<std::int64_t> n(cross_product(t, s));
            const point_ti<std::int64_t> m(cross_product(*prev.target - *next->reverse->target, *next->target - *next->reverse->target));
            assert(m != 0);
            assert(dot_product(n, m) != 0);

            return (dot_product(n, m) > 0) ? orientation_t::counter_clockwise : orientation_t::clockwise;
        }

        return orientation_t::counter_clockwise;
    }
    else if (prev.prev == next)
    {
        return orientation_t::clockwise;
    }
    else
    {
        return orientation_t::none;
    }
}

dac_convex_hull::edge* dac_convex_hull::find_max_angle(Rational64 *const min_cot, const vertex &start, const point_ti<std::int64_t> &s, const point_ti<std::int64_t> &rxs, const point_ti<std::int64_t> &sxrxs, const bool ccw)
{
    edge* e = start.edges;
    if (e == nullptr)
    {
        return nullptr;
    }

    edge* min_edge = nullptr;
    do
    {
        if (e->copy > _merge_stamp)
        {
            const point_ti<std::int64_t> t(*e->target - start);
            const Rational64 cot(dot_product(t, sxrxs), dot_product(t, rxs));
            assert(!cot.isNaN() || (ccw ? (dot_product(t, s) < 0) : (dot_product(t, s) > 0)));

            if (!cot.isNaN())
            {
                int cmp;
                if (min_edge == nullptr)
                {
                    (*min_cot)  = cot;
                    min_edge    = e;
                }
                else if ((cmp = cot.compare(*min_cot)) < 0)
                {
                    (*min_cot)  = cot;
                    min_edge    = e;
                }
                else if ((cmp == 0) && (ccw == (get_orientation(*min_edge, e, s, t) == orientation_t::counter_clockwise)))
                {
                    min_edge    = e;
                }
            }
        }

        e = e->next;
    } while (e != start.edges);

    return min_edge;
}

void dac_convex_hull::find_edge_for_coplanar_faces(const vertex &c0, const vertex &c1, edge*& e0, edge*& e1, const vertex *const stop0, const vertex *const stop1)
{
    edge* start0 = e0;
    edge* start1 = e1;
    point_ti<std::int64_t> et0(start0 ? start0->target->point : c0.point);
    point_ti<std::int64_t> et1(start1 ? start1->target->point : c1.point);
    point_ti<std::int64_t> s(c1.point - c0.point);
    point_ti<std::int64_t> normal(cross_product((start0 ? start0 : start1)->target->point - c0.point, s));
    std::int64_t dist = dot_product(c0.point, normal);
    assert(!start1 || (dot_product(start1->target->point, normal) == dist));
    point_ti<std::int64_t> perp(cross_product(s, normal));
    assert(perp != 0);
    
    std::int64_t maxDot0 = dot_product(et0, perp);
    if (e0)
    {
        while (e0->target != stop0)
        {
            edge* e = e0->reverse->prev;
            if (dot_product(e->target->point, normal) < dist)
            {
                break;
            }
            assert(dot_product(e->target->point, normal) == dist);
            if (e->copy == _merge_stamp)
            {
                break;
            }
            std::int64_t dot = dot_product(e->target->point, perp);
            if (dot <= maxDot0)
            {
                break;
            }
            maxDot0 = dot;
            e0 = e;
            et0 = e->target->point;
        }
    }
    
    std::int64_t maxDot1 = dot_product(et1, perp);
    if (e1)
    {
        while (e1->target != stop1)
        {
            edge* e = e1->reverse->next;
            if (dot_product(e->target->point, normal) < dist)
            {
                break;
            }

            assert(dot_product(e->target->point, normal) == dist);
            if (e->copy == _merge_stamp)
            {
                break;
            }

            std::int64_t dot = dot_product(e->target->point, perp);
            if (dot <= maxDot1)
            {
                break;
            }

            maxDot1 = dot;
            e1 = e;
            et1 = e->target->point;
        }
    }

    std::int64_t dx = maxDot1 - maxDot0;
    if (dx > 0)
    {
        while (true)
        {
            std::int64_t dy = dot_product(et1 - et0, s);            
            if (e0 && (e0->target != stop0))
            {
                edge* f0 = e0->next->reverse;
                if (f0->copy > _merge_stamp)
                {
                    std::int64_t dx0 = dot_product(f0->target->point - et0, perp);
                    std::int64_t dy0 = dot_product(f0->target->point - et0, s);
                    if ((dx0 == 0) ? (dy0 < 0) : ((dx0 < 0) && (Rational64(dy0, dx0).compare(Rational64(dy, dx)) >= 0)))
                    {
                        et0 = f0->target->point;
                        dx = dot_product(et1 - et0, perp);
                        e0 = (e0 == start0) ? nullptr : f0;
                        continue;
                    }
                }
            }
            
            if (e1 && (e1->target != stop1))
            {
                edge* f1 = e1->reverse->next;
                if (f1->copy > _merge_stamp)
                {
                    point_ti<std::int64_t> d1 = f1->target->point - et1;
                    if (dot_product(d1, normal) == 0)
                    {
                        std::int64_t dx1 = dot_product(d1, perp);
                        std::int64_t dy1 = dot_product(d1, s);
                        std::int64_t dxn = dot_product(f1->target->point - et0, perp);
                        if ((dxn > 0) && ((dx1 == 0) ? (dy1 < 0) : ((dx1 < 0) && (Rational64(dy1, dx1).compare(Rational64(dy, dx)) > 0))))
                        {
                            e1 = f1;
                            et1 = e1->target->point;
                            dx = dxn;
                            continue;
                        }
                    }
                    else
                    {
                        assert((e1 == start1) && (dot_product(d1, normal) < 0));
                    }
                }
            }

            break;
        }
    }
    else if (dx < 0)
    {
        while (true)
        {
            std::int64_t dy = dot_product(et1 - et0, s);            
            if (e1 && (e1->target != stop1))
            {
                edge* f1 = e1->prev->reverse;
                if (f1->copy > _merge_stamp)
                {
                    std::int64_t dx1 = dot_product(f1->target->point - et1, perp);
                    std::int64_t dy1 = dot_product(f1->target->point - et1, s);
                    if ((dx1 == 0) ? (dy1 > 0) : ((dx1 < 0) && (Rational64(dy1, dx1).compare(Rational64(dy, dx)) <= 0)))
                    {
                        et1 = f1->target->point;
                        dx = dot_product(et1 - et0, perp);
                        e1 = (e1 == start1) ? nullptr : f1;
                        continue;
                    }
                }
            }
            
            if (e0 && (e0->target != stop0))
            {
                edge* f0 = e0->reverse->prev;
                if (f0->copy > _merge_stamp)
                {
                    point_ti<std::int64_t> d0 = f0->target->point - et0;
                    if (dot_product(d0, normal) == 0)
                    {
                        std::int64_t dx0 = dot_product(d0, perp);
                        std::int64_t dy0 = dot_product(d0, s);
                        std::int64_t dxn = dot_product(et1 - f0->target->point, perp);
                        if ((dxn < 0) && ((dx0 == 0) ? (dy0 > 0) : ((dx0 < 0) && (Rational64(dy0, dx0).compare(Rational64(dy, dx)) < 0))))
                        {
                            e0 = f0;
                            et0 = e0->target->point;
                            dx = dxn;
                            continue;
                        }
                    }
                    else
                    {
                        assert((e0 == start0) && (dot_product(d0, normal) < 0));
                    }
                }
            }

            break;
        }
    }
}

bool dac_convex_hull::merge_projection(intermediate_hull *const h0, intermediate_hull *const h1, vertex*& c0, vertex*& c1)
{
    vertex* v0 = h0->maxYx;
    vertex* v1 = h1->minYx;
    if ((v0->point.x == v1->point.x) && (v0->point.y == v1->point.y))
    {
        assert(v0->point.z < v1->point.z);
        vertex *const v1p = v1->prev;
        if (v1p == v1)
        {
            c0 = v0;
            if (v1->edges)
            {
                assert(v1->edges->next == v1->edges);
                v1 = v1->edges->target;
                assert(v1->edges->next == v1->edges);
            }

            c1 = v1;
            return false;
        }

        vertex* v1n = v1->next;
        v1p->next = v1n;
        v1n->prev = v1p;
        if (v1 == h1->minXy)
        {
            if ((v1n->point.x < v1p->point.x) || ((v1n->point.x == v1p->point.x) && (v1n->point.y < v1p->point.y)))
            {
                h1->minXy = v1n;
            }
            else
            {
                h1->minXy = v1p;
            }
        }

        if (v1 == h1->maxXy)
        {
            if ((v1n->point.x > v1p->point.x) || ((v1n->point.x == v1p->point.x) && (v1n->point.y > v1p->point.y)))
            {
                h1->maxXy = v1n;
            }
            else
            {
                h1->maxXy = v1p;
            }
        }
    }
    
    v0 = h0->maxXy;
    v1 = h1->maxXy;
    vertex* v00 = nullptr;
    vertex* v10 = nullptr;
    std::int32_t sign = 1;
    for (int side = 0; side <= 1; ++side)
    {       
        std::int32_t dx = (v1->point.x - v0->point.x) * sign;
        if (dx > 0)
        {
            while (true)
            {
                vertex *const w0 = side ? v0->next : v0->prev;
                const std::int32_t dy = v1->point.y - v0->point.y;
                if (w0 != v0)
                {
                    const std::int32_t dx0 = (w0->point.x - v0->point.x) * sign;
                    const std::int32_t dy0 = w0->point.y - v0->point.y;
                    if ((dy0 <= 0) && ((dx0 == 0) || ((dx0 < 0) && ((dy0 * dx) <= (dy * dx0)))))
                    {
                        v0 = w0;
                        dx = (v1->point.x - v0->point.x) * sign;
                        continue;
                    }
                }

                vertex *const w1 = side ? v1->next : v1->prev;
                if (w1 != v1)
                {
                    const std::int32_t dx1 = (w1->point.x - v1->point.x) * sign;
                    const std::int32_t dy1 = w1->point.y - v1->point.y;
                    const std::int32_t dxn = (w1->point.x - v0->point.x) * sign;
                    if ((dxn > 0) && (dy1 < 0) && ((dx1 == 0) || ((dx1 < 0) && ((dy1 * dx) < (dy * dx1)))))
                    {
                        v1 = w1;
                        dx = dxn;
                        continue;
                    }
                }

                break;
            }
        }
        else if (dx < 0)
        {
            while (true)
            {
                vertex *const w1 = side ? v1->prev : v1->next;
                const std::int32_t dy = v1->point.y - v0->point.y;                
                if (w1 != v1)
                {
                    const std::int32_t dx1 = (w1->point.x - v1->point.x) * sign;
                    const std::int32_t dy1 = w1->point.y - v1->point.y;
                    if ((dy1 >= 0) && ((dx1 == 0) || ((dx1 < 0) && ((dy1 * dx) <= (dy * dx1)))))
                    {
                        v1 = w1;
                        dx = (v1->point.x - v0->point.x) * sign;
                        continue;
                    }
                }
                
                vertex *const w0 = side ? v0->prev : v0->next;
                if (w0 != v0)
                {
                    const std::int32_t dx0 = (w0->point.x - v0->point.x) * sign;
                    const std::int32_t dy0 = w0->point.y - v0->point.y;
                    const std::int32_t dxn = (v1->point.x - w0->point.x) * sign;
                    if ((dxn < 0) && (dy0 > 0) && ((dx0 == 0) || ((dx0 < 0) && ((dy0 * dx) < (dy * dx0)))))
                    {
                        v0 = w0;
                        dx = dxn;
                        continue;
                    }
                }
                
                break;
            }
        }
        else
        {
            std::int32_t y0 = v0->point.y;
            vertex* w0 = v0;
            vertex* t;
            while (((t = side ? w0->next : w0->prev) != v0) && (t->point.x == v0->point.x) && (t->point.y <= y0))
            {
                w0 = t;
                y0 = t->point.y;
            }
            v0 = w0;

            std::int32_t y1 = v1->point.y;
            vertex* w1 = v1;
            while (((t = side ? w1->prev : w1->next) != v1) && (t->point.x == v0->point.x) && (t->point.y >= y1))
            {
                w1 = t;
                y1 = t->point.y;
            }
            v1 = w1;
        }
        
        if (side == 0)
        {
            v00 = v0;
            v10 = v1;

            v0 = h0->minXy;
            v1 = h1->minXy;
            sign = -1;
        }
    }

    v0->prev = v1;
    v1->next = v0;

    v00->next = v10;
    v10->prev = v00;

    if (h1->minXy->point.x < h0->minXy->point.x)
    {
        h0->minXy = h1->minXy;
    }

    if (h1->maxXy->point.x >= h0->maxXy->point.x)
    {
        h0->maxXy = h1->maxXy;
    }
    
    h0->maxYx = h1->maxYx;

    c0 = v00;
    c1 = v10;

    return true;
}

void dac_convex_hull::merge(intermediate_hull *const h0, intermediate_hull *const h1)
{
    if (!h1->maxXy)
    {
        return;
    }

    if (!h0->maxXy)
    {
        (*h0) = (*h1);
        return;
    }
    
    --_merge_stamp;
    point_ti<std::int64_t> prevPoint;
    vertex* c0 = nullptr;
    vertex* c1 = nullptr;
    if (merge_projection(h0, h1, c0, c1))
    {
        point_ti<std::int64_t> s = *c1 - *c0;
        point_ti<std::int64_t> normal(cross_product(point_ti<std::int64_t>(0, 0, -1), s));
        point_ti<std::int64_t> t(cross_product(s, normal));
        assert(t != 0);

        edge* e = c0->edges;
        edge* start0 = nullptr;
        if (e != nullptr)
        {
            do
            {
                const std::int64_t dot = dot_product(*e->target - *c0, normal);
                assert(dot <= 0);
                if ((dot == 0) && (dot_product(*e->target - *c0, t) > 0))
                {
                    if (!start0 || (get_orientation(*start0, e, s, point_ti<std::int64_t>(0, 0, -1)) == orientation_t::clockwise))
                    {
                        start0 = e;
                    }
                }
                e = e->next;
            } while (e != c0->edges);
        }
        
        e = c1->edges;
        edge* start1 = nullptr;
        if (e != nullptr)
        {
            do
            {
                const std::int64_t dot = dot_product(*e->target - *c1, normal);
                assert(dot <= 0);
                if ((dot == 0) && (dot_product(*e->target - *c1, t) > 0))
                {
                    if (!start1 || (get_orientation(*start1, e, s, point_ti<std::int64_t>(0, 0, -1)) == orientation_t::counter_clockwise))
                    {
                        start1 = e;
                    }
                }
                e = e->next;
            } while (e != c1->edges);
        }

        if (start0 || start1)
        {
            find_edge_for_coplanar_faces(*c0, *c1, start0, start1, nullptr, nullptr);
            if (start0)
            {
                c0 = start0->target;
            }
            if (start1)
            {
                c1 = start1->target;
            }
        }

        prevPoint = c1->point;
        ++prevPoint.z;
    }
    else
    {
        prevPoint = c1->point;
        ++prevPoint.x;
    }

    edge* toPrev0       = nullptr;
    edge* firstNew0     = nullptr;
    edge* pendingHead0  = nullptr;
    edge* pendingTail0  = nullptr;
    edge* toPrev1       = nullptr;
    edge* firstNew1     = nullptr;
    edge* pendingHead1  = nullptr;
    edge* pendingTail1  = nullptr;
    vertex* first0      = c0;
    vertex* first1      = c1;
    bool firstRun       = true;
    while (true)
    {
        const point_ti<std::int64_t> s(*c1 - *c0);
        const point_ti<std::int64_t> r(prevPoint - c0->point);
        const point_ti<std::int64_t> rxs(cross_product(r, s));
        const point_ti<std::int64_t> sxrxs(cross_product(s, rxs));
        
        Rational64 min_cot0(0, 0);
        edge* min0 = find_max_angle(&min_cot0, *c0, s, rxs, sxrxs, false);
        Rational64 min_cot1(0, 0);
        edge* min1 = find_max_angle(&min_cot1, *c1, s, rxs, sxrxs, true);
        if ((min0 == nullptr) && (min1 == nullptr))
        {
            edge* e = new_edge_pair(c0, c1);
            e->link(e);
            c0->edges = e;

            e = e->reverse;
            e->link(e);
            c1->edges = e;
            return;
        }
        else
        {
            const int cmp = (min0 == nullptr) ? 1 : (min1 == nullptr) ? -1 : min_cot0.compare(min_cot1);
            if (firstRun || ((cmp >= 0) ? !min_cot1.isNegativeInfinity() : !min_cot0.isNegativeInfinity()))
            {
                edge* e = new_edge_pair(c0, c1);
                if (pendingTail0)
                {
                    pendingTail0->prev = e;
                }
                else
                {
                    pendingHead0 = e;
                }
                e->next = pendingTail0;
                pendingTail0 = e;

                e = e->reverse;
                if (pendingTail1)
                {
                    pendingTail1->next = e;
                }
                else
                {
                    pendingHead1 = e;
                }
                e->prev = pendingTail1;
                pendingTail1 = e;
            }
            
            edge* e0 = min0;
            edge* e1 = min1;
            if (cmp == 0)
            {
                find_edge_for_coplanar_faces(*c0, *c1, e0, e1, nullptr, nullptr);
            }

            if ((cmp >= 0) && e1)
            {
                if (toPrev1)
                {
                    for (edge* e = toPrev1->next, *n = nullptr; e != min1; e = n)
                    {
                        n = e->next;
                        remove_edge_pair(e);
                    }
                }

                if (pendingTail1)
                {
                    if (toPrev1)
                    {
                        toPrev1->link(pendingHead1);
                    }
                    else
                    {
                        min1->prev->link(pendingHead1);
                        firstNew1 = pendingHead1;
                    }
                    pendingTail1->link(min1);
                    pendingHead1 = nullptr;
                    pendingTail1 = nullptr;
                }
                else if (!toPrev1)
                {
                    firstNew1 = min1;
                }

                prevPoint = c1->point;
                c1 = e1->target;
                toPrev1 = e1->reverse;
            }

            if ((cmp <= 0) && e0)
            {
                if (toPrev0)
                {
                    for (edge* e = toPrev0->prev, *n = nullptr; e != min0; e = n)
                    {
                        n = e->prev;
                        remove_edge_pair(e);
                    }
                }

                if (pendingTail0)
                {
                    if (toPrev0)
                    {
                        pendingHead0->link(toPrev0);
                    }
                    else
                    {
                        pendingHead0->link(min0->next);
                        firstNew0 = pendingHead0;
                    }
                    min0->link(pendingTail0);
                    pendingHead0 = nullptr;
                    pendingTail0 = nullptr;
                }
                else if (!toPrev0)
                {
                    firstNew0 = min0;
                }

                prevPoint = c0->point;
                c0 = e0->target;
                toPrev0 = e0->reverse;
            }
        }

        if ((c0 == first0) && (c1 == first1))
        {
            if (toPrev0 == nullptr)
            {
                pendingHead0->link(pendingTail0);
                c0->edges = pendingTail0;
            }
            else
            {
                for (edge* e = toPrev0->prev, *n = nullptr; e != firstNew0; e = n)
                {
                    n = e->prev;
                    remove_edge_pair(e);
                }
                if (pendingTail0)
                {
                    pendingHead0->link(toPrev0);
                    firstNew0->link(pendingTail0);
                }
            }

            if (toPrev1 == nullptr)
            {
                pendingTail1->link(pendingHead1);
                c1->edges = pendingTail1;
            }
            else
            {
                for (edge* e = toPrev1->next, *n = nullptr; e != firstNew1; e = n)
                {
                    n = e->next;
                    remove_edge_pair(e);
                }
                if (pendingTail1)
                {
                    toPrev1->link(pendingHead1);
                    pendingTail1->link(firstNew1);
                }
            }
            
            return;
        }

        firstRun = false;
    }
}

void dac_convex_hull::compute(const std::vector<point_t<>> &coords)
{
    point_t<> min_pt(coords[0]);
    point_t<> max_pt(coords[0]);
    for (int i = 1; i < static_cast<int>(coords.size()); ++i)
    {
        min_pt = min(min_pt, coords[i]);
        max_pt = max(max_pt, coords[i]);
    }

    point_t<> s(max_pt - min_pt);
    _max_axis = s.max_axis();
    _min_axis = s.min_axis();
    if (_min_axis == _max_axis)
    {
        _min_axis = (_max_axis + 1) % 3;
    }
    _med_axis = 3 - _max_axis - _min_axis;

    s /= 10216.0f;
    if (((_med_axis + 1) % 3) != _max_axis)
    {
        s *= -1;
    }
    _scaling = s;

    if (s[0] != 0.0f)
    {
        s[0] = 1.0f / s[0];
    }
    if (s[1] != 0.0f)
    {
        s[1] = 1.0f / s[1];
    }
    if (s[2] != 0.0f)
    {
        s[2] = 1.0f / s[2];
    }

    _center = (min_pt + max_pt) * 0.5f;

    std::vector<point_ti<std::int64_t>> points;
    points.resize(coords.size());
    for (int i = 0; i < static_cast<int>(coords.size()); ++i)
    {
        const point_t<> p((coords[i] - _center) * s);
        points[i].x = static_cast<std::int32_t>(p[_med_axis]);
        points[i].y = static_cast<std::int32_t>(p[_max_axis]);
        points[i].z = static_cast<std::int32_t>(p[_min_axis]);
    }
    std::sort(points.begin(), points.end(), [](const point_ti<std::int64_t> &p, const point_ti<std::int64_t> &q)
        {
            return (p.y < q.y) || ((p.y == q.y) && ((p.x < q.x) || ((p.x == q.x) && (p.z < q.z))));
        });

    _original_vertices.resize(coords.size());
    for (int i = 0; i < static_cast<int>(coords.size()); ++i)
    {
        _original_vertices[i].point = points[i];
    }

    _merge_stamp = -3;

    intermediate_hull hull;
    compute_internal(&hull, 0, coords.size());
    _vertex_list = hull.minXy;
}

point_t<> dac_convex_hull::get_coordinates(const vertex* v)
{
    point_t<> p;
    p[_med_axis] = v->xvalue();
    p[_max_axis] = v->yvalue();
    p[_min_axis] = v->zvalue();
    return p * _scaling + _center;
}
}; /* namespace raptor_convex_decomposition */
