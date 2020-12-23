#pragma once

/* Standard Headers */
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

/* Common headers */
#include "point2d.h"


class dac_convex_hull_2d
{
    public:
        dac_convex_hull_2d(std::shared_ptr<std::vector<point2d<long>>> &scratch, point2d<long> *const begin, point2d<long> *const end) : 
            _scratch(scratch), _begin(begin), _end(end) {  }

        size_t size() const { return _end - _begin; }

        point2d<long> &at(const size_t i) { return *(_begin + i); }
        point2d<long> &operator[](const size_t i) { return at(i); }

        const point2d<long> &at(const size_t i) const { return *(_begin + i); }
        const point2d<long> &operator[](const size_t i) const { return at(i); }

        dac_convex_hull_2d& merge(const dac_convex_hull_2d &rhs)
        {
            /* Find the closest points */
            const size_t a_idx = std::distance(_begin, std::max_element(_begin, _end, [](const auto &l, const auto &r){ return l.x < r.x;}));
            const size_t b_idx = std::distance(rhs._begin, std::min_element(rhs._begin, rhs._end, [](const auto &l, const auto &r){ return l.x < r.x;}));

            /* Move line up to find top tangent */
            auto [a_top, b_top] = find_tangent_top(rhs, a_idx, b_idx);

            /* Move line down to find bottom tangent */
            auto [a_bot, b_bot] = find_tangent_bottom(rhs, a_idx, b_idx);

            /* Merge vertices */
            a_top = increment(a_top);
            bool wrote_last_b = false;
            const size_t b_end = rhs.increment(b_bot);
            if (a_top != a_bot)
            {
                do
                {
                    at(a_top) = rhs.at(b_top);
                    wrote_last_b = (b_top == b_bot);
                    a_top = increment(a_top);
                    b_top = rhs.increment(b_top);
                } while ((a_top != a_bot) && (b_top != b_end));
            }

            /* Need more space */
            if ((a_top == a_bot) && !wrote_last_b)
            {
                const size_t scratch_size = std::distance(_begin + a_top, _end);
                std::move(_begin + a_top, _end, _scratch->begin());  /* Make space, it all needs shuffling up anyway */
                if (b_top <= b_bot)
                {
                    std::move(rhs._begin + b_top, rhs._begin + b_bot + 1, _begin + a_top);              /* Move down new data   */

                    point2d<long> *move_end = _begin + a_top + (b_bot - b_top) + 1;
                    std::move(_scratch->begin(), _scratch->begin() + scratch_size, move_end);           /* Return old data      */
                    _end = move_end + scratch_size;
                }
                /* Wrap around new two inserts */
                else if (b_top > b_bot)
                {
                    /* Oh dear, need to move out the lower data as well */
                    std::move(rhs._begin, rhs._begin + b_end, _scratch->begin() + scratch_size);

                    std::move(rhs._begin + b_top, rhs._end, _begin + a_top);                            /* Move down new data   */
                    std::move(_scratch->begin() + scratch_size, _scratch->begin() + scratch_size + b_end, _begin + a_top + rhs.size() - b_top);     /* Move down new data   */

                    point2d<long> *move_end = _begin + a_top + rhs.size() - b_top + b_end;
                    std::move(_scratch->begin(), _scratch->begin() + scratch_size, move_end);           /* Return old data      */
                    _end = move_end + scratch_size;
                }
            }
            /* Too much space */
            else if (b_top == b_end)
            {
                if (a_top < a_bot)
                {
                    std::move(_begin + a_bot, _end, _begin + a_top);
                    _end = _begin + a_top + std::distance(_begin + a_bot, _end);
                }
                /* Wrap around cant just update the ends */
                else if (a_top > a_bot)
                {
                    _end = _begin + a_top;
                    _begin = _begin + a_bot;
                }
            }

            return *this;
        }

        std::pair<size_t, size_t> find_tangent(const dac_convex_hull_2d &rhs, const long dir) const
        {
            const size_t a_idx = std::distance(_begin, std::max_element(_begin, _end, [](const auto &l, const auto &r) { return l.x < r.x; }));
            const size_t b_idx = std::distance(rhs._begin, std::min_element(rhs._begin, rhs._end, [](const auto &l, const auto &r) { return l.x < r.x; }));
            return find_tangent(rhs, a_idx, b_idx, dir);
        }

        std::pair<size_t, size_t> find_tangent(const dac_convex_hull_2d &rhs, size_t a_idx, size_t b_idx, const long dir) const
        {
            if (dir > 0)
            {
                return find_tangent_top(rhs, a_idx, b_idx);
            }
            else
            {
                return find_tangent_bottom(rhs, a_idx, b_idx);
            }
        }

    private:
        std::pair<size_t, size_t> find_tangent_top(const dac_convex_hull_2d &rhs, size_t a_idx, size_t b_idx) const
        {
            /* Begin moving a and b index towards the tangent one at a time */
            bool advanced = true;
            while (advanced)
            {
                advanced = false;
                const size_t next_a = decrement(a_idx);
                const long wa = winding(at(a_idx), rhs.at(b_idx), at(next_a));
                if ((wa > 0) || ((wa == 0) && (at(next_a).x < at(a_idx).x)))
                {
                    advanced = true;
                    a_idx = next_a;
                }

                const size_t next_b = rhs.increment(b_idx);
                const long wb = winding(at(a_idx), rhs.at(b_idx), rhs.at(next_b));
                if ((wb > 0) || ((wb == 0) && (rhs.at(next_b).x > rhs.at(b_idx).x)))
                {
                    advanced = true;
                    b_idx = next_b;
                }
            }

            return std::pair(a_idx, b_idx);
        }

        std::pair<size_t, size_t> find_tangent_bottom(const dac_convex_hull_2d &rhs, size_t a_idx, size_t b_idx) const
        {
            /* Begin moving a and b index towards the tangent one at a time */
            bool advanced = true;
            while (advanced)
            {
                advanced = false;
                const size_t next_a = increment(a_idx);
                const long wa = -winding(at(a_idx), rhs.at(b_idx), at(next_a));
                if ((wa > 0) || ((wa == 0) && (at(next_a).x < at(a_idx).x)))
                {
                    advanced = true;
                    a_idx = next_a;
                }

                const size_t next_b = rhs.decrement(b_idx);
                const long wb = -winding(at(a_idx), rhs.at(b_idx), rhs.at(next_b));
                if ((wb > 0) || ((wb == 0) && (rhs.at(next_b).x > rhs.at(b_idx).x)))
                {
                    advanced = true;
                    b_idx = next_b;
                }
            }

            return std::pair(a_idx, b_idx);
        }

        long increment(long idx) const
        {
            ++idx;
            if (idx == static_cast<long>(size()))
            {
                return 0;
            }
            
            return idx;
        }

        long decrement(long idx) const
        {
            --idx;
            if (idx < 0)
            {
                return size() - 1;
            }
            
            return idx;
        }

        std::shared_ptr<std::vector<point2d<long>>> _scratch;
        point2d<long> *                             _begin;
        point2d<long> *                             _end;
};


dac_convex_hull_2d build(std::shared_ptr<std::vector<point2d<long>>> &scratch, std::vector<point2d<long>> &vertices, const size_t begin, const size_t end)
{
    const size_t size = end - begin;
    if (size < 4)
    {
        /* Check winding for triangles */
        if (size == 3)
        {
            const long w = winding(vertices[begin], vertices[begin + 1], vertices[begin + 2]);
            if (w > 0)
            {
                std::swap(vertices[begin], vertices[begin + 1]);
            }
        }

        return dac_convex_hull_2d(scratch, &vertices[begin], &vertices[end]);
    }

    /* Recurse */
    const size_t mid = (begin + end) >> 1;
    auto left(build(scratch, vertices, begin, mid));
    const auto &right = build(scratch, vertices, mid, end);

    /* Return merged */
    return left.merge(right);
}

void clean_points(std::vector<point2d<long>> &vertices)
{
    /* Sort by x then y */
    std::sort(vertices.begin(), vertices.end(), [](const auto &lhs, const auto &rhs) { return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y)); });

    /* Remove all but the most extreme y for each x. This also removes any co-incident points */
    size_t wr_idx = 0;
    size_t max_idx = 0;
    size_t min_idx = 0;
    long x = vertices[0].x;
    long max_y = vertices[0].y;
    long min_y = vertices[0].y;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        /* Continuing the run, look for new y extremes */
        if (vertices[i].x == x)
        {
            if (vertices[i].y > max_y)
            {
                max_idx = i;
            }

            if (vertices[i].y < min_y)
            {
                min_idx = i;
            }
        }
        /* New run, move down the last one */
        else
        {
            vertices[wr_idx++] = vertices[min_idx];
            if (max_idx != min_idx)
            {
                vertices[wr_idx++] = vertices[max_idx];
            }

            max_idx = i;
            min_idx = i;
            x = vertices[i].x;
            max_y = vertices[i].y;
            min_y = vertices[i].y;
        }
    }

    /* Move for the last run */
    vertices[wr_idx++] = vertices[min_idx];
    if (max_idx != min_idx)
    {
        vertices[wr_idx++] = vertices[max_idx];
    }
    vertices.erase(vertices.begin() + wr_idx, vertices.end());
}

dac_convex_hull_2d build(std::vector<point2d<long>> &vertices)
{
    /* Clean up the points a bit */
    clean_points(vertices);

    /* Recurse */
    auto scratch(std::make_shared<std::vector<point2d<long>>>(vertices.size(), point2d<long>(0, 0)));
    return build(scratch, vertices, 0, vertices.size());
}
