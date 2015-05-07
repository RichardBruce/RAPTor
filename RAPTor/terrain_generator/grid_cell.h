#ifndef __GRID_CELL_H__
#define __GRID_CELL_H__

/* Common headers */
#include "common.h"
#include "point_t.h"


namespace raptor_terrain
{
enum class neighbour_t : char { NORTH = 0, WEST = 1, SOUTH = 2, EAST = 3 };

neighbour_t turn_left(const neighbour_t n)
{
    return static_cast<neighbour_t>((static_cast<int>(n) + 1) & 0x3);
}

neighbour_t turn_right(const neighbour_t n)
{
    return static_cast<neighbour_t>((static_cast<int>(n) - 1) & 0x3);
}

neighbour_t turn_around(const neighbour_t n)
{
    return static_cast<neighbour_t>(static_cast<int>(n) ^ 0x2);
}


class grid_cell
{
    public :
        grid_cell(const point_t *const verts, const int n, const int s, const int e, const int w, const int v0, const int v1, const int v2, const int v3)
        : _verts(verts), _n(n), _s(s), _e(e), _w(w), _v0(v0), _v1(v1), _v2(v2), _v3(v3) { };

        /* Getters */
        int neighbour(const neighbour_t n) const
        {
            switch (n)
            {
                case neighbour_t::NORTH  : return north();
                case neighbour_t::SOUTH  : return south();
                case neighbour_t::EAST   : return east();
                case neighbour_t::WEST   : return west();
                default     : assert(false);
            }
        }

        int north() const { return _n; }
        int south() const { return _s; }
        int east()  const { return _e; }
        int west()  const { return _w; }

        int top_left()      const { return _v0; }
        int bottom_left()   const { return _v1; }
        int bottom_right()  const { return _v2; }
        int top_right()     const { return _v3; }

        int corner_entering(const neighbour_t moving, const neighbour_t turning) const
        {
            assert(moving != turning);
            switch (moving)
            {
                case neighbour_t::NORTH  : 
                    switch(turning)
                    {
                        case neighbour_t::SOUTH : return top_right();
                        case neighbour_t::EAST  : return bottom_right();
                        case neighbour_t::WEST  : return top_right();
                        case neighbour_t::NORTH : assert(false);
                    }
                case neighbour_t::SOUTH  : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return bottom_left();
                        case neighbour_t::EAST  : return bottom_left();
                        case neighbour_t::WEST  : return top_left();
                        case neighbour_t::SOUTH : assert(false);
                    }
                case neighbour_t::EAST   : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return bottom_right();
                        case neighbour_t::SOUTH : return bottom_left();
                        case neighbour_t::WEST  : return bottom_right();
                        case neighbour_t::EAST  : assert(false);
                    }
                case neighbour_t::WEST   : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return top_right();
                        case neighbour_t::SOUTH : return top_left();
                        case neighbour_t::EAST  : return top_left();
                        case neighbour_t::WEST  : assert(false);
                    }
                default : assert(false);
            }
        }

        int corner_leaving(const neighbour_t moving, const neighbour_t turning) const
        {
            assert(moving != turning);
            switch (moving)
            {
                case neighbour_t::NORTH  : 
                    switch(turning)
                    {
                        case neighbour_t::SOUTH : return top_left();
                        case neighbour_t::EAST  : return bottom_right();
                        case neighbour_t::WEST  : return top_right();
                        case neighbour_t::NORTH : assert(false);
                    }
                case neighbour_t::SOUTH  : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return bottom_right();
                        case neighbour_t::EAST  : return bottom_left();
                        case neighbour_t::WEST  : return top_left();
                        case neighbour_t::SOUTH : assert(false);
                    }
                case neighbour_t::EAST   : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return bottom_right();
                        case neighbour_t::SOUTH : return bottom_left();
                        case neighbour_t::WEST  : return top_right();
                        case neighbour_t::EAST  : assert(false);
                    }
                case neighbour_t::WEST   : 
                    switch(turning)
                    {
                        case neighbour_t::NORTH : return top_right();
                        case neighbour_t::SOUTH : return top_left();
                        case neighbour_t::EAST  : return bottom_left();
                        case neighbour_t::WEST  : assert(false);
                    }
                default : assert(false);
            }
        }

        int furthest_index(const neighbour_t moving) const
        {
            switch(moving)
            {
                case neighbour_t::NORTH : return top_right();
                case neighbour_t::SOUTH : return bottom_left();
                case neighbour_t::EAST  : return bottom_right();
                case neighbour_t::WEST  : return top_left();
                default                 : assert(false);
            }
        }


        /* Check if the grid cells have roughly the same slope (within tolerance min_cos) */
        /* This function assume that it doesnt matter which way up the grid cell is and that the grid cell has an area */
        bool can_merge(const grid_cell &c, const float min_cos) const
        {
            const point_t n(normalise(cross_product(_verts[_v1]  - _verts[_v0], _verts[_v3] - _verts[_v0])));
            const point_t n_c(normalise(cross_product(c._verts[c._v1]  - c._verts[c._v0], c._verts[c._v3] - c._verts[c._v0])));

            return (fabs(dot_product(n, n_c)) > min_cos);
        }

    private :
        const point_t *const    _verts; /* List of all vertices in the world        */
        const int               _n;     /* Neighbouring grid cell to the north      */
        const int               _s;     /* Neighbouring grid cell to the south      */
        const int               _e;     /* Neighbouring grid cell to the east       */
        const int               _w;     /* Neighbouring grid cell to the west       */
        const int               _v0;    /* Index in verts of top left vertex        */
        const int               _v1;    /* Index in verts of bottom left vertex     */
        const int               _v2;    /* Index in verts of bottom right vertex    */
        const int               _v3;    /* Index in verts of top right vertex       */
};


grid_cell** vertices_to_grid_cells(const point_t *const verts, const int y, const int cell_x, const int cell_y)
{
    /* Create grid cells for this set opf vertices */
    grid_cell** raw_grid_cells = new grid_cell *[cell_x * cell_y];
    for (int i = 0; i < cell_x; ++i)
    {
        /* Calculate row offsets for vertices */
        const int vert_row = i * y;
        const int vert_row_p1 = vert_row + y;

        /* Calculate row offsets for cells */
        const int cell_row = i * cell_y;
        const int cell_row_m1 = cell_row - cell_y;
        const int cell_row_p1 = ((i + 1) >= cell_x) ? -1 : (cell_row + cell_y);
        for (int j = 0; j < cell_y; ++j)
        {
            /* Calulate column offsets for cells */
            const int cell_m1 = (j == 0) ? -1 : (cell_row + j - 1);
            const int cell_p1 = ((j + 1) >= cell_y) ? -1 : (cell_row + j + 1);
            const int cell_row_p1_pj = (cell_row_p1 == -1) ? -1 : (cell_row_p1 + j);

            /* Create grid cell */
            raw_grid_cells[cell_row + j] = new grid_cell(verts, std::max(-1, cell_row_m1 + j), cell_row_p1_pj, cell_p1, cell_m1, vert_row + j, vert_row_p1 + j, vert_row_p1 + j + 1, vert_row + j + 1);
        }
    }

    return raw_grid_cells;
}
}; /* namespace raptor_terrain */

#endif /* #ifndef __GRID_CELL_H__ */
