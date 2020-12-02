#include <vector>

#include "mgf_parser.h"
extern "C"
{
#include <stdio.h>
#include "parser.h"
}

#include "polygon_to_triangles.h"


namespace raptor_raytracer
{
bool                    cur_light   = false;
light_list              *global_l   = nullptr;
primitive_store         *global_e   = nullptr;
std::list<material *>   *global_m   = nullptr;
material                *cur_mat    = nullptr;


void appply_material()
{
    if (c_cmaterial->clock != 0)                    /* apply material */
    {
        cur_light   = false;
        cur_mat     = new cook_torrance_cxy(c_cmaterial->rd_c.cx, c_cmaterial->rd_c.cy, c_cmaterial->rd, 
                                                c_cmaterial->td     , c_cmaterial->rs     , c_cmaterial->ts,
                                                c_cmaterial->rs_a   , c_cmaterial->ts_a   , c_cmaterial->nr,
                                                c_cmaterial->ni);
        global_m->push_back(cur_mat);
        c_cmaterial->clock = 0;
    }
}

int mgf_face_handler(int ac, char **av)     /* face handling routine */
{
    std::vector<point_t<>>  points;
    std::vector<point_t<>>  normals;
    C_VERTEX                *vp;            /* vertex structure pointer */
    FVECT                   vert;           /* vertex point location */
    FVECT                   norm;           /* vertex normal */
    int                     i;

    /* check # arguments */
    if (ac < 4)
    {
        return(MG_EARGC);
    }
    
    /* Make sure the first and last vertex arnt the same */
    if (strcmp(av[1], av[ac-1]) == 0)
    {
        ac--;
    }
    
    /* Check for vertex normals */
    if ((vp = c_getvert(av[1])) == nullptr)
    {
    	return(MG_EUNDEF);
    }
    
    /* apply transform */
    xf_rotvect(norm, vp->n);

    /* No vertex normals */
    if ((norm[0] == 0.0) && (norm[1] == 0.0) && (norm[2] == 0.0))
    {
    	for (i = 1; i < ac; ++i)
        {
            /* get vertex from name */
    		if ((vp = c_getvert(av[i])) == nullptr)
            {
        			return(MG_EUNDEF);
            }
            
            /* apply transform */
            xf_xfmpoint(vert, vp->p);
            
            points.emplace_back(vert[0], vert[1], vert[2]);
    	}

        /* apply material */
        appply_material();
    
        /* Convert face to triangles */
        face_to_triangles(global_e, global_l, points, cur_mat, cur_light);
    }
    /* Vertex normals */
    else
    {
    	for (i = 1; i < ac; ++i)
        {
    		if ((vp = c_getvert(av[i])) == nullptr)	    /* vertex from name */
            {
    			return(MG_EUNDEF);
            }
            
            /* apply transform */
            xf_xfmpoint(vert, vp->p);
            xf_rotvect(norm, vp->n);

            points.emplace_back(vert[0], vert[1], vert[2]);

            normals.emplace_back(norm[0], norm[1], norm[2]);
            assert(normals.back() != 0.0f);
    	}

        /* apply material */
        appply_material();
    
        /* Convert face to triangles */
        face_to_triangles(global_e, global_l, points, cur_mat, cur_light, &normals);

        /* clean up */
        normals.clear();
    }

    /* Clean up */
    points.clear();

    /* normal exit */
    return (MG_OK);
}


void mgf_parser(
    const char              *mgf_file,
    light_list              &l, 
    primitive_store         &e,
    std::list<material *>   &m)
{
    global_l = &l;
    global_e = &e;
    global_m = &m;

    c_cmaterial->clock      = 1;
    c_cmaterial->rd_c.cx    = 0.33f;
    c_cmaterial->rd_c.cy    = 0.33f;
    c_cmaterial->rd         = 0.5f;
    c_cmaterial->td         = 0.0f;
    c_cmaterial->rs         = 0.0f;
    c_cmaterial->ts         = 0.0f;
    c_cmaterial->rs_a       = 0.0f;
    c_cmaterial->ts_a       = 0.0f;
    c_cmaterial->nr         = 0.0f;
    c_cmaterial->ni         = 0.0f;
    
    /* Initialize dispatch table */
    mg_ehand[MG_E_FACE]     = mgf_face_handler;     /* ours                     */
    mg_ehand[MG_E_VERTEX]   = c_hvertex;            /* parser lib               */
    mg_ehand[MG_E_NORMAL]   = c_hvertex;            /* parser lib               */
    mg_ehand[MG_E_POINT]    = c_hvertex;            /* parser lib               */
    mg_ehand[MG_E_XF]       = xf_handler;           /* parser lib               */
    mg_ehand[MG_E_MATERIAL] = c_hmaterial;          /* support "m" entity       */
    mg_ehand[MG_E_ED]       = c_hmaterial;          /* support "ed" entity      */
    mg_ehand[MG_E_IR]       = c_hmaterial;          /* support "ir" entity      */
    mg_ehand[MG_E_RD]       = c_hmaterial;          /* support "rd" entity      */
    mg_ehand[MG_E_RS]       = c_hmaterial;          /* support "rs" entity      */
    mg_ehand[MG_E_SIDES]    = c_hmaterial;          /* support "sides" entity   */
    mg_ehand[MG_E_TD]       = c_hmaterial;          /* support "td" entity      */
    mg_ehand[MG_E_TS]       = c_hmaterial;          /* support "ts" entity      */
    mg_ehand[MG_E_COLOR]    = c_hcolor;	            /* support "c" entity       */
    mg_ehand[MG_E_CXY]      = c_hcolor;             /* support "cxy" entity     */
    mg_ehand[MG_E_CCT]      = c_hcolor;             /* support "cct" entity     */
    mg_ehand[MG_E_CMIX]     = c_hcolor;             /* support "cmix" entity    */
    mg_ehand[MG_E_OBJECT]   = obj_handler;          /* parser lib               */
    mg_init();                                      /* initialize parser        */

    /* Parse */
    if (mg_load(mgf_file) != MG_OK)
    {
        std::cout << "MGF parsing failed" << std::endl;
    }

    return;
}
}; /* namespace raptor_raytracer */
