#include <vector>

#include "mgf_parser.h"
extern "C"
{
#include <stdio.h>
#include "parser.h"
}


namespace raptor_raytracer
{
bool                cur_light   = false;
light_list          *global_l   = nullptr;
primitive_list      *global_e   = nullptr;
list<material *>    *global_m   = nullptr;
material            *cur_mat    = nullptr;


void appply_material()
{
    if (c_cmaterial->clock != 0)                    /* apply material */
    {
//        if (c_cmaterial->ed < 100.0)
//        {
            cur_light   = false;
            cur_mat     = new cook_torrance_cxy(c_cmaterial->rd_c.cx, c_cmaterial->rd_c.cy, c_cmaterial->rd, 
                                                c_cmaterial->td     , c_cmaterial->rs     , c_cmaterial->ts,
                                                c_cmaterial->rs_a   , c_cmaterial->ts_a   , c_cmaterial->nr,
                                                c_cmaterial->ni);
//        }
//        else
//        {
//            cur_light   = true;
//            cur_mat     = new light_shader(min(static_cast<float>(C_CMAXV * 10), (static_cast<float>(c_cmaterial->ed))));
//        }
        
        global_m->push_back(cur_mat);
        c_cmaterial->clock = 0;
    }
}


//int mgf_cone_handler(int ac, char **av)             /* Cone handling routine */
//{
//	C_VERTEX	*vp0,  *vp1;                        /* vertex structure pointer */
//	FVECT	    vert0, vert1;                       /* vertex point location */
//
//	if (ac != 5)			                        /* check # arguments */
//    {
//		return(MG_EARGC);
//    }
//    
//    if ((vp0 = c_getvert(av[1])) == NULL)	        /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//
//    if ((vp1 = c_getvert(av[3])) == NULL)	        /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//    
//    xf_xfmpoint(vert0, vp0->p);		                /* apply transform */
//    xf_xfmpoint(vert1, vp1->p);
//    float r0 = xf_scale(atof(av[2]));
//    float r1 = xf_scale(atof(av[4]));
//    
//    appply_material();                              /* apply material */
//    
//    /* Declare the cone */
//    if (cur_light)
//    {
//        cout << "cone light" << endl;
//    }
//    point_t a(vert0[0], vert0[1], vert0[2]);
//    point_t b(vert1[0], vert1[1], vert1[2]);
//    new_cone(global_e, global_l, cur_mat, a, b, r0, r1, cur_light);
//
//	return(MG_OK);			                        /* normal exit */
//}
//
//
//int mgf_cylinder_handler(int ac, char **av)         /* Cylinder handling routine */
//{
//	C_VERTEX	*vp0,  *vp1;                        /* vertex structure pointer */
//	FVECT	    vert0, vert1;                       /* vertex point location */
//
//	if (ac != 4)			                        /* check # arguments */
//    {
//		return(MG_EARGC);
//    }
//    
//    if ((vp0 = c_getvert(av[1])) == NULL)	        /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//
//    if ((vp1 = c_getvert(av[3])) == NULL)	        /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//    
//    xf_xfmpoint(vert0, vp0->p);		                /* apply transform */
//    xf_xfmpoint(vert1, vp1->p);
//    float r = xf_scale(atof(av[2]));
//    
//    appply_material();                              /* apply material */
//    
//    /* Declare the cyclinder */
//    if (cur_light)
//    {
//        cout << "cyclinder light" << endl;
//    }
//    point_t a(vert0[0], vert0[1], vert0[2]);
//    point_t b(vert1[0], vert1[1], vert1[2]);
//    new_cylinder(global_e, global_l, cur_mat, a, b, r, cur_light);
//
//	return(MG_OK);			                        /* normal exit */
//}


// int mgf_prism_handler(int ac, char **av)            /* prism handling routine */
// {
// //    C_VERTEX    *vp;                                /* vertex structure pointer */
// //    FVECT        vert;                               /* vertex point location */

//     assert(false);                                 /* Make known when a prism is found */

//     appply_material();                              /* apply material */
    
//     return(MG_OK);                                    /* normal exit */
// }


//int mgf_ring_handler(int ac, char **av)	            /* Ring handling routine */
//{
//	C_VERTEX	*vp;	                            /* vertex structure pointer */
//	FVECT	    vert;		                        /* vertex point location */
//
//	if (ac != 4)			                        /* check # arguments */
//    {
//		return(MG_EARGC);
//    }
//    
//    if ((vp = c_getvert(av[1])) == NULL)	        /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//    xf_xfmpoint(vert, vp->p);		                /* apply transform */
//    float i = xf_scale(atof(av[2]));
//    float r = xf_scale(atof(av[3]));
//    
//    appply_material();                              /* apply material */
//    
//    /* Declare the ring */
//    if (cur_light)
//    {
//        cout << "ring light" << endl;
//    }
//    new_ring(global_e, global_l, cur_mat, point_t(vert[0], vert[1], vert[2]), vector_t(vp->n[0], vp->n[1], vp->n[2]), r, i, cur_light);
//
//	return(MG_OK);			                        /* normal exit */
//}
//
//
//int mgf_sphere_handler(int ac, char **av)           /* Sphere handling routine */
//{
//	C_VERTEX	*vp;                                /* vertex structure pointer */
//	FVECT	    vert;                               /* vertex point location */
//
//	if (ac != 3)			                        /* check # arguments */
//    {
//		return(MG_EARGC);
//    }
//    
//    if ((vp = c_getvert(av[1])) == NULL)            /* vertex from name */
//    {
//    	return(MG_EUNDEF);
//    }
//
//    xf_xfmpoint(vert, vp->p);                       /* apply transform */
//    float r = xf_scale(atof(av[2]));
//    
//    appply_material();                              /* apply material */
//    
//    /* Declare the sphere */
//    if (cur_light)
//    {
//        cout << "sphere light" << endl;
//    }
//    new_sphere(global_e, global_l, cur_mat, point_t(vert[0], vert[1], vert[2]), r, cur_light);
//
//	return(MG_OK);			                        /* normal exit */
//}
//
//
//int mgf_torus_handler(int ac, char **av)            /* Torus handling routine */
//{
////    C_VERTEX	*vp;                                /* vertex structure pointer */
////    FVECT	    vert;                               /* vertex point location */
//
//	assert(false);                                 /* Make known when a torus is found */
//
//    appply_material();                              /* apply material */
//    
//	return(MG_OK);			                        /* normal exit */
//}


int mgf_face_handler(int ac, char **av)             /* face handling routine */
{
    vector<point_t> points;
    vector<point_t> normals;
	C_VERTEX        *vp;	                    /* vertex structure pointer */
	FVECT           vert;		                /* vertex point location */
	FVECT           norm;		                /* vertex normal */
	int	            i;

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
    if ((vp = c_getvert(av[1])) == NULL)
    {
    	return(MG_EUNDEF);
    }
    
    /* apply transform */
    xf_rotvect(norm, vp->n);

    /* No vertex normals */
    if ((norm[0] == 0.0) && (norm[1] == 0.0) && (norm[2] == 0.0))
    {
    	for (i = 1; i < ac; i++)
        {
            /* get vertex from name */
    		if ((vp = c_getvert(av[i])) == NULL)
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
    	for (i = 1; i < ac; i++)
        {
    		if ((vp = c_getvert(av[i])) == NULL)	    /* vertex from name */
            {
    			return(MG_EUNDEF);
            }
            
            /* apply transform */
            xf_xfmpoint(vert, vp->p);
            xf_rotvect(norm, vp->n);

            points.emplace_back(vert[0], vert[1], vert[2]);

            normals.emplace_back(norm[0], norm[1], norm[2]);
            assert(normals.back() != 0.0);
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
    const char          *mgf_file,
    light_list          &l, 
    primitive_list      &e,
    list<material *>    &m)
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
//    mg_ehand[MG_E_CONE]     = mgf_cone_handler;
//    mg_ehand[MG_E_CYL]      = mgf_cylinder_handler;
    mg_ehand[MG_E_FACE]     = mgf_face_handler;     /* ours                     */
//    mg_ehand[MG_E_PRISM]    = mgf_prism_handler;
//    mg_ehand[MG_E_RING]     = mgf_ring_handler;
//    mg_ehand[MG_E_SPH]      = mgf_sphere_handler;
//    mg_ehand[MG_E_TORUS]    = mgf_torus_handler;
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
//    mg_ehand[MG_E_CSPEC]    = c_hcolor;             /* support "cspec" entity   */
    mg_ehand[MG_E_CCT]      = c_hcolor;             /* support "cct" entity     */
    mg_ehand[MG_E_CMIX]     = c_hcolor;             /* support "cmix" entity    */
    mg_ehand[MG_E_OBJECT]   = obj_handler;          /* parser lib               */
    mg_init();                                      /* initialize parser        */
/*
MG_E_INCLUDE
MG_E_IES	
*/

    /* Parse */
    if (mg_load(mgf_file) != MG_OK)
    {
        cout << "MGF parsing failed" << endl;
    }

    return;
}
}; /* namespace raptor_raytracer */
