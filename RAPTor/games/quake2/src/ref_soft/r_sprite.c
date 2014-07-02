/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/
// r_sprite.c
#include "r_local.h"

#ifdef RAPTOR_RAYTRACE
#include "raptor.h"
#endif//RAPTOR_RAYTRACE

extern polydesc_t r_polydesc;

void R_BuildPolygonFromSurface(msurface_t *fa);
void R_PolygonCalculateGradients (void);

extern void R_PolyChooseSpanletRoutine( float alpha, qboolean isturbulent );

extern vec5_t r_clip_verts[2][MAXWORKINGVERTS+2];

extern void	R_ClipAndDrawPoly( float alpha, qboolean isturbulent, qboolean textured );

/*
** R_DrawSprite
**
** Draw currententity / currentmodel as a single texture
** mapped polygon
*/
void R_DrawSprite (void)
{
	vec5_t		*pverts;
	vec3_t		left, up, right, down;
	dsprite_t	*s_psprite;
	dsprframe_t	*s_psprframe;


	s_psprite = (dsprite_t *)currentmodel->extradata;
#if 0
	if (currententity->frame >= s_psprite->numframes
		|| currententity->frame < 0)
	{
		ri.Con_Printf (PRINT_ALL, "No such sprite frame %i\n", 
			currententity->frame);
		currententity->frame = 0;
	}
#endif
	currententity->frame %= s_psprite->numframes;

	s_psprframe = &s_psprite->frames[currententity->frame];

	r_polydesc.pixels       = currentmodel->skins[currententity->frame]->pixels[0];
	r_polydesc.pixel_width  = s_psprframe->width;
	r_polydesc.pixel_height = s_psprframe->height;
	r_polydesc.dist         = 0;

	// generate the sprite's axes, completely parallel to the viewplane.
	VectorCopy (vup, r_polydesc.vup);
	VectorCopy (vright, r_polydesc.vright);
	VectorCopy (vpn, r_polydesc.vpn);

// build the sprite poster in worldspace
	VectorScale (r_polydesc.vright, 
		s_psprframe->width - s_psprframe->origin_x, right);
	VectorScale (r_polydesc.vup, 
		s_psprframe->height - s_psprframe->origin_y, up);
	VectorScale (r_polydesc.vright,
		-s_psprframe->origin_x, left);
	VectorScale (r_polydesc.vup,
		-s_psprframe->origin_y, down);

	// invert UP vector for sprites
	VectorInverse( r_polydesc.vup );

	pverts = r_clip_verts[0];

	pverts[0][0] = r_entorigin[0] + up[0] + left[0];
	pverts[0][1] = r_entorigin[1] + up[1] + left[1];
	pverts[0][2] = r_entorigin[2] + up[2] + left[2];
	pverts[0][3] = 0;
	pverts[0][4] = 0;

	pverts[1][0] = r_entorigin[0] + up[0] + right[0];
	pverts[1][1] = r_entorigin[1] + up[1] + right[1];
	pverts[1][2] = r_entorigin[2] + up[2] + right[2];
	pverts[1][3] = s_psprframe->width;
	pverts[1][4] = 0;

	pverts[2][0] = r_entorigin[0] + down[0] + right[0];
	pverts[2][1] = r_entorigin[1] + down[1] + right[1];
	pverts[2][2] = r_entorigin[2] + down[2] + right[2];
	pverts[2][3] = s_psprframe->width;
	pverts[2][4] = s_psprframe->height;

	pverts[3][0] = r_entorigin[0] + down[0] + left[0];
	pverts[3][1] = r_entorigin[1] + down[1] + left[1];
	pverts[3][2] = r_entorigin[2] + down[2] + left[2];
	pverts[3][3] = 0;
	pverts[3][4] = s_psprframe->height;

	r_polydesc.nump = 4;
	r_polydesc.s_offset = ( r_polydesc.pixel_width  >> 1);
	r_polydesc.t_offset = ( r_polydesc.pixel_height >> 1);
	VectorCopy( modelorg, r_polydesc.viewer_position );

	r_polydesc.stipple_parity = 1;

#ifdef RAPTOR_RAYTRACE
{
    vec3_t dummy = {0,0,0};
    vec3_t transformed[4];
    int i, j;

    assert(currentmodel->skins[currententity->frame]->pixels[0]);
    assert(currentmodel->skins[currententity->frame]->width);
    assert(currentmodel->skins[currententity->frame]->height);

    for (i=0; i<4; ++i)
    {
        for (j=0; j<3; ++j)
        {
            vec3_t local;
            VectorSubtract(pverts[i], r_origin, local);
            TransformVector(local, transformed[i]);
        }
    }
    raptor_push_triangle_texture(currentmodel->skins[currententity->frame]->pixels[0],
                                 currentmodel->skins[currententity->frame]->width,
                                 currentmodel->skins[currententity->frame]->height,
                                 dummy, dummy, dummy,
                                 pverts[0][3],
                                 pverts[0][4],
                                 pverts[1][3],
                                 pverts[1][4],
                                 pverts[2][3],
                                 pverts[2][4]);
    raptor_push_triangle(transformed[0], transformed[1], transformed[2], 1, 0, 0, 0, raptor_fake_light);
    printf("t: %f %f %f\n", transformed[0][0], transformed[0][1], transformed[0][2]);
    raptor_push_triangle_texture(currentmodel->skins[currententity->frame]->pixels[0],
                                 currentmodel->skins[currententity->frame]->width,
                                 currentmodel->skins[currententity->frame]->height,
                                 dummy, dummy, dummy,
                                 pverts[3][3],
                                 pverts[3][4],
                                 pverts[0][3],
                                 pverts[0][4],
                                 pverts[2][3],
                                 pverts[2][4]);
    raptor_push_triangle(transformed[3], transformed[0], transformed[2], 1, 0, 0, 0, raptor_fake_light);
}
#endif//RAPTOR_RAYTRACE

	if ( currententity->flags & RF_TRANSLUCENT )
		R_ClipAndDrawPoly ( currententity->alpha, false, true );
	else
		R_ClipAndDrawPoly ( 1.0F, false, true );
	r_polydesc.stipple_parity = 0;
}

