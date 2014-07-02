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

// draw.c

#include "r_local.h"

#ifdef RAPTOR_RAYTRACE
    #include "SDL.h"
    #include "raptor.h"
#endif//RAPTOR_RAYTRACE

image_t		*draw_chars;				// 8*8 graphic characters

//=============================================================================

/*
================
Draw_FindPic
================
*/
image_t *Draw_FindPic (char *name)
{
	image_t	*image;
	char	fullname[MAX_QPATH];

	if (name[0] != '/' && name[0] != '\\')
	{
		Com_sprintf (fullname, sizeof(fullname), "pics/%s.pcx", name);
		image = R_FindImage (fullname, it_pic);
	}
	else
		image = R_FindImage (name+1, it_pic);

	return image;
}



/*
===============
Draw_InitLocal
===============
*/
void Draw_InitLocal (void)
{
	draw_chars = Draw_FindPic ("conchars");
}



/*
================
Draw_Char

Draws one 8*8 graphics character
It can be clipped to the top of the screen to allow the console to be
smoothly scrolled off.
================
*/
void Draw_Char (int x, int y, int num)
{
	byte			*dest;
	byte			*source;
	int				drawline;	
	int				row, col;

	num &= 255;

	if (num == 32 || num == 32+128)
		return;

	if (y <= -8)
		return;			// totally off screen

//	if ( ( y + 8 ) >= vid.height )
	if ( ( y + 8 ) > vid.height )		// PGM - status text was missing in sw...
		return;

#ifdef PARANOID
	if (y > vid.height - 8 || x < 0 || x > vid.width - 8)
		ri.Sys_Error (ERR_FATAL,"Con_DrawCharacter: (%i, %i)", x, y);
	if (num < 0 || num > 255)
		ri.Sys_Error (ERR_FATAL,"Con_DrawCharacter: char %i", num);
#endif

	row = num>>4;
	col = num&15;
	source = draw_chars->pixels[0] + (row<<10) + (col<<3);

	if (y < 0)
	{	// clipped
		drawline = 8 + y;
		source -= 128*y;
		y = 0;
	}
	else
		drawline = 8;


#ifdef RAPTOR_RAYTRACE
        dest = vid.buffer + y*vid.rowbytes + x*4;
#else
	dest = vid.buffer + y*vid.rowbytes + x;
#endif

	while (drawline--)
	{
#ifdef RAPTOR_RAYTRACE
                int h;
                for (h=0; h<8; ++h) {
                    if (source[h] != TRANSPARENT_COLOR) {
                        byte colour = source[h];
                        byte r = ( d_8to24table[colour] ) & 0xFF;
                        byte g = ( d_8to24table[colour] >> 8 ) & 0xFF;
                        byte b = ( d_8to24table[colour] >> 16 ) & 0xFF;

                        RAPTOR_SWAP_R_B(r, b);

                        *((unsigned int*)&dest[h*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);

//                        dest[h*4]   = b;
//                        dest[h*4+1] = g;
//                        dest[h*4+2] = r;
                    }
                }
#else
		if (source[0] != TRANSPARENT_COLOR)
			dest[0] = source[0];
		if (source[1] != TRANSPARENT_COLOR)
			dest[1] = source[1];
		if (source[2] != TRANSPARENT_COLOR)
			dest[2] = source[2];
		if (source[3] != TRANSPARENT_COLOR)
			dest[3] = source[3];
		if (source[4] != TRANSPARENT_COLOR)
			dest[4] = source[4];
		if (source[5] != TRANSPARENT_COLOR)
			dest[5] = source[5];
		if (source[6] != TRANSPARENT_COLOR)
			dest[6] = source[6];
		if (source[7] != TRANSPARENT_COLOR)
			dest[7] = source[7];
#endif//RAPTOR_RAYTRACE
		source += 128;
		dest += vid.rowbytes;
	}
}

/*
=============
Draw_GetPicSize
=============
*/
void Draw_GetPicSize (int *w, int *h, char *pic)
{
	image_t *gl;

	gl = Draw_FindPic (pic);
	if (!gl)
	{
		*w = *h = -1;
		return;
	}
	*w = gl->width;
	*h = gl->height;
}

/*
=============
Draw_StretchPicImplementation
=============
*/
void Draw_StretchPicImplementation (int x, int y, int w, int h, image_t	*pic)
{
	byte			*dest, *source;
	int				v, u, sv;
	int				height;
	int				f, fstep;
	int				skip;

	if ((x < 0) ||
		(x + w > vid.width) ||
		(y + h > vid.height))
	{
		ri.Sys_Error (ERR_FATAL,"Draw_Pic: bad coordinates");
	}

	height = h;
	if (y < 0)
	{
		skip = -y;
		height += y;
		y = 0;
	}
	else
		skip = 0;

#ifdef RAPTOR_RAYTRACE
        dest = vid.buffer + y*vid.rowbytes + x*4;
#else
    dest = vid.buffer + y*vid.rowbytes + x;
#endif

	for (v=0 ; v<height ; v++, dest += vid.rowbytes)
	{
		sv = (skip + v)*pic->height/h;
		source = pic->pixels[0] + sv*pic->width;
#ifdef RAPTOR_RAYTRACE
		f = 0;
        fstep = pic->width*0x10000/w;
        for (u=0 ; u<w ; u+=4)
        {
            byte colour = source[f>>16];
            byte r      = ( d_8to24table[colour]       ) & 0xFF;
            byte g      = ( d_8to24table[colour] >>  8 ) & 0xFF;
            byte b      = ( d_8to24table[colour] >> 16 ) & 0xFF;
            RAPTOR_SWAP_R_B(r, b);
            *((unsigned int*)&dest[u*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
            f += fstep;

            colour          = source[f>>16];
            r               = ( d_8to24table[colour]       ) & 0xFF;
            g               = ( d_8to24table[colour] >>  8 ) & 0xFF;
            b               = ( d_8to24table[colour] >> 16 ) & 0xFF;
            RAPTOR_SWAP_R_B(r, b);
            *((unsigned int*)&dest[(u+1)*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
            f += fstep;

            colour          = source[f>>16];
            r               = ( d_8to24table[colour]       ) & 0xFF;
            g               = ( d_8to24table[colour] >>  8 ) & 0xFF;
            b               = ( d_8to24table[colour] >> 16 ) & 0xFF;
            RAPTOR_SWAP_R_B(r, b);
            *((unsigned int*)&dest[(u+2)*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
            f += fstep;

            colour          = source[f>>16];
            r               = ( d_8to24table[colour]       ) & 0xFF;
            g               = ( d_8to24table[colour] >>  8 ) & 0xFF;
            b               = ( d_8to24table[colour] >> 16 ) & 0xFF;
            RAPTOR_SWAP_R_B(r, b);
            *((unsigned int*)&dest[(u+3)*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
            f += fstep;
        }
#else
		if (w == pic->width)
			memcpy (dest, source, w);
		else
		{
			f = 0;
			fstep = pic->width*0x10000/w;
			for (u=0 ; u<w ; u+=4)
			{
				dest[u] = source[f>>16];
				f += fstep;
				dest[u+1] = source[f>>16];
				f += fstep;
				dest[u+2] = source[f>>16];
				f += fstep;
				dest[u+3] = source[f>>16];
				f += fstep;
			}
		}
#endif
	}
}

/*
=============
Draw_StretchPic
=============
*/
void Draw_StretchPic (int x, int y, int w, int h, char *name)
{
	image_t	*pic;

	pic = Draw_FindPic (name);
	if (!pic)
	{
		ri.Con_Printf (PRINT_ALL, "Can't find pic: %s\n", name);
		return;
	}
	Draw_StretchPicImplementation (x, y, w, h, pic);
}

/*
=============
Draw_StretchRaw
=============
*/
void Draw_StretchRaw (int x, int y, int w, int h, int cols, int rows, byte *data)
{
	image_t	pic;

	pic.pixels[0] = data;
	pic.width = cols;
	pic.height = rows;
	Draw_StretchPicImplementation (x, y, w, h, &pic);
}

/*
=============
Draw_Pic
=============
*/
void Draw_Pic (int x, int y, char *name)
{
	image_t			*pic;
	byte			*dest, *source;
	int				v, u;
	int				tbyte;
	int				height;

	pic = Draw_FindPic (name);
	if (!pic)
	{
		ri.Con_Printf (PRINT_ALL, "Can't find pic: %s\n", name);
		return;
	}

	if ((x < 0) ||
		(x + pic->width > vid.width) ||
		(y + pic->height > vid.height))
		return;	//	ri.Sys_Error (ERR_FATAL,"Draw_Pic: bad coordinates");

	height = pic->height;
	source = pic->pixels[0];
	if (y < 0)
	{
		height += y;
		source += pic->width*-y;
		y = 0;
	}
#ifdef RAPTOR_RAYTRACE
	dest = vid.buffer + y * vid.rowbytes + x*4;
#else
        dest = vid.buffer + y * vid.rowbytes + x;
#endif

#ifndef RAPTOR_RAYTRACE
	if (!pic->transparent) {
#else
	if (1) {
#endif
		for (v=0 ; v<height ; v++)
		{
#ifdef RAPTOR_RAYTRACE
			int h;
			for (h=0; h<pic->width; ++h) {
                if (!pic->transparent || (source[h] != TRANSPARENT_COLOR)) {
                    byte colour = source[h];
                    byte r = ( d_8to24table[colour] ) & 0xFF;
                    byte g = ( d_8to24table[colour] >> 8 ) & 0xFF;
                    byte b = ( d_8to24table[colour] >> 16 ) & 0xFF;

                    RAPTOR_SWAP_R_B(r, b);

                    *((unsigned int*)&dest[h*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
                }
            }
#else
			memcpy (dest, source, pic->width);
#endif
			dest += vid.rowbytes;
			source += pic->width;
		}
	}
	else
	{
		if (pic->width & 7)
		{	// general
			for (v=0 ; v<height ; v++)
			{
				for (u=0 ; u<pic->width ; u++)
					if ( (tbyte=source[u]) != TRANSPARENT_COLOR)
						dest[u] = tbyte;

				dest += vid.rowbytes;
				source += pic->width;
			}
		}
		else
		{	// unwound
			for (v=0 ; v<height ; v++)
			{
				for (u=0 ; u<pic->width ; u+=8)
				{
					if ( (tbyte=source[u]) != TRANSPARENT_COLOR)
						dest[u] = tbyte;
					if ( (tbyte=source[u+1]) != TRANSPARENT_COLOR)
						dest[u+1] = tbyte;
					if ( (tbyte=source[u+2]) != TRANSPARENT_COLOR)
						dest[u+2] = tbyte;
					if ( (tbyte=source[u+3]) != TRANSPARENT_COLOR)
						dest[u+3] = tbyte;
					if ( (tbyte=source[u+4]) != TRANSPARENT_COLOR)
						dest[u+4] = tbyte;
					if ( (tbyte=source[u+5]) != TRANSPARENT_COLOR)
						dest[u+5] = tbyte;
					if ( (tbyte=source[u+6]) != TRANSPARENT_COLOR)
						dest[u+6] = tbyte;
					if ( (tbyte=source[u+7]) != TRANSPARENT_COLOR)
						dest[u+7] = tbyte;
				}
				dest += vid.rowbytes;
				source += pic->width;
			}
		}
	}
}

/*
=============
Draw_TileClear

This repeats a 64*64 tile graphic to fill the screen around a sized down
refresh window.
=============
*/
void Draw_TileClear (int x, int y, int w, int h, char *name)
{
	int			i, j;
	byte		*psrc;
	byte		*pdest;
	image_t		*pic;
	int			x2;

	if (x < 0)
	{
		w += x;
		x = 0;
	}
	if (y < 0)
	{
		h += y;
		y = 0;
	}
	if (x + w > vid.width)
		w = vid.width - x;
	if (y + h > vid.height)
		h = vid.height - y;
	if (w <= 0 || h <= 0)
		return;

	pic = Draw_FindPic (name);
	if (!pic)
	{
		ri.Con_Printf (PRINT_ALL, "Can't find pic: %s\n", name);
		return;
	}
	x2 = x + w;

	pdest = vid.buffer + y*vid.rowbytes;
	for (i=0 ; i<h ; i++, pdest += vid.rowbytes)
	{
		psrc = pic->pixels[0] + pic->width * ((i+y)&63);
#ifdef RAPTOR_RAYTRACE
        for (j=x ; j<x2 ; j++)
        {
            byte c = psrc[j&63];
            byte r = ( d_8to24table[c]       ) & 0xFF;
            byte g = ( d_8to24table[c] >>  8 ) & 0xFF;
            byte b = ( d_8to24table[c] >> 16 ) & 0xFF;

            RAPTOR_SWAP_R_B(r, b);

            *((unsigned int*)&pdest[j*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
        }
#else
        for (j=x ; j<x2 ; j++)
            pdest[j] = psrc[j&63];
#endif
	}
}


/*
=============
Draw_Fill

Fills a box of pixels with a single color
=============
*/
void Draw_Fill (int x, int y, int w, int h, int c)
{
	byte			*dest;
	int				u, v;

	if (x+w > vid.width)
		w = vid.width - x;
	if (y+h > vid.height)
		h = vid.height - y;
	if (x < 0)
	{
		w += x;
		x = 0;
	}
	if (y < 0)
	{
		h += y;
		y = 0;
	}
	if (w < 0 || h < 0)
		return;
#ifdef RAPTOR_RAYTRACE
    dest = vid.buffer + y*vid.rowbytes + x*4;
#else
	dest = vid.buffer + y*vid.rowbytes + x;
#endif
	for (v=0 ; v<h ; v++, dest += vid.rowbytes)
		for (u=0 ; u<w ; u++)
		{
#ifdef RAPTOR_RAYTRACE
            byte r = ( d_8to24table[c]       ) & 0xFF;
            byte g = ( d_8to24table[c] >>  8 ) & 0xFF;
            byte b = ( d_8to24table[c] >> 16 ) & 0xFF;

            RAPTOR_SWAP_R_B(r, b);

            *((unsigned int*)&dest[u*4]) = SDL_MapRGB((SDL_PixelFormat*)vid.pixelformat, r, g, b);
#else
		    dest[u] = c;
#endif
		}
}
//=============================================================================

/*
================
Draw_FadeScreen

================
*/
void Draw_FadeScreen (void)
{
	int			x,y;
	byte		*pbuf;
	int	t;

	for (y=0 ; y<vid.height ; y++)
	{
		pbuf = (byte *)(vid.buffer + vid.rowbytes*y);
		t = (y & 1) << 1;

		for (x=0 ; x<vid.width ; x++)
		{
			if ((x & 3) != t)
			{
#ifdef RAPTOR_RAYTRACE
				pbuf[x*4] = pbuf[x*4+1] = pbuf[x*4+2] = 0;
#else
                pbuf[x] = 0;
#endif
			}
		}
	}
}
