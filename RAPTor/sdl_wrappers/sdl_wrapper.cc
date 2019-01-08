/* Standard headers */
#include <cassert>
#include <iostream>
#include <string>

/* SDL and TTF headers */
#include "SDL.h"
#include "SDL_ttf.h"


/* Display related constants */
const int BPP        = 4;
const int DEPTH      = 32;


/* Clean up SDL and TTF */
/*  screen is the SDL surface to be freed */
/*  font is the TTF font to be freed */
void sdl_clean_up(SDL_Window *const window, SDL_Renderer *const renderer, SDL_Texture *const texture, TTF_Font *const font)
{
    /* TTF clean up */
    if (font != nullptr)
    {
        TTF_CloseFont(font);
        TTF_Quit();
    }
    
    /* SDL clean up */
    if (texture != nullptr)
    {
        SDL_DestroyTexture(texture);
    }

    if (renderer != nullptr)
    {
        SDL_DestroyRenderer(renderer);
    }

    if (window != nullptr)
    {
        SDL_DestroyWindow(window);
    }

    if ((texture != nullptr) || (renderer != nullptr) || (window != nullptr))
    {
        SDL_Quit();
    }
}


/* Set up SDL for drawing to the screen. */
/*  window is the SDL window that will be available to draw to. */
/*  renderer is the SDL renderer that will be available to draw with. */
/*  texture is the SDL texture that will be available to draw on. */
/*  caption will be used as the window caption. */
/*  x_res is the x resolution of the screen */
/*  y_res is the y resolution of the screen */
/*  flags are flags for window creation */
/*  returns 0 if successfull, else an error code indicating where the error occurred */
int sdl_set_up(SDL_Window **window, SDL_Renderer **renderer, SDL_Texture **texture, const std::string &caption, const int x_res, const int y_res, const int flags)
{
    /* Initialise and lock the screen */
    if (SDL_Init(SDL_INIT_VIDEO) < 0 )
    {
        std::cout << "Error: SDL init failed with error: " << SDL_GetError() << std::endl;
        return 1;
    }
   
    if (!((*window) = SDL_CreateWindow(caption.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, x_res, y_res, flags)))
    {
        std::cout << "Error: SDL create window failed with error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 2;
    }

    if (flags & SDL_WINDOW_VULKAN)
    {
        std::cout << "Vulkan set up complete" << std::endl;
        return 0;
    }
    
    SDL_RendererInfo info;
    std::cout << "Got " << SDL_GetNumRenderDrivers() << " render drivers" << std::endl;
    for (int i = 0; i < SDL_GetNumRenderDrivers(); ++i)
    {
        SDL_GetRenderDriverInfo(i, &info);
        std::cout << "\t" << info.name << std::endl;
    }

    if (!((*renderer) = SDL_CreateRenderer(*window, -1, 0)))
    {
        std::cout << "Error: SDL create renderer failed with error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(*window);
        SDL_Quit();
        return 3;
    }

    if (!((*texture) = SDL_CreateTexture(*renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, x_res, y_res)))
    {
        std::cout << "Error: SDL create texture failed with error: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(*renderer);
        SDL_DestroyWindow(*window);
        SDL_Quit();
        return 4;
    }

    /* Clear the window */
    SDL_RenderClear(*renderer);
    SDL_RenderPresent(*renderer);

    return 0;
}


/* Set up SDL and TTF */
/*  window is the SDL window that will be available to draw to. */
/*  renderer is the SDL renderer that will be available to draw with. */
/*  texture is the SDL texture that will be available to draw on. */
/*  font is the TTF font that will be loaded */
/*  caption will be used as the window caption. */
/*  x_res is the x resolution of the screen */
/*  y_res is the y resolution of the screen */
/*  flags are flags for window creation */
/*  returns 0 if successfull, else an error code indicating where the error occurred */
int sdl_set_up(SDL_Window **window, SDL_Renderer **renderer, SDL_Texture **texture, TTF_Font **font, const std::string &caption, const int x_res, const int y_res, const int flags)
{
    /* Initialise SDL and check error state */
    const int sdl_status = sdl_set_up(window, renderer, texture, caption, x_res, y_res, flags);
    if (sdl_status)
    {
        return sdl_status;
    }

    /* Initialise text rendering */
    if (TTF_Init() != 0)
    {
        std::cout << "Error: SDL TTF init failed with " << TTF_GetError() << std::endl;
        sdl_clean_up(*window, *renderer, *texture, nullptr);
        return 5;
    }
    
    (*font) = TTF_OpenFont("FreeSans.ttf", 12);
    if ((*font) == nullptr)
    {
        std::cout << "Error: Failed to open TTF font with " << TTF_GetError() << std::endl;
        TTF_Quit();
        sdl_clean_up(*window, *renderer, *texture, nullptr);
        return 6;
    }

    return 0;
}


/* Function to draw to the screen. */
/* renderer is an SDL renderer to draw with. */
/* texture is an SDL texture to put on the screen. */
int draw_screen(SDL_Renderer *renderer, SDL_Texture *texture)
{
    /* Write out the data */
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
    
    return 0;
}


/* Function to draw to the screen. */
/* renderer is an SDL renderer to draw with. */
/* texture is an SDL texture to draw on. */
/* screen_data is the picture data to write to the screen. */
int draw_screen(SDL_Renderer *renderer, SDL_Texture *texture, const unsigned char *const screen_data)
{
    /* Get texture width */
    int w, h;
    SDL_QueryTexture(texture, nullptr, nullptr, &w, &h);

    /* Write out the data */
    SDL_UpdateTexture(texture, nullptr, screen_data, w * 3);
    SDL_RenderClear(renderer);
    SDL_RenderCopyEx(renderer, texture, nullptr, nullptr, 0, nullptr, SDL_FLIP_VERTICAL);
    SDL_RenderPresent(renderer);
    
    return 0;
}


/* Function to draw to the screen. */
/* window is an SDL window to draw in. */
/* renderer is an SDL renderer to draw with. */
/* texture is an SDL texture to draw on. */
/* screen_data is the picture data to write to the screen. */
int draw_screen(SDL_Window *window, SDL_Renderer *renderer, SDL_Texture *texture, TTF_Font *font, const std::string &text, const unsigned char *const screen_data)
{
    /* Write to the screen */
    draw_screen(renderer, texture, screen_data);

    const SDL_Color text_colour = { 255, 255, 255 };
    SDL_Surface *sdl_text = TTF_RenderText_Solid(font, text.c_str(), text_colour);
    if (sdl_text == nullptr)
    {
        std::cout << "Error: Failed to set text with " << TTF_GetError() << std::endl;
        sdl_clean_up(window, renderer, texture, font);
        return 7;
    }

    int texW = 0;
    int texH = 0;
    SDL_Texture *ttf_texture = SDL_CreateTextureFromSurface(renderer, sdl_text);
    SDL_QueryTexture(ttf_texture, nullptr, nullptr, &texW, &texH);
    SDL_Rect dstrect = { 0, 0, texW, texH };
    
    /* Add the text */
    SDL_RenderCopy(renderer, ttf_texture, nullptr, &dstrect);
    SDL_RenderPresent(renderer);

    /* Clean up the text */
    SDL_FreeSurface(sdl_text);
    SDL_DestroyTexture(ttf_texture);
    sdl_text = nullptr;
    
    return 0;
}
