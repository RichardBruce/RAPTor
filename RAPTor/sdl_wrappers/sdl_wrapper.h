#ifndef __SDL_WRAPPER_H__
#define __SDL_WRAPPER_H__

/* Forward declartions */
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;
struct _TTF_Font;
typedef _TTF_Font TTF_Font;

/* Display related constants */
extern const int BPP;
extern const int DEPTH;

/* SDL and TTF clean up functions */
void sdl_clean_up(SDL_Window *const window, SDL_Renderer *const renderer, SDL_Texture *const texture, TTF_Font *const font);

/* SDL and TTF set up functions */
int sdl_set_up(SDL_Window **window, SDL_Renderer **renderer, SDL_Texture **texture, const std::string &caption, const int x_res, const int y_res);
int sdl_set_up(SDL_Window **window, SDL_Renderer **renderer, SDL_Texture **texture, TTF_Font **font, const std::string &caption, const int x_res, const int y_res);

/* Write text and screen_data to screen (the video window) */
int draw_screen(SDL_Renderer *renderer, SDL_Texture *texture);
int draw_screen(SDL_Renderer *renderer, SDL_Texture *texture, const unsigned char *const screen_data);
int draw_screen(SDL_Window *window, SDL_Renderer *renderer, SDL_Texture *texture, TTF_Font *font, const std::string &text, const unsigned char *const screen_data);

#endif /* #ifndef __SDL_WRAPPER_H__ */
