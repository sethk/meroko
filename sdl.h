#include <SDL/SDL_main.h>

extern void sdl_video_write(int offset, unsigned int bits);
extern void sdl_video_write_byte(int offset, int bytenr, unsigned char bits);
extern int sdl_init(unsigned long int *pscreensize, char **pframebuffer,
                    int width, int height);
extern void sdl_set_bow_mode(char new_mode);
extern void display_poll(void);

