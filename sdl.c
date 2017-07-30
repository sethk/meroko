/*
 * sdl.c
 * $Id$
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <SDL/SDL.h>

#if defined(FB_WIDTH) && defined(FB_HEIGHT)
#define VIDEO_WIDTH FB_WIDTH
#define VIDEO_HEIGHT FB_HEIGHT
#else
#define VIDEO_HEIGHT 1024
#define VIDEO_WIDTH 1280
#endif

static SDL_Surface *screen;
static int video_width = VIDEO_WIDTH;
static int video_height = VIDEO_HEIGHT;

#define MAX_BITMAP_OFFSET	((VIDEO_WIDTH * VIDEO_HEIGHT) / 32)
static unsigned int bitmap[MAX_BITMAP_OFFSET];

#define COLOR_WHITE	0xff
#define COLOR_BLACK	0

typedef struct DisplayState {
    unsigned char *data;
    int linesize;
    int depth;
    int width;
    int height;
} DisplayState;

static DisplayState display_state;
static DisplayState *ds = &display_state;

#define MOUSE_EVENT_LBUTTON 1
#define MOUSE_EVENT_MBUTTON 4
#define MOUSE_EVENT_RBUTTON 2

extern void kbd_handle_char(int sc, int sym, int updown, int shifted);

void
sdl_video_read(int offset, unsigned int *pv)
{
	*pv = 0;

	/* the real h/w has memory for 768x1024 */
	if (offset < MAX_BITMAP_OFFSET)
		*pv = bitmap[offset];
}

static void sdl_process_key(SDL_KeyboardEvent *ev, int updown)
{
	int mod_state, extra;

	mod_state = SDL_GetModState();

	extra = 0;
#if 0
	if (mod_state & (KMOD_LMETA | KMOD_RMETA))
		extra |= 3 << 12;

	if (mod_state & (KMOD_LALT | KMOD_RALT))
		extra |= 3 << 12;

	if (mod_state & (KMOD_LSHIFT | KMOD_RSHIFT))
		extra |= 3 << 6;

	if (mod_state & (KMOD_LCTRL | KMOD_RCTRL))
		extra |= 3 << 10;
#endif

#if 0
	{
		char b[256];
		sprintf(b, "scancode %x, sym %x, mod %x, unicode %x\n",
		       ev->keysym.scancode,
		       ev->keysym.sym,
		       ev->keysym.mod,
		       ev->keysym.unicode);
		logmsg(b);
	}
#endif

	kbd_handle_char(ev->keysym.scancode, ev->keysym.sym, updown,
			mod_state & KMOD_SHIFT);
}

static void sdl_send_mouse_event(void)
{
  int dx, dy, state;
  extern unsigned char sib_mouse_motion_reg;
  extern unsigned int sib_mouse_x_pos;
  extern unsigned int sib_mouse_y_pos;

	state = SDL_GetRelativeMouseState(&dx, &dy);

	if (state & SDL_BUTTON(SDL_BUTTON_LEFT))
	  sib_mouse_motion_reg |= 1<<6;
	else
	  sib_mouse_motion_reg &= ~(1<<6);

	if (state & SDL_BUTTON(SDL_BUTTON_MIDDLE))
	  sib_mouse_motion_reg |= 1<<5;
	else
	  sib_mouse_motion_reg &= ~(1<<5);

	if (state & SDL_BUTTON(SDL_BUTTON_RIGHT))
	  sib_mouse_motion_reg |= 1<<4;
	else
	  sib_mouse_motion_reg &= ~(1<<4);
	
	sib_mouse_x_pos += dx;
	sib_mouse_y_pos += dy;

#if 0
	logmsgf("Mouse event: x %d (%d), y %d (%d), buttons %04x\n",
		sib_mouse_x_pos, dx,
		sib_mouse_y_pos, dy, sib_mouse_motion_reg);
#endif
#if 0
	sib_mouse_event(x, y, dx, dy, buttons);
#endif
}

static void sdl_update(DisplayState *ds, int x, int y, int w, int h)
{
    SDL_UpdateRect(screen, x, y, w, h);
}

void
sdl_system_shutdown_request(void)
{
	exit(0);
}


static char video_bow_mode = 1;	/* 1 => White on Black, 0 => Black on White */

void
sdl_set_bow_mode(char new_mode)
{
	unsigned char *p;
	int i, j;

	if (!screen)
		return;
	p = screen->pixels;
#if 0
	printf("Setting Black-on-White mode: was %d, setting %d\n",
	       video_bow_mode, new_mode);
#endif
	if (video_bow_mode == new_mode)
		return;

	/* Need to complement it */
	video_bow_mode = new_mode;

	for (i = 0; i < video_width; i++) {
		for (j = 0; j < video_height; j++) {
			*p = ~*p;
			p++;
		}
	}

	SDL_UpdateRect(screen, 0, 0, video_width, video_height);
}


int u_minh = 0x7fffffff, u_maxh = 0, u_minv = 0x7fffffff, u_maxv = 0;

void
accumulate_update(int h, int v, int hs, int vs)
{
	if (h < u_minh) u_minh = h;
	if (h+hs > u_maxh) u_maxh = h+hs;
	if (v < u_minv) u_minv = v;
	if (v+vs > u_maxv) u_maxv = v+vs;
}

void
send_accumulated_updates(void)
{
	int hs, vs;

	hs = u_maxh - u_minh;
	vs = u_maxv - u_minv;
	if (u_minh != 0x7fffffff && u_minv != 0x7fffffff &&
	    u_maxh && u_maxv)
	{
		SDL_UpdateRect(screen, u_minh, u_minv, hs, vs);
	}

	u_minh = 0x7fffffff;
	u_maxh = 0;
	u_minv = 0x7fffffff;
	u_maxv = 0;
}

void
sdl_video_write(int offset, unsigned int bits)
{
	if (screen) {
		unsigned char *ps = screen->pixels;
		int i, h, v;

		if (offset > MAX_BITMAP_OFFSET) {
#if 0
		  fprintf(stderr, "sdl_video_write: "
			  "offset too big (0x%x > 0x%x): 0x%x\n",
			  offset, MAX_BITMAP_OFFSET, bits);
#endif
		  return;
		}

		bitmap[offset] = bits;

		offset *= 32;

		v = offset / video_width;
		h = offset % video_width;

		if (0) printf("v,h %d,%d <- %o (offset %d)\n",
			      v, h, bits, offset);

		for (i = 0; i < 32; i++)
		{
			ps[offset + i] =
				(bits & 1) ? COLOR_WHITE : COLOR_BLACK;
			if (video_bow_mode == 0)
			  ps[offset + i] ^= ~0;
			bits >>= 1;
		}

#if 0
		SDL_UpdateRect(screen, h, v, 32, 1);
#else
		accumulate_update(h, v, 32, 1);
#endif
	}
}

/* hack, but it works */
void
sdl_video_write_byte(int offset, int bytenr, unsigned char bits)
{
	if (screen) {
		unsigned int word = bitmap[offset];
		word &= ~(0xff << 8*bytenr);
		word |= (bits << 8*bytenr);
		sdl_video_write(offset, word);
	}
}

void
sdl_sync(unsigned char *fb, int len)
{
	unsigned char *ps = screen->pixels;
	int offset = fb - ps;
	int h, v;

	v = offset / video_width;
	h = offset % video_width;

#if 0
	SDL_UpdateRect(screen, h, v, len, 1);
#else
	accumulate_update(h, v, len, 1);
#endif
}

void
sdl_refresh(void)
{
	SDL_Event ev1, *ev = &ev1;
	int mod_state;
        extern unsigned int sib_mouse_x_pos;
        extern unsigned int sib_mouse_y_pos;

#if 1
	send_accumulated_updates();
#endif

	while (SDL_PollEvent(ev)) {

		switch (ev->type) {
		case SDL_VIDEOEXPOSE:
			sdl_update(ds, 0, 0, screen->w, screen->h);
			break;

		case SDL_KEYDOWN:
			mod_state = (SDL_GetModState() &
				     (KMOD_LSHIFT | KMOD_LCTRL)) ==
				(KMOD_LSHIFT | KMOD_LCTRL);

			if (mod_state) {
				switch(ev->key.keysym.sym) {
				case SDLK_F1 ... SDLK_F12:
					break;
				default:
					break;
				}
			}

			sdl_process_key(&ev->key, 1);
			break;

		case SDL_KEYUP:
			sdl_process_key(&ev->key, 0);
			break;
		case SDL_QUIT:
			sdl_system_shutdown_request();
			break;
		case SDL_MOUSEMOTION:
			sdl_send_mouse_event();
			break;
		case SDL_MOUSEBUTTONDOWN:
		case SDL_MOUSEBUTTONUP:
		{
			sdl_send_mouse_event();
		}
		break;

//		case SDL_ACTIVEEVENT:
                case SDL_APPMOUSEFOCUS:
                        SDL_GetMouseState(&sib_mouse_x_pos,&sib_mouse_y_pos);
			break;
		default:
			break;
		}
	}
}

static void sdl_resize(DisplayState *ds, int w, int h)
{
    int flags;

    flags = SDL_HWSURFACE|SDL_ASYNCBLIT|SDL_HWACCEL;
//    flags |= SDL_RESIZABLE;
//    flags |= SDL_FULLSCREEN;
//    screen = SDL_SetVideoMode(w, h, 0, flags);
//    screen = SDL_SetVideoMode(w, h, 8, flags);
    screen = SDL_SetVideoMode(w, h, 8, flags);

    if (!screen) {
        fprintf(stderr, "Could not open SDL display\n");
        exit(1);
    }

    ds->data = screen->pixels;
    ds->linesize = screen->pitch;
    ds->depth = screen->format->BitsPerPixel;
    ds->width = w;
    ds->height = h;
}

static void sdl_update_caption(void)
{
    char buf[1024];

    strcpy(buf, "Meroko");
#if 0
    if (cpu_die == 0) {
        strcat(buf, " [Running]");
    } else {
        strcat(buf, " [Stopped]");
    }
#endif
    SDL_WM_SetCaption(buf, "Meroko");
}

static void sdl_cleanup(void) 
{
    SDL_Quit();
}


void
sdl_setup_display(void)
{
//	SDL_Surface *logo;
	unsigned char *p = screen->pixels;
	int i, j;

	for (i = 0; i < video_width; i++) {
		for (j = 0; j < video_height; j++)
			*p++ = COLOR_WHITE;
	}

#if 0
	{
		char *p;
		unsigned char *ps = screen->pixels;
		for (j = 0; j < 116; j++) {
			p = logo_xpm[3+j];
			for (i = 0; i < 432; i++) {
				if (p[i] != '.')
					ps[i] = COLOR_BLACK;
			}
			ps += video_width;
		}
	}
#endif

	SDL_UpdateRect(screen, 0, 0, video_width, video_height);
}

static void sdl_display_init(void)
{
    int flags;

    flags = SDL_INIT_VIDEO | SDL_INIT_NOPARACHUTE;

    if (SDL_Init(flags)) {
        fprintf(stderr, "SDL initialization failed\n");
        exit(1);
    }

    /* NOTE: we still want Ctrl-C to work - undo the SDL redirections*/
    signal(SIGINT, SIG_DFL);
    signal(SIGQUIT, SIG_DFL);

    sdl_resize(ds, video_width, video_height);

    sdl_update_caption();

    SDL_EnableKeyRepeat(250, 50);
//    SDL_EnableUNICODE(1);

    sdl_setup_display();

    SDL_ShowCursor(0);

    atexit(sdl_cleanup);
}

void
display_poll(void)
{
	sdl_refresh();

#if 0
	if (old_run_state != run_ucode_flag) {
		old_run_state = run_ucode_flag;
		sdl_update_caption();
	}
#endif
}


int
sdl_init(unsigned long int *pscreensize, char **pframebuffer, int width, int height)
{
	if ((width > VIDEO_WIDTH) || (height > VIDEO_HEIGHT)) {
		fprintf(stderr, "**** Too large screen requested: "
			"%d x %d (max is %d x %d)\n",
			width, height, VIDEO_WIDTH, VIDEO_HEIGHT);
		return 1;
	}

#if 1
	video_width = width;
	video_height = height;
#endif

printf("XXX wid %d high %d\n", video_width, video_height);

	sdl_display_init();

	*pscreensize= video_width * video_height;
	*pframebuffer = screen->pixels;

	return 0;
}
