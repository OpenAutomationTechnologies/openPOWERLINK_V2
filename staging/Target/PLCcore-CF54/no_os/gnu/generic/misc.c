/*
 * misc.c
 * 
 * This is a collection of several routines from gzip-1.0.3 
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 * puts by Nick Holloway 1993, better puts by Martin Mares 1995
 * High loaded stuff by Hans Lermen & Werner Almesberger, Feb. 1996
 *
 * JUN/99 -- hacked to work with uClinux/Coldfire (gerg@moreton.com.au)
 */

//#include "prototypes.h"

#define	NULL		0

typedef unsigned int	size_t;

/*
 * gzip declarations
 */

#define OF(args)  args
#define STATIC static

#ifdef BOOTDEBUG
#define	puts		console_nextPutAll
static void		error(char *m);
#else
#define	puts(x)
#define	error(x)	recover()
#endif

void* memset(void* s, int c, size_t n);
void* memcpy(void* __dest, const void* __src, size_t __n);

#define memzero(s, n)     memset ((s), 0, (n))


typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

#define WSIZE 0x8000		/* Window size must be at least 32k, */
				/* and a power of two */

static uch *inbuf;	     /* input buffer */
static uch window[WSIZE];    /* Sliding window buffer */

static unsigned insize = 0;  /* valid bytes in inbuf */
static unsigned inptr = 0;   /* index of next byte to be processed in inbuf */
static unsigned outcnt = 0;  /* bytes in output buffer */

/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define CONTINUATION 0x02 /* bit 1 set: continuation of multi-part gzip file */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define ENCRYPTED    0x20 /* bit 5 set: file is encrypted */
#define RESERVED     0xC0 /* bit 6,7:   reserved */

#define get_byte()  (inptr < insize ? inbuf[inptr++] : fill_inbuf())
		
/* Diagnostic functions */
#ifdef DEBUG
#  define Assert(cond,msg) {if(!(cond)) error(msg);}
#  define Trace(x) fprintf x
#  define Tracev(x) {if (verbose) fprintf x ;}
#  define Tracevv(x) {if (verbose>1) fprintf x ;}
#  define Tracec(c,x) {if (verbose && (c)) fprintf x ;}
#  define Tracecv(c,x) {if (verbose>1 && (c)) fprintf x ;}
#else
#  define Assert(cond,msg)
#  define Trace(x)
#  define Tracev(x)
#  define Tracevv(x)
#  define Tracec(c,x)
#  define Tracecv(c,x)
#endif

static int  fill_inbuf(void);
static void flush_window(void);
static void gzip_mark(void **);
static void gzip_release(void **);
static void *malloc(int size);
static void free(void *where);
static void gzip_mark(void **);
static void gzip_release(void **);
  
static int input_len;
static uch *input_data;
static uch *output_data;
static unsigned long output_ptr = 0;
static long bytes_out = 0;
  
extern unsigned int _ebss;
static unsigned long free_mem_ptr = (long) &_ebss;
static unsigned long free_mem_end_ptr = ((long) &_ebss) + (512 * 1024);

/* ========================================================================= */
 
#include "inflate.c"

/* ========================================================================= */

static void *malloc(int size)
{
	void *p;

	if (size <0) error("Malloc error\n");
	if (free_mem_ptr <= 0) error("Memory error\n");

	free_mem_ptr = (free_mem_ptr + 3) & ~3;	/* Align */

	p = (void *)free_mem_ptr;
	free_mem_ptr += size;

	if (free_mem_ptr >= free_mem_end_ptr)
		error("\nOut of memory\n");

	return p;
}

static void free(void *where)
{	/* Don't care */
}

static void gzip_mark(void **ptr)
{
	*ptr = (void *) free_mem_ptr;
}

static void gzip_release(void **ptr)
{
	free_mem_ptr = (long) *ptr;
}

void* memset(void* s, int c, size_t n)
{
	int i;
	volatile unsigned char *ss = (volatile unsigned char *) s;

	for (i=0;i<n;i++) ss[i] = c;
}

void* memcpy(void* __dest, const void* __src, size_t __n)
{
	int i;
	volatile unsigned char *d = (volatile unsigned char *) __dest;
	volatile unsigned char *s = (volatile unsigned char *) __src;

	for (i=0;i<__n;i++) d[i] = s[i];
}

/* ===========================================================================
 * Fill the input buffer. This is called only when the buffer is empty
 * and at least one byte is really needed.
 */
static int fill_inbuf()
{
	if (insize != 0) {
		error("ran out of input data\n");
	}

	inbuf = input_data;
	insize = input_len;
	inptr = 1;
	return inbuf[0];
}

/* ===========================================================================
 * Write the output window window[0..outcnt-1] and update crc and bytes_out.
 * (Used for the decompressed data only.)
 */
static void flush_window_low()
{
    ulg c = crc;         /* temporary variable */
    unsigned n;
    uch *in, *out, ch;
    
    in = window;
    out = &output_data[output_ptr]; 
    for (n = 0; n < outcnt; n++) {
	    ch = *out++ = *in++;
	    c = crc_32_tab[((int)c ^ ch) & 0xff] ^ (c >> 8);
    }
    crc = c;
    bytes_out += (ulg)outcnt;
    output_ptr += (ulg)outcnt;
    outcnt = 0;
}

static void flush_window()
{
	flush_window_low();
}

#ifdef BOOTDEBUG
static void error(char *x)
{
	puts("\n\n");
	puts(x);
	puts("\n\n -- System halted");

	while(1);	/* Halt */
}
#endif

int decompress_image(void *src, void *dst, unsigned int len)
{
	int result_size;
	input_data = (uch *) src;
	input_len = len;
	output_data = (uch *) dst;

	makecrc();
	puts("Uncompressing...");
	result_size = gunzip();
	puts("done.\n");
	return(result_size);
}
