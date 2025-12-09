/*
 * display.h
 *
 * Author: Luke Kamols
 */ 

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "pixel_colour.h"

// display dimensions, these match the size of the playing field
#define WIDTH  8
#define HEIGHT 16

// object definitions

#define PLAYER			1
#define FACING			2
#define BREAKABLE	 	3
#define UNBREAKABLE		4
#define DIAMOND			5
#define UNDISCOVERED	6

#define EMPTY_SQUARE    0
#define ELEVATOR		1
#define FLOOR			2
#define TRAVELLER_TO_0	3
#define TRAVELLER_TO_1	4
#define TRAVELLER_TO_2	5
#define TRAVELLER_TO_3	6

// matrix colour definitions

#define MATRIX_COLOUR_EMPTY			COLOUR_BLACK
#define MATRIX_COLOUR_ELEVATOR		COLOUR_RED
#define MATRIX_COLOUR_FLOOR			COLOUR_GREEN
#define MATRIX_COLOUR_TRAVELLER_0	COLOUR_LIGHT_RED
#define MATRIX_COLOUR_TRAVELLER_1	COLOUR_LIGHT_GREEN
#define MATRIX_COLOUR_TRAVELLER_2	COLOUR_LIGHT_YELLOW
#define MATRIX_COLOUR_TRAVELLER_3	COLOUR_LIGHT_ORANGE

/*
 * initialise the display for the playing field
 */
void initialise_display(void);

/*
 * display a start screen
 */
void start_display(void);


/*
 * animates start screen
 */
void start_display_animation(uint8_t frame);

/*
 * updates the colour at square (x, y) to be the colour
 * of the object 'object'
 * 'object' is expected to be EMPTY_SQUARE, PLAYER, FACING, 
 * BREAKABLE, UNBREAKABLE, DIAMOND or UNDISCOVERED
 */
void update_square_colour(uint8_t x, uint8_t y, uint8_t object);

#endif 