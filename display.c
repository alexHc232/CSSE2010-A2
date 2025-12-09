/*
 * display.c
 *
 * Author: Luke Kamols, Ahmed Baig
 */ 

#include <stdio.h>
#include <avr/pgmspace.h>

#include "display.h"
#include "pixel_colour.h"
#include "ledmatrix.h"

// constant value used to display elevator on launch
static const uint16_t elevator_display[MATRIX_NUM_COLUMNS] = {
	(1<<7)|(1<<6)|(1<<5)|(0<<4)|(0<<3)|(1<<2)|(1<<1)|(1<<0) | (1<<8),
	(0<<7)|(0<<6)|(1<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (1<<8),
	(1<<7)|(1<<6)|(1<<5)|(0<<4)|(0<<3)|(1<<2)|(1<<1)|(1<<0) | (1<<8),
	(0<<7)|(0<<6)|(1<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (1<<8),
	(1<<7)|(1<<6)|(1<<5)|(0<<4)|(0<<3)|(1<<2)|(1<<1)|(1<<0) | (1<<8),
	(0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0) | (1<<8),
	
	(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0) | (0<<8),
	(1<<7)|(1<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(1<<1)|(1<<0) | (0<<8),
	(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(1<<0) | (0<<8),
	(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0) | (0<<8)
	};

void initialise_display(void) {
	// clear the LED matrix
	ledmatrix_clear();
}

void start_display(void) {
	PixelColour colour;
	MatrixColumn column_colour_data;
	uint16_t col_data;
		
	ledmatrix_clear(); // start by clearing the LED matrix
	for (uint8_t col = 0; col < MATRIX_NUM_COLUMNS; col++) {
		col_data = elevator_display[col];
		// using the 9th bit as the colour determining bit, 1 is red, 0 is green
		if (col_data & 0x0100) {
			colour = COLOUR_RED;
		} else {
			colour = COLOUR_GREEN;
		}
		// go through the bottom 8 bits and set any to be the correct colour
		for(uint8_t i = 0; i < 8; i++) {
			if(col_data & 0x01) {
				column_colour_data[i] = colour;
				} else {
				column_colour_data[i] = 0;
			}
			col_data >>= 1;
		}
		//column_colour_data[0] = 0;
		ledmatrix_update_column(col, column_colour_data);
	}
	
}

void start_display_animation(uint8_t frame) {
	MatrixColumn column_colour_data;
	uint16_t col_data;
		
	// Doors
	for (uint8_t col = 9; col < MATRIX_NUM_COLUMNS-1; col++) {
		col_data = (1<<7)|(1<<(3-frame))|(1<<(4+frame))|(1<<0);
		// go through the bottom 8 bits and set any to be the correct colour
		for(uint8_t i = 0; i < 8; i++) {
			if(col_data & 0x01) {
				column_colour_data[i] = COLOUR_GREEN;
				} else {
				column_colour_data[i] = 0;
			}
			col_data >>= 1;
		}
		ledmatrix_update_column(col, column_colour_data);
	}
}


/*
 * This function treats x and y coordinates in some unintuitive ways.
  * You are not expected to follow all the logic of this.
 */
void update_square_colour(uint8_t x, uint8_t y, uint8_t object) {
	
	// first check that this is a square within the game field
	// if outside the game field, don't update anything
	if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
		return;
	}
	
	// determine which colour corresponds to this object
	PixelColour colour;

	if (object == ELEVATOR) {
		colour = MATRIX_COLOUR_ELEVATOR;
	} else if (object == FLOOR) {
		colour = MATRIX_COLOUR_FLOOR;
	} else if (object == TRAVELLER_TO_0) {
		colour = MATRIX_COLOUR_TRAVELLER_0;
	} else if (object == TRAVELLER_TO_1) {
	colour = MATRIX_COLOUR_TRAVELLER_1;
	} else if (object == TRAVELLER_TO_2) {
	colour = MATRIX_COLOUR_TRAVELLER_2;
	} else if (object == TRAVELLER_TO_3) {
	colour = MATRIX_COLOUR_TRAVELLER_3;
	} else {
		// anything unexpected (or empty) will be black
		colour = MATRIX_COLOUR_EMPTY;
	}

	// update the pixel at the given location with this colour
	/* x and y are swapped here because the ledmatrix.c code
	 * treats the matrix as being horizontal, while the elevator
	 * controller treats the matrix vertically. We also want x
	 * to be interpreted as from bottom to top, not top to bottom.
	 */
	ledmatrix_update_pixel(15 - y, x, colour); 
}