/*
 * Elevator-Emulator.c
 *
 * Main file
 *
 * Authors: Peter Sutton, Ahmed Baig
 * Modified by Alex Holdcroft
 */ 

/* Definitions */

#define F_CPU 8000000L

/* External Library Includes */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdbool.h>

/* Internal Library Includes */

#include "display.h"
#include "ledmatrix.h"
#include "buttons.h"
#include "serialio.h"
#include "terminalio.h"
#include "timer0.h"

/* Data Structures */

typedef enum {UNDEF_FLOOR = -1, FLOOR_0=0, FLOOR_1=4, FLOOR_2=8, FLOOR_3=12} ElevatorFloor;

/* Global Variables */
uint32_t time_since_move;
ElevatorFloor current_position;
ElevatorFloor destination;
uint8_t direction_change;
uint8_t old_direction_change;
ElevatorFloor old_floor;
ElevatorFloor current_floor; // Stores the last floor the elevator visited
bool traveller_active; // True if there is a traveller waiting or being moved.
bool traveller_onboard; // True if a traveller is on the elevator.
ElevatorFloor traveller_floor; // Stores the floor the traveller waits on.
uint8_t floors_w_traveller = 0; // Counts the total number of floors travelled with a traveller
uint8_t floors_no_traveller = 0; // Counts the total number of floors travelled without a traveller
volatile uint8_t digit = 0; // The CC value to be set
volatile bool floor_just_reached; // True if we just reached a traveller floor and should execute the door animation
volatile uint8_t animation_phase = 0;   // Records which stage of the LED door animation we are in
volatile uint16_t animation_timer = 0; // Used to count the progress of the LED door animation
volatile bool elevator_door_open = false;
volatile bool button_just_pushed; //True if the placing traveller sound should be played
volatile uint8_t button_timer; //Counts how long it has been after the button push

/* Internal Function Declarations */

void initialise_hardware(void);
void start_screen(void);
void start_elevator_emulator(void);
void handle_inputs(uint8_t);
void draw_elevator(void);
void draw_floors(void);
void display_information(void);
void handle_displays(void);
void handle_seven_seg(uint8_t);
void start_3kHz_sound(void);
void start_500Hz_sound(void);
void stop_sound(void);

/* Main */

int main(void) {
	// Setup hardware and call backs. This will turn on 
	// interrupts.
	initialise_hardware();
	
	// Show the splash screen message. Returns when display is complete
	start_screen();
	
	// Start elevator controller software
	start_elevator_emulator();
}

/* Internal Function Definitions */

/**
 * @brief All hardware initialisation occurs here
 * @arg none
 * @retval none
*/
void initialise_hardware(void) {
	
	ledmatrix_setup();
	init_button_interrupts();
	// Setup serial port for 19200 baud communication with no echo
	// of incoming characters
	init_serial_stdio(19200,0);
	
	init_timer0();

	// Set Pin C7, C6, and C5 as inputs
	DDRC &= ~((1 << PC7)|(1 << PC6)|(1 << PC5));

	// Set Pin C4 as an output, to control the CC
	DDRC |= (1 << PC4)|(1 << PC3)|(1 << PC2)|(1 << PC1)|(1 << PC0);

	// Set port A pins to be outputs, for the seven segment display
    DDRA = 0xFF;

	/* Initialise timer/counter 1 so that it reaches the output compare
	** register value every 1 millisecond, then resets. We need to divide
	** the 8MHz value by 8, which will give 1000 ticks per ms.
	*/
	OCR1A = 499;
	TCCR1A = 0; // Normal operation, no PWM
	TCCR1B = (0 << WGM13) | (1 << WGM12) // Two most significant WGM bits
	| (0 << CS12) | (1 << CS11) | (0 <<CS10); // Divide clock by 8

	// Enable timer/counter1 Output Compare A Match
	TIMSK1 = (1 << OCIE1A);

	// Clear the flag value
	TIFR1 = (1 << OCF1A);

	// Configure OC2A, with prescalar 64, Fast PWM mode
	TCCR2A = (1 << COM2B1)|(1 << WGM21)|(1 << WGM20);
	TCCR2B = (1 << WGM22)|(1 << CS22);

	// Turn on global interrupts
	sei();
}

/**
 * @brief Displays the "EC" start screen with elevator symbol
 * @arg none
 * @retval none
*/
void start_screen(void) {
	// Clear terminal screen and output a message
	clear_terminal();
	move_terminal_cursor(10,10);
	printf_P(PSTR("Elevator Controller"));
	move_terminal_cursor(10,12);
	printf_P(PSTR("CSSE2010/7201 project by Alexandra Holdcroft, 48926782"));
	
	// Show start screen
	start_display();
	
	// Animation variables
	uint32_t doors_frame_time = 0;
	uint32_t interval_delay = 150;
	uint8_t frame = 0;
	uint8_t doors_opening_closing = 1; // 1 => opening, 0 => closing
	
	// Wait until a button is pressed, or 's' is pressed on the terminal
	while(1) {
		
		// Don't worry about this if/else tree. Its purely for animating
		// the elevator doors on the start screen
		if (get_current_time() - doors_frame_time  > interval_delay) {
			start_display_animation(frame);
			doors_frame_time   = get_current_time(); // Reset delay until next movement update
			if (doors_opening_closing) {
				interval_delay = 150;
				frame++;
				if (frame == 1) interval_delay = 2000;
				if (frame == 3) doors_opening_closing = 0;
			} else {
				interval_delay = 150;
				frame--;
				if (frame == 2) interval_delay = 500;
				if (frame == 0) doors_opening_closing = 1;
			}
		}
	
		// First check for if a 's' is pressed
		// There are two steps to this
		// 1) collect any serial input (if available)
		// 2) check if the input is equal to the character 's'
		char serial_input = -1;
		if (serial_input_available()) {
			serial_input = fgetc(stdin);
		}
		// If the serial input is 's', then exit the start screen
		if (serial_input == 's' || serial_input == 'S') {
			break;
		}
		// Next check for any button presses
		int8_t btn = button_pushed();
		if (btn != NO_BUTTON_PUSHED) {
			break;
		}
	}
}

/**
 * @brief Initialises LED matrix and then starts infinite loop handling elevator
 * @arg none
 * @retval none
*/
void start_elevator_emulator(void) {
	
	// Clear the serial terminal
	clear_terminal();
	hide_cursor();
	
	// Initialise Display
	initialise_display();
	
	// Clear a button push or serial input if any are waiting
	// (The cast to void means the return value is ignored.)
	(void)button_pushed();
	clear_serial_input_buffer();

	// Initialise local variables
	time_since_move = get_current_time();
	
	// Draw the floors and elevator
	draw_elevator();
	draw_floors();

	// Display the initial information
	display_information();
	
	current_position = FLOOR_0;
	destination = FLOOR_0;
	direction_change = 0;
	old_direction_change = 0;
	old_floor = FLOOR_0;
	current_floor = FLOOR_0;
	traveller_active = false;

	// Set the LEDs with the initial value (doors should be closed)
	PORTC |= (1 << PC2)|(1 << PC1); 
	PORTC &= ~((1 << PC0)|(1 << PC3));
	
	while(true) {
		
		uint16_t speed;
		if (PINC & (1 << PC7)) {
			speed = 300;
		} else {
			speed = 125;
		}

		// Only update the elevator every 125/300ms, depending on the speed.
		if (get_current_time() - time_since_move > speed && !floor_just_reached) {	
			
			// Adjust the elevator based on where it needs to go
			if (destination - current_position > 0) { // Move up
				current_position++;
			} else if (destination - current_position < 0) { // Move down
				current_position--;
			}
			
			// As we have potentially changed the elevator position, lets redraw it
			draw_elevator();
			
			time_since_move = get_current_time(); // Reset delay until next movement update
		}
		if (!floor_just_reached) {
			uint8_t switch_value = PINC & ((1<<PC6)|(1<<PC5)); // Reads input from switches 0 and 1
			uint8_t floor_choice = switch_value >> 5; // Gives 0, 1, 2 or 3 depending on the switch values

			// Handle any button or key inputs
			handle_inputs(floor_choice);

			// Upon picking up traveller, set the new destination (must be done before the display is handled)
			if (traveller_active && traveller_floor == current_position) {
				destination = (ElevatorFloor)(floor_choice * 4);
			}

			// Handle the display updates:
			handle_displays();

			// Tell the animation to begin when reaching the traveller floor
			if (traveller_active && traveller_floor == current_position) {
				floor_just_reached = true;
			}

			// Drop off the traveller (begin the animation and deactivate them)
			if (traveller_onboard && current_position == destination) {
				floor_just_reached = true;
				traveller_onboard = false;
				traveller_active = false;
			}
		}
		// Take the traveller inside when the doors open
		if (traveller_active && traveller_floor == current_position && elevator_door_open) {
			update_square_colour(5, traveller_floor + 1, EMPTY_SQUARE);
			traveller_onboard = true;
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	// Handle the seven segment display every 0.5ms (every interrupt)
		handle_seven_seg(digit);   // Update SSD
		digit = 1 - digit;         // Toggle between left and right

	// Every 400ms, we want to handle the door display
	if (floor_just_reached) {
		animation_timer ++;
	}
	if (button_just_pushed) {
		button_timer ++;
	}
	if (button_timer == 1) {
		start_3kHz_sound();
	}
	if (button_timer >= 100) {
		button_timer = 0;
		button_just_pushed = false;
		stop_sound();
	}
	switch (animation_phase) {
		case 0: 
			// doors should remain the same (closed) for the first 400ms, no need to update
			if (animation_timer >= 800) {
				animation_phase = 1;
				animation_timer = 0;
			}
			break;
		case 1: // open the doors and play the picking up/ dropping off sound
			if (animation_timer == 1) {
				PORTC |= (1 << PC0)|(1 << PC3); 
				PORTC &= ~((1 << PC1)|(1 << PC2));
				elevator_door_open = true;
				start_500Hz_sound();
			}
			if (animation_timer == 100) { // stop the drop off sound after 50ms
				stop_sound();
			}
			if (animation_timer >= 800) {
				animation_phase = 2;
				animation_timer = 0;
			}
			break;
		case 2:
			if (animation_timer == 1) {
				PORTC |= (1 << PC1)|(1 << PC2); 
				PORTC &= ~((1 << PC0)|(1 << PC3));
				elevator_door_open = false;
			}
			if (animation_timer >= 800) {
				animation_phase = 0;
				animation_timer = 0;
				floor_just_reached = false;
			}
			break;
	}
}


/**
 * @brief Draws 4 lines of "FLOOR" coloured pixels
 * @arg none
 * @retval none
*/
void draw_floors(void) {
	for (uint8_t i = 0; i < WIDTH; i++) {
		update_square_colour(i, FLOOR_0, FLOOR);
		update_square_colour(i, FLOOR_1, FLOOR);
		update_square_colour(i, FLOOR_2, FLOOR);
		update_square_colour(i, FLOOR_3, FLOOR);
	}
}

/**
 * @brief Draws the elevator at the current_position
 * @arg none
 * @retval none
*/
void draw_elevator(void) {
	
	// Store where it used to be with old_position
	static uint8_t old_position; // static variables maintain their value, every time the function is called
	
	int8_t y = 0; // Height position to draw elevator (i.e. y axis)
	
	// Clear where the elevator was
	if (old_position > current_position) { // Elevator going down - clear above
		y = old_position + 3;
		} else if (old_position < current_position) { // Elevator going up - clear below
		y = old_position + 1;
	}
	if (y % 4 != 0) { // Do not draw over the floor's LEDs
		update_square_colour(1, y, EMPTY_SQUARE);
		update_square_colour(2, y, EMPTY_SQUARE);
	}
	old_position = current_position;
	
	// Draw a 2x3 block representing the elevator
	for (uint8_t i = 1; i <= 3; i++) { // 3 is the height of the elevator sprite on the LED matrix
		y = current_position + i; // Adds current floor position to i=1->3 to draw elevator as 3-high block
		if (y % 4 != 0) { // Do not draw on the floor
			update_square_colour(1, y, ELEVATOR);
			update_square_colour(2, y, ELEVATOR); // Elevator is 2 LEDs wide so draw twice
		}
	}
}

/**
 * @brief Reads btn values and serial input and adds a traveller as appropriate
 * @arg none
 * @retval none
*/
void handle_inputs(uint8_t floor_choice) {
	
	// We need to check if any button has been pushed
	uint8_t btn = button_pushed();

	// Additionally check if any serial input has been pressed
	char serial_input = -1;
	if (serial_input_available()) {
		serial_input = fgetc(stdin);
	}
	
	// Check if any elevator destination has been specified, either by buttons or serial input
	ElevatorFloor floors[] = {FLOOR_0, FLOOR_1, FLOOR_2, FLOOR_3};
	uint16_t colours[] = {TRAVELLER_TO_0, TRAVELLER_TO_1, TRAVELLER_TO_2, TRAVELLER_TO_3};

	// i is the floor the traveller is being placed
	// floor choice is the floor the traveller wants to go
	for (int i = 0; i < 4; i++) {
		if (btn == i || serial_input == '0' + i) {
			if (!traveller_active && i != floor_choice) {
				destination = floors[i];
				update_square_colour(5, 4*i + 1, colours[floor_choice]);
				traveller_active = true;
				traveller_floor = destination;
				button_just_pushed = true;
			}
		}
	}
}

void handle_displays(void) {
	bool floor_changed;
	
	// Check the direction
	direction_change = destination - current_position;

	// Update the current floor
	if (current_position == FLOOR_0 || current_position == FLOOR_1 || current_position == FLOOR_2 
		|| current_position == FLOOR_3) {
		current_floor = current_position;
	}

	// Check whether the elevator has changed_floors
	floor_changed = old_floor != current_floor;

	// If the floor has changed, update the floors travelled with or without a traveller
	if (floor_changed) {
		if (traveller_onboard) {
			floors_w_traveller += 1;
		}
		else {
			floors_no_traveller += 1;
		}
	}

	// If the direction has changed or the floor has changed, update the displays
	if (direction_change != old_direction_change || floor_changed) {
		display_information();
	}

	old_direction_change = direction_change;
	old_floor = current_floor;
}

/// @brief handles the seven segment display
/// @param digit the value determining whether the right or left display is shown 
void handle_seven_seg(uint8_t digit) {
	// Seven segment display values, for the floor number, corresponding to '0', '1', '2' and '3'.
	uint8_t seven_seg[4] = {63,6,91,79};

	/* Output the current digit */
	if(digit == 0) { // show right display
		PORTC &= ~(1 << 4);  // C4 = 0, enable right SSD
		// First update the current floor to the current position whenever the elevator is on a floor (not between).
		if (current_position == FLOOR_0 || current_position == FLOOR_1 || current_position == FLOOR_2 
			|| current_position == FLOOR_3) {
			PORTA = seven_seg[current_floor/4];
		} else {
			PORTA = seven_seg[current_floor/4]| 0b10000000;
		}
	} else { // show the left display
		PORTC |= (1 << 4);   // C4 = 1, enable left SSD

		if (floor_just_reached) {
			PORTA = 0b01000000; // Segment G for stationary when floor just reached
		}
		else {
			if (destination - current_position > 0) { // Moving up
				PORTA = 0b00000001; // Segment A
			} else if (destination - current_position < 0) { // Moving down
				PORTA = 0b00001000; // Segment D
			} else if (destination == current_position) { // Stationary
				PORTA = 0b01000000; // Segment G
			}
		}
	}
}

void display_information(void) {
	// First handle the floor level updates:
	// Clear terminal screen
	move_terminal_cursor(10,10);

	// Print the required output, depending on the floor.

	printf_P(PSTR("Current floor: %d"), current_floor/4);

	clear_to_end_of_line();

	// Next handle the elevator direction display: 
	move_terminal_cursor(10,12);

	if (floor_just_reached) {
		printf_P(PSTR("Direction of travel: Stationary")); // always stationary while dropping off/ picking up
	}
	else {
		if (destination - current_position > 0) { // Moving up
			printf_P(PSTR("Direction of travel: Up"));
		} else if (destination - current_position < 0) { // Moving down
			printf_P(PSTR("Direction of travel: Down"));
		} else if (destination == current_position) { // Stationary
			printf_P(PSTR("Direction of travel: Stationary"));
		}
	}
	clear_to_end_of_line();

	// Handle displaying the floors moved with and without a traveller
	move_terminal_cursor(10,14);
	printf_P(PSTR("Number of floors moved with traveller: %d"), floors_w_traveller);
	clear_to_end_of_line();
	move_terminal_cursor(10,15);
	printf_P(PSTR("Number of floors moved without traveller: %d"), floors_no_traveller);
	clear_to_end_of_line();
}

void start_3kHz_sound(void) {
	// Set Pin D6 as an output, to control the piezo buzzer.
	DDRD |= (1 << PD6);
	OCR2A = 41;
	OCR2B = OCR2A/2; // divide by 2 to get 50% duty cycle
}

void start_500Hz_sound(void) {
	// Set Pin D6 as an output, to control the piezo buzzer.
	DDRD |= (1 << PD6);
	OCR2A = 249;
	OCR2B = OCR2A/2; // divide by 2 to get 50% duty cycle
}

void stop_sound(void) {
	// Turn pin D6 off to fully silence it
	DDRD &= ~(1<<PD6);
	OCR2A = 0;
	OCR2B = 0; // gives 0% duty cycle
}