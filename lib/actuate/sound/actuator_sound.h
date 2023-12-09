/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CEREBRI_ACTUATE_SOUND_ACTUATOR_SOUND_H
#define CEREBRI_ACTUATE_SOUND_ACTUATOR_SOUND_H

#define thrirtysecond 19
#define sixteenth 38
#define eigth 75
#define quarter 150
#define half 300
#define whole 600

#define Gb3 185
#define Eb3 208
#define Bb3 233
#define B3 247
#define C4 262
#define Db4 277
#define D4 294
#define Eb4 311
#define E4 330
#define F4 349
#define Gb4 370
#define G4 392
#define Ab4 415
#define A4 440
#define Bb4 466
#define B4 494
#define C5 523
#define Db5 554
#define D5 587
#define Eb5 622
#define E5 659
#define F5 698
#define Gb5 740
#define G5 784
#define Ab5 831
#define A5 880
#define Bb5 932
#define B5 988
#define C6 1046
#define Db6 1109
#define D6 1175
#define Eb6 1245
#define E6 1319
#define F6 1397
#define Gb6 1480
#define G6 1568
#define Ab6 1661
#define A6 1760
#define Bb6 1865
#define B6 1976
#define REST 1

struct tones_t {
    int note;
    int duration;
};

struct tones_t airy_start_tone[] = {
    { .note = B4, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B4, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B4, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = B4, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = Gb5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = Gb5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = Gb5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = Gb5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = A5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = A4, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = Gb5, .duration = quarter },
    { .note = REST, .duration = whole },
    { .note = REST, .duration = whole },
};

/* Manual Mode Morse code 1*/
struct tones_t manual_mode_tone[] = {
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
};

/* Auto Mode Morse code 2*/
struct tones_t auto_mode_tone[] = {
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
};

/* CMD_VEL Mode Morse code 3*/
struct tones_t cmd_vel_mode_tone[] = {
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
};

/* CAL Mode Morse code 4*/
struct tones_t cal_mode_tone[] = {
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = quarter },
    { .note = REST, .duration = thrirtysecond },
};

/* Armed Morse code A all high*/
struct tones_t armed_tone[] = {
    { .note = B5, .duration = half },
    { .note = REST, .duration = thrirtysecond },
    { .note = D5, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
};

/* Disarmed Morse code A high low*/
struct tones_t disarmed_tone[] = {
    { .note = B5, .duration = half },
    { .note = REST, .duration = thrirtysecond },
    { .note = D4, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
};

/* Safety OFF Morse code S low mid high */
struct tones_t safety_off_tone[] = {
    { .note = B3, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
    { .note = B4, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
    { .note = B5, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
};

/* Safety ON Morse code S HIGH LOW MID */
struct tones_t safety_on_tone[] = {
    { .note = B5, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
    { .note = B4, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
    { .note = B3, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
};

/* Fuel Tone Morse code C high low high low*/
struct tones_t fuel_tone[] = {
    { .note = C5, .duration = half },
    { .note = REST, .duration = thrirtysecond },
    { .note = B6, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
    { .note = C5, .duration = half },
    { .note = REST, .duration = thrirtysecond },
    { .note = B6, .duration = whole },
    { .note = REST, .duration = thrirtysecond },
};

/* No Joystick Received*/
struct tones_t joy_loss_tone[] = {
    { .note = REST, .duration = thrirtysecond },
    { .note = C6, .duration = eigth },
    { .note = REST, .duration = thrirtysecond },
};

/* Reject Tone */
struct tones_t reject_tone[] = {
    { .note = 110, .duration = half },
    { .note = REST, .duration = thrirtysecond },
    { .note = 1325, .duration = 12 },
    { .note = 1480, .duration = 12 },
    { .note = 1634, .duration = 12 },
    { .note = 1781, .duration = 12 },
    { .note = 1921, .duration = 12 },
    { .note = 2050, .duration = 12 },
    { .note = 2166, .duration = 12 },
    { .note = 2268, .duration = 12 },
    { .note = 2352, .duration = 12 },
    { .note = 2419, .duration = 12 },
    { .note = 2466, .duration = 12 },
    { .note = 2493, .duration = 12 },
    { .note = 2499, .duration = 12 },
    { .note = 2485, .duration = 12 },
    { .note = 2449, .duration = 12 },
    { .note = 2394, .duration = 12 },
    { .note = 2321, .duration = 12 },
    { .note = 2229, .duration = 12 },
    { .note = 2122, .duration = 12 },
    { .note = 2000, .duration = 12 },
    { .note = 1866, .duration = 12 },
    { .note = 1723, .duration = 12 },
    { .note = 1573, .duration = 12 },
    { .note = 1418, .duration = 12 },
    { .note = 1262, .duration = 12 },
    { .note = 1107, .duration = 12 },
    { .note = 955, .duration = 12 },
    { .note = 811, .duration = 12 },
    { .note = 675, .duration = 12 },
    { .note = 551, .duration = 12 },
    { .note = 440, .duration = 12 },
    { .note = 345, .duration = 12 },
    { .note = 268, .duration = 12 },
    { .note = 209, .duration = 12 },
    { .note = 170, .duration = 12 },
    { .note = 151, .duration = 12 },
    { .note = 153, .duration = 12 },
    { .note = 176, .duration = 12 },
    { .note = 219, .duration = 12 },
    { .note = 282, .duration = 12 },
    { .note = 363, .duration = 12 },
    { .note = 461, .duration = 12 },
    { .note = 575, .duration = 12 },
    { .note = 701, .duration = 12 },
    { .note = 839, .duration = 12 },
    { .note = 985, .duration = 12 },
    { .note = 1138, .duration = 12 },
    { .note = 1293, .duration = 12 },
    { .note = 1449, .duration = 12 },
    { .note = 1603, .duration = 12 },
    { .note = 1752, .duration = 12 },
    { .note = 1894, .duration = 12 },
    { .note = 2025, .duration = 12 },
    { .note = 2144, .duration = 12 },
    { .note = 2249, .duration = 12 },
    { .note = 2337, .duration = 12 },
    { .note = 2407, .duration = 12 },
    { .note = 2458, .duration = 12 },
    { .note = 2489, .duration = 12 },
    { .note = 2500, .duration = 12 },
    { .note = REST, .duration = whole },
};

#endif // CEREBRI_ACTUATE_SOUND_ACTUATOR_SOUND_H
/* vi: ts=4 sw=4 et */
