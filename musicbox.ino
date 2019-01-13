/*
 * Music Box Code
 *
 * This code emulates the sound of a mechanical music box using DDS.
 * This code is for an Arduino Nano board
 * Written by: Craig A. Lindley, Fabian Tschopp
 * Last Update: 11/01/2018
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include "Arduino.h"
#include "converted_melodies/music_data.h"
#include <FastLED.h>

// LED output pin
#define LED_PIN                           2

// Analog potentiometer to select the LED palette
#define LED_PALETTE_SELECTION_PIN         1

// Number of LED palettes
#define NUMBER_OF_LED_PALETTES            1

// Number of LEDs
#define NUM_LEDS                          6

// Max. brightness divider
#define BRIGHTNESS_DIV                    4

// Type of attached LED
#define LED_TYPE                      WS2811

// Color order
#define COLOR_ORDER                      GRB

// LEDs
CRGB leds[NUM_LEDS];

// Current LED palette and blending
int ledBrightness = 0;
int ledPaletteIndex = 0;
int ledStateIndex = 0;
CRGBPalette16 ledPalette;
TBlendType    ledBlending;


// PWM output pin
#define OUTPUT_PIN                        11

// Analog potentiometer to select the song
#define SOUND_SELECTION_PIN                0

// Analog potentiometer to control lighting
#define LIGHT_SELECTION_PIN                1

// Waiting time to start a song again
#define REPETITION_DELAY_SEC               3

// Shut board down after X repeats
#define REPETITION_SHUTDOWN               10

// Sound Generator Data
volatile uint16_t m[NUMBER_OF_GENERATORS];
volatile uint32_t phaseAccumulator[NUMBER_OF_GENERATORS];
volatile uint16_t envelopeIndex[NUMBER_OF_GENERATORS];

// Count of ISR calls
volatile int32_t sampleCount;

// Global Program Data
uint16_t noteIndex;
uint16_t tickCount;
int32_t  timeoutCount;
int8_t   nextGenerator;

int songIndex = 0;
int repetitionCount = 0;

// Reset system variables in preparation for playing/replaying the song
void systemReset() {

  // Clear all global data
  noteIndex = 0;
  nextGenerator = 0;
  sampleCount = 0;
  tickCount = 0;
  
  // Clear sound generator data
  for (int i = 0; i < NUMBER_OF_GENERATORS; i++) {
    m[i] = 0;
    phaseAccumulator[i] = 0;
    envelopeIndex[i] = 0;
  }
}

void updateSongIndexFromInput() {
  int newSongIndex = analogRead(SOUND_SELECTION_PIN) / ((1023 - 1) / NUMBER_OF_SONGS + 1);
  if (newSongIndex != songIndex) {
    songIndex = newSongIndex;
    repetitionCount = 0;
    systemReset();
  }
}

void updateLEDPaletteFromInput() {
  int ledInput = analogRead(LED_PALETTE_SELECTION_PIN);
  int newLedBrightness = ((ledInput % ((1024 - 1) / NUMBER_OF_LED_PALETTES + 1)) * NUMBER_OF_LED_PALETTES) / BRIGHTNESS_DIV;
  int newLedPaletteIndex = ledInput / ((1024 - 1) / NUMBER_OF_LED_PALETTES + 1);
  if (newLedPaletteIndex != ledPaletteIndex) {
    ledPaletteIndex = newLedPaletteIndex;

  }
  if (newLedBrightness != ledBrightness) {
    ledBrightness = newLedBrightness;
    FastLED.setBrightness(ledBrightness);
    FastLED.show();
  }
}

void fillLEDsFromPaletteColors() {
 if (ledBrightness > 10) {
   int scale = 30;
   for (int i = 0; i < NUM_LEDS; i++ ) {
      int v = inoise8(i * 3 * scale, ledStateIndex);
      int h = inoise8(i * scale, ledStateIndex);
      h = qsub8(h, 16);
      h = qadd8(h, scale8(h, 39));
      leds[i] = CHSV(h, 255, v);
    }
    FastLED.show();
    ++ledStateIndex;
  }
}

void musicbox_shutdown() {
  // Power the LEDs down
   for (int i = 0; i < NUM_LEDS; i++ ) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.setBrightness(0);
  FastLED.show();
  // Shut down interrupts
  cli();
  // PWR down sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // Allow sleep
  sleep_enable();
  // Put to sleep
  sleep_mode();
}

// Calculate a timeout value sometime in the future
int32_t calculateTimeout(int32_t time) {
  return sampleCount + time;
}

// Return false until the timeout interval has passed
boolean checkTimeout(int32_t timeout) {
  return ((sampleCount - timeout) > 0);
}   

// This ISR is called at the SAMPLE_RATE interval.
ISR(TIMER1_COMPA_vect) {

  int32_t pwmSample = 0;
  uint32_t ph;
  boolean decaying = false;

  // Process all sound generators
  for (uint8_t gen = 0; gen < NUMBER_OF_GENERATORS; gen++) {    

    // Get phase for this sound generator
    ph = phaseAccumulator[gen];

    // Get wave table address
    uint32_t waveTableIndex = (ph >> POT);

    // Update phase with phasor
    phaseAccumulator[gen] += m[gen];

    // Setup looping over sustain portion of wavetable
    // once we have reached end of wavetable data.
    if (phaseAccumulator[gen] >= ((uint32_t) WAVETABLE_LENGTH) << POT) {
      phaseAccumulator[gen] -= (((uint32_t) SUSTAIN_LENGTH) << POT);
      decaying = true;
    }

    // Get the sample -127 .. 127
    int32_t s = (int8_t) pgm_read_byte(waveTableData + waveTableIndex);

    // Get the envelope value 255 .. 0
    uint8_t e = pgm_read_byte(envelopeData + envelopeIndex[gen]);

    // Multiply sample times envelope value and add to sample sum
    pwmSample += s * e;

    // Advance envelope position if not at end of envelope table and in the process of decaying
    if ((e != 0) && decaying) {
      envelopeIndex[gen]++;
    }
  }
  // Convert 24.8 fixed point value to integer
  pwmSample >>= POT + 2;

  // Offset the signed pwm sample and update timer 2
  OCR2A = (uint8_t)(127 + pwmSample);

  // Update count
  sampleCount++;
}

// Prepare for operation
void setup() {

  // Set timer2 output pin as output
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // Power-up safety delay
  delay(3000);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(0);
  FastLED.show();
    
  ledPalette = RainbowColors_p;
  ledBlending = LINEARBLEND;

  cli();

  /*
   * Timer 1 Configuration
   * Set up Timer 1 to generate interrupt at SAMPLE_RATE.
   */
  // Set CTC mode (Clear Timer on Compare Match)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~(_BV(WGM13) | _BV(WGM12))) | _BV(WGM12);
  TCCR1A = (TCCR1A & ~(_BV(WGM11) | _BV(WGM10)));

  // CPU/1 prescaler
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11) | _BV(CS10))) | _BV(CS10);

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  // 16e6 Hz / 22000 Hz = 727.
  // 16 MHz Arduino not fast enough to support values < 700 (e.g. 16e6 Hz / 32000 Hz = 500 not supported)
  // Interrupts will be skipped, and the song will play too slow
  OCR1A = F_CPU / SAMPLE_RATE;

  // Enable interrupt when TCNT1 == OCR1A
  TIMSK1 |= _BV(OCIE1A);

  /*
   * Timer 2 Configuration
   * Set up Timer 2 to do pulse width modulation at frequency of 62,500 Hz
   */
  // Set fast PWM mode
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);

  // Do non-inverting PWM on pin OC2A. Arduino pin 11
  TCCR2A = (TCCR2A & ~(_BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0))) | _BV(COM2A1);

  // No prescaler so PWM frequency is 16000000 / 256 or 62,500 Hz
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21) | _BV(CS20))) | _BV(CS20);

  // Set initial pulse width to zero.
  OCR2A = 0;

  systemReset();

  updateSongIndexFromInput();

  sei();
}

// Loop until reset  
void loop() {

  if (repetitionCount >= REPETITION_SHUTDOWN) {
    // Max. repetition count reached, shut the board down
    musicbox_shutdown();
  }

  // Reset in preparation for playing the tune
  cli();
  systemReset();
  sei();

  // Set the timeout count
  timeoutCount = calculateTimeout(INTERRUPTS_PER_TICK);

  while(true) {

    // Has timeout occurred ?
    if (checkTimeout(timeoutCount) || tickCount == 0) {

      // Timeout occurred, schedule next one
      timeoutCount = calculateTimeout(INTERRUPTS_PER_TICK);

      // Update tick count;
      tickCount++;

      const uint16_t* progDataAddress;
      uint16_t tick;

      // Get ptr to current song
      progDataAddress = getSongDataAddress(songIndex);

      // Get the tick value for the next note  
      tick = pgm_read_word_near(progDataAddress + noteIndex);

      // Is the song over ?
      if (tick == 0) {
        break;
      }

      // Time for next note ?
      if (tickCount >= tick) {
        // Update LED pattern together with each new note
        fillLEDsFromPaletteColors();
        
        // Yes fetch next note
        uint16_t noteNumber;

        // Get the note
        noteNumber = pgm_read_word_near(progDataAddress + noteIndex + 1);

        // Advance note index
        noteIndex += 2;

        // Allocate the next available sound generator for playing a note
        // A sound generator may be reallocated before finishing a
        // note (especially a low frequency note). This may be
        // audible. Less likely with more generators.         
        int generatorIndex = nextGenerator++;

        cli(); 

        // Configure sound generator
        m[generatorIndex] = midiPitchData[noteNumber + MIDI_KEY_OFFSET];
        phaseAccumulator[generatorIndex] = 0;
        envelopeIndex[generatorIndex] = 0;

        sei();

        // Keep index in bounds
        if (nextGenerator >= NUMBER_OF_GENERATORS) {
          nextGenerator = 0;
        }   
      } else {
        // Update current song index and LED palette periodically
        if (tickCount % 100 == 0) {
          updateSongIndexFromInput();
          updateLEDPaletteFromInput();
        }
      }
    }
  }
  repetitionCount += 1;

  // Delay between song repetitions
  // Set the timeout count
  timeoutCount = calculateTimeout((int32_t) REPETITION_DELAY_SEC * SAMPLE_RATE);
  
  // Loop until timeout occurs
  while(true) {
    if (checkTimeout(timeoutCount)) {
      break;
    }
  }
}
