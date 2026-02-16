#pragma GCC optimize("-Os")

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "clion_compat.h"

/*
 * =========================================================================================
 * FIXED POINT TRIGONOMETRY SUBSYSTEM
 * =========================================================================================
 *
 * THE PROBLEM:
 * ------------
 * The ATtiny85 (and Arduino Uno) has no FPU (Floating Point Unit).
 * Calculating `sin(angle)` using standard `math.h` (`float` or `double`) triggers
 * software emulation. This consumes ~2KB of Flash and takes thousands of cycles
 * per calculation, causing the OLED display to lag visibly.
 *
 * THE SOLUTION: INTEGER LOOK-UP TABLE (LUT)
 * -----------------------------------------
 * We pre-calculate the Sine values for 0 to 90 degrees and store them in Flash (PROGMEM).
 * We use the mathematical symmetry of the unit circle to derive 91-360 degrees.
 *
 * SCALING FACTOR (FIXED POINT):
 * -----------------------------
 * Instead of returning a float 0.0 to 1.0, we return an integer -127 to +127.
 * - Value   0 represents 0.0
 * - Value 127 represents 1.0
 *
 * To use the result in a calculation:
 * Result = (Input * sinInt(angle)) / 127;
 *
 * Or, for faster bit-shifting (Power of 2):
 * If we treat 127 as "almost 128", we can bit-shift >> 7.
 * For the compass rotation, we scale this further to Q8.8 (256 base) via left-shifting.
 * =========================================================================================
 */

/*
 * Sine Look-Up Table (0 to 90 degrees).
 * Range: 0 to 127.
 *
 * Why only 0-90?
 * A sine wave is symmetrical.
 * - Quadrant 1 (0-90):   Values increase 0 -> 1.
 * - Quadrant 2 (91-180): Values decrease 1 -> 0 (Mirror of Q1).
 * - Quadrant 3 (181-270): Values decrease 0 -> -1 (Negative Mirror of Q1).
 * - Quadrant 4 (271-360): Values increase -1 -> 0 (Negative Mirror of Q1).
 *
 * Storing only 91 bytes saves 270 bytes of Flash memory.
 */
static const int8_t MATH_sin_LUT[] PROGMEM = {
  0, 2, 4, 7, 9, 11, 13, 15, 18, 20,
  22, 24, 26, 29, 31, 33, 35, 37, 39, 41,
  43, 46, 48, 50, 52, 54, 56, 58, 60, 62,
  64, 65, 67, 69, 71, 73, 75, 76, 78, 80,
  82, 83, 85, 87, 88, 90, 91, 93, 94, 96,
  97,  99, 100, 101, 103, 104, 105, 107, 108, 109,
  110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
  119, 120, 121, 121, 122, 123, 123, 124, 124, 125,
  125, 125, 126, 126, 126, 127, 127, 127, 127, 127,
};

/**
 * @brief  Calculates the Sine of an angle using Integer Math.
 *
 * @param  angle  Input angle in degrees.
 *                Accepted Range: -32768 to +32767.
 *                (Ideally 0-359 for speed).
 *
 * @return int8_t Scaled Sine value (-127 to +127).
 */
static int8_t MATH_sin(int16_t angle) {
  /*
   * 1. INPUT NORMALIZATION
   * ----------------------
   * We need the angle to be in the range [0, 359].
   *
   * Optimization Note:
   * On an 8-bit AVR, the modulo operator (%) involves a software division library
   * which is slow (~100-200 cycles).
   * Since compass headings usually change incrementally (e.g., 359 -> 360),
   * a `while` loop subtract/add is significantly faster than division.
   */
  while (angle < 0)
    angle += 360;
  while (angle >= 360)
    angle -= 360;

  /*
   * 2. QUADRANT MAPPING
   * -------------------
   * Map the 0-359 angle to the 0-90 lookup table index.
   */

  /* Quadrant 1: 0 to 90 degrees */
  if (angle < 90) {
    /* Direct Lookup */
    return (int8_t) pgm_read_byte(&MATH_sin_LUT[angle]);
  }

  /* Quadrant 2: 91 to 180 degrees */
  if (angle < 180) {
    /* Mirror Horizontal: sin(170) == sin(10) */
    return (int8_t) pgm_read_byte(&MATH_sin_LUT[179 - angle]);
  }

  /* Quadrant 3: 181 to 270 degrees */
  if (angle < 270) {
    /* Mirror Vertical: sin(190) == -sin(10) */
    /* Note: We read the positive value, then negate the result. */
    return -(int8_t) pgm_read_byte(&MATH_sin_LUT[angle - 180]);
  }

  return -(int8_t) pgm_read_byte(&MATH_sin_LUT[359 - angle]);
}

/**
 * @brief  Calculates the Cosine of an angle using Integer Math.
 *
 * @details
 * Relies on the trigonometric identity: cos(x) = sin(x + 90).
 * This allows us to reuse the exact same LUT and normalization logic
 * without consuming extra Flash memory for a Cosine table.
 *
 * @param  angle  Input angle in degrees.
 * @return int8_t Scaled Cosine value (-127 to +127).
 */
static int8_t MATH_cos(int16_t angle) {
  // cos(x) is just sin(x + 90)
  return MATH_sin(angle + 90);
}

static const uint8_t MATH_atan_LUT[] PROGMEM = {
  0,  0,  1,  1,  2,  2,  3,  3,
  4,  4,  5,  5,  5,  6,  6,  7,
  7,  8,  8,  9,  9,  9, 10, 10,
  11, 11, 12, 12, 12, 13, 13, 14,
  14, 15, 15, 15, 16, 16, 17, 17,
  17, 18, 18, 19, 19, 20, 20, 20,
  21, 21, 21, 22, 22, 23, 23, 23,
  24, 24, 25, 25, 25, 26, 26, 26,
  27, 27, 27, 28, 28, 29, 29, 29,
  30, 30, 30, 31, 31, 31, 32, 32,
  32, 33, 33, 33, 33, 34, 34, 34,
  35, 35, 35, 36, 36, 36, 37, 37,
  37, 37, 38, 38, 38, 38, 39, 39,
  39, 40, 40, 40, 40, 41, 41, 41,
  41, 42, 42, 42, 42, 43, 43, 43,
  43, 44, 44, 44, 44, 45, 45, 45,
};

/**
 * @brief  Computes approximate angle in degrees (0-359).
 *
 * OPTIMIZATION STRATEGY:
 * 1. SYMMETRY: We only calculate the angle for 0-45 degrees (Slope 0.0 to 1.0).
 *              We map the other 7 octants using simple subtraction.
 *
 * 2. LINEAR APPROXIMATION:
 *    Real formula: theta = atan(y/x)
 *    Linear approx: theta ~= 45 * (y/x)
 *    Error: Max ~4 degrees at 22.5 degrees. Acceptable for visual compass.
 *
 * 3. 16-BIT MATH:
 *    To calculate (y * 45) / x without overflowing 16-bit integers (max 32767),
 *    inputs are bit-shifted down until they fit. This avoids linking the heavy
 *    32-bit division library.
 *
 * @param  y  Signed 16-bit Y component.
 * @param  x  Signed 16-bit X component.
 * @return uint16_t Angle in degrees.
 */
uint16_t MATH_atan2(int16_t y, int16_t x) {
  // 1. Handle special case (Origin)
  if (x == 0 && y == 0) return 0;

  // 2. Get Absolute Values
  // Cast to unsigned to safely handle -32768
  uint16_t ax = (x < 0) ? (uint16_t)(-x) : (uint16_t)x;
  uint16_t ay = (y < 0) ? (uint16_t)(-y) : (uint16_t)y;

  // 3. Determine Min/Max for the 0-45 degree ratio
  uint16_t mn = (ax < ay) ? ax : ay;
  uint16_t mx = (ax > ay) ? ax : ay;

  // 4. Pre-scale to prevent overflow
  // We need to calculate (mn * 45) / mx.
  // (mn * 45) must fit in uint16_t (< 65535).
  // Therefore, mn must be < 1456 (65535 / 45).
  while (mx > 1400) {
    mx >>= 1;
    mn >>= 1;
  }

  // 5. Calculate Octant Angle (0-45 degrees)
  // Formula: angle = slope * 45
  // Note: If mx is 0 (should be impossible handled by step 1), result is 0.
#if 0
  // linear
  uint16_t angle = (mn * 45) / mx;
#else
  // curve
  uint16_t index = ((uint32_t) mn << 7) / mx;
  if (index > 127) index = 127;
  uint16_t angle = (uint8_t) pgm_read_byte(&MATH_atan_LUT[index]);
#endif

  // 6. Map Octant to Circle (0-360)
  if (x >= 0) {
    if (y >= 0) {
      // Quadrant 1 (x+, y+)
      if (ax >= ay) return angle;          // 0-45
      else          return 90 - angle;     // 45-90
    } else {
      // Quadrant 4 (x+, y-)
      if (ax < ay) return 270 + angle;    // 315-360
      else if (angle == 0) return 0;      // full circle
      else return 360 - angle;            // 270-315
    }
  } else {
    if (y >= 0) {
      // Quadrant 2 (x-, y+)
      if (ax >= ay) return 180 - angle;    // 135-180
      else          return 90 + angle;     // 90-135
    } else {
      // Quadrant 3 (x-, y-)
      if (ax >= ay) return 180 + angle;    // 180-225
      else          return 270 - angle;    // 225-270
    }
  }
}

/*
 * =========================================================================================
 * I2C MASTER SUBSYSTEM - BARE METAL BIT-BANG DRIVER
 * =========================================================================================
 *
 * 1. PHYSICAL LAYER & ELECTRICAL CHARACTERISTICS
 * -----------------------------------------------------------------------------------------
 * TOPOLOGY: Open-Drain / Open-Collector
 * - The bus relies on "Active Low" logic.
 * - Logic 0 (LOW):  The pin is set to OUTPUT and driven to GND.
 * - Logic 1 (HIGH): The pin is set to INPUT (High-Z / Tri-state).
 *                   The voltage is pulled to VCC by external resistors.
 * - CRITICAL: We NEVER drive a hard Logic 1 (VCC) output. This would cause a short circuit
 *             if a target tries to pull the line Low simultaneously (Clock Stretching/ACK).
 *
 * IMPEDANCE & CURRENT CALCULATION:
 * - Configuration: 1 Master (ATtiny/Uno) + 3 Targets (OLED, BME280, QMC5883P).
 * - Assumption: Each module contains a ~4.7k Ohm pull-up resistor on SDA/SCL.
 * - Total Bus Resistance (Parallel):
 *   R_total = 1 / (1/4700 + 1/4700 + 1/4700 + 1/4700) ~= 1175 Ohms (Worst case with 4 resistors).
 *   R_total = ~1566 Ohms (Typical if Master has no pull-up).
 * - Current Sink Requirement (at 3.3V):
 *   I_sink = VCC / R_total = 3.3V / 1566 Ohms ~= 2.1 mA.
 * - Safety Margin:
 *   The ATtiny85 can sink ~20mA per pin. The I2C Standard requires sinking 3mA.
 *   Result: The setup is electrically safe.
 * - Internal Pull-ups:
 *   The MCU's internal pull-ups (~30k) are explicitly DISABLED in software to prevent
 *   unpredictable impedance changes and reduce phantom current consumption.
 *
 * -----------------------------------------------------------------------------------------
 * 2. PROTOCOL LOGIC: THE "SETUP & SAMPLE" MODEL
 * -----------------------------------------------------------------------------------------
 * The state of SDA is dictated by the state of SCL.
 *
 * STATE A: SCL IS LOW (The "Setup" Phase)
 * - The Bus is "Claimed" by the Master.
 * - SDA is MUTABLE.
 * - The transmitter (Master during Write, Target during Read) sets the bit value now.
 * - SDA changes are safe because the receiver is not looking at the line yet.
 *
 * STATE B: SCL IS HIGH (The "Sample" Phase)
 * - SDA is FROZEN.
 * - The receiver (Target during Write, Master during Read) samples the bit now.
 * - Rising SCL Edge: Logically marks the start of the bit. Data must be stable.
 * - Falling SCL Edge: Logically marks the end of the bit.
 *
 * EXCEPTION: START & STOP CONDITIONS
 * - These are the ONLY times SDA is allowed to change while SCL is HIGH.
 * - START: SDA falls while SCL is High. (Indicates bus is now BUSY).
 * - STOP:  SDA rises while SCL is High. (Indicates bus is now IDLE).
 *
 * -----------------------------------------------------------------------------------------
 * 3. READ/WRITE SYMMETRY
 * -----------------------------------------------------------------------------------------
 * - WRITE BIT: Master asserts SDA (Low/High-Z) -> Toggles SCL High -> Toggles SCL Low.
 * - READ BIT:  Master releases SDA (High-Z) -> Toggles SCL High -> Senses SDA -> Toggles SCL Low.
 * - TIMING:    During Read, the Master assumes the Target asserts SDA immediately
 *              after the previous SCL falling edge (0ns Hold Time per I2C Spec).
 *              The Master waits one Half-Cycle delay before raising SCL to guarantee
 *              the Target's signal is stable.
 *
 * -----------------------------------------------------------------------------------------
 * 4. BUS STATES
 * -----------------------------------------------------------------------------------------
 * - IDLE: SCL=High, SDA=High (Bus released, pulled up by resistors).
 * - BUSY: Time between a START and a STOP condition.
 * - ACK/NACK (9th Bit):
 *   After 8 data bits, the Transmitter releases SDA (High-Z).
 *   ACK:  Receiver pulls SDA Low.
 *   NACK: Receiver leaves SDA High.
 *
 * -----------------------------------------------------------------------------------------
 * 5. TARGET DEVICE MAP
 * -----------------------------------------------------------------------------------------
 * - SSD1306 OLED: 0x3C (Write Only)
 * - BME280 Env:   0x76 (Read/Write, Registers)
 * - QMC5883P Mag: 0x2C (Read/Write, Registers - Note 'P' variant ID)
 *
 * =========================================================================================
 */

/* =========================================================================
 *                      HARDWARE ABSTRACTION LAYER
 * ========================================================================= */

/*
 * I2C Speed Configuration.
 * 5.0 us = 100 kHz (Standard Mode)
 * 1.2 us = 400 kHz (Fast Mode)
 * 2.0 us = ~250 kHz (Robust Compromise)
 */
#define I2C_DELAY_HALF_CYCLE_US  2

#if defined(__AVR_ATtiny85__)
// ATtiny85: Port B, SDA=PB0, SCL=PB2
#define I2C_PORT_REG PORTB
#define I2C_DDR_REG  DDRB
#define I2C_PIN_REG  PINB
#define PIN_SDA      PB0
#define PIN_SCL      PB2
#else
// Arduino Uno (ATmega328P): Port C, SDA=PC4, SCL=PC5
#define I2C_PORT_REG PORTC
#define I2C_DDR_REG  DDRC
#define I2C_PIN_REG  PINC
#define PIN_SDA      PC4
#define PIN_SCL      PC5
#endif

// Constants for Bitwise Operations (Rule 10.x compliance)
// 1U ensures the shift occurs on an unsigned integer
#define MASK_SDA ((uint8_t)(1U << PIN_SDA))
#define MASK_SCL ((uint8_t)(1U << PIN_SCL))

/* =========================================================================
 *                     PHYSICAL LAYER FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Inserts the half-cycle delay required for bit timing.
 * @details This function isolates the compiler intrinsic `_delay_us` to one location.
 *          Since it is `static inline`, the compiler will replace calls with
 *          the actual delay code, avoiding function call overhead.
 */
static inline void I2C_delay() {
  _delay_us(I2C_DELAY_HALF_CYCLE_US);
}

/**
 * @brief  Drives the SDA line to Logic 0 (LOW).
 * @details
 * - PHYSICAL: Pin configured as OUTPUT. Voltage driven to GND.
 * - LOGICAL:  Sets Data Direction Register (DDR) bit to 1.
 *             Sets PORT register bit to 0.
 */
static inline void I2C_sda_low() {
  // Set bit in DDR to make it Output
  I2C_DDR_REG  = (uint8_t)(I2C_DDR_REG | MASK_SDA);
  // Clear bit in PORT to drive Low
  I2C_PORT_REG = (uint8_t)(I2C_PORT_REG & (uint8_t)(~MASK_SDA));
}

/**
 * @brief  Releases the SDA line to Logic 1 (HIGH-Z).
 * @details
 * - PHYSICAL: Pin configured as INPUT. Voltage floats.
 *             External resistors pull the line to VCC.
 * - LOGICAL:  Sets Data Direction Register (DDR) bit to 0.
 *             CRITICAL: Sets PORT bit to 0 to DISABLE internal MCU pull-up.
 */
static inline void I2C_sda_high() {
  // Clear bit in DDR to make it Input (High Impedance)
  I2C_DDR_REG  = (uint8_t)(I2C_DDR_REG & (uint8_t)(~MASK_SDA));
  // Clear bit in PORT to DISABLE internal pull-up
  I2C_PORT_REG = (uint8_t)(I2C_PORT_REG & (uint8_t)(~MASK_SDA));
}

/**
 * @brief  Drives the SCL line to Logic 0 (LOW).
 * @details Sets DDR to Output, PORT to Low.
 */
static inline void I2C_scl_low() {
  // Set bit in DDR to make it Output
  I2C_DDR_REG  = (uint8_t)(I2C_DDR_REG | MASK_SCL);
  // Clear bit in PORT to drive Low
  I2C_PORT_REG = (uint8_t)(I2C_PORT_REG & (uint8_t)(~MASK_SCL));
}

/**
 * @brief  Releases the SCL line to Logic 1 (HIGH-Z).
 * @details Sets DDR to Input, PORT to Low (Disable internal pull-up).
 */
static inline void I2C_scl_high() {
  // Clear bit in DDR to make it Input (High Impedance)
  I2C_DDR_REG  = (uint8_t)(I2C_DDR_REG & (uint8_t)(~MASK_SCL));
  // Clear bit in PORT to DISABLE internal pull-up
  I2C_PORT_REG = (uint8_t)(I2C_PORT_REG & (uint8_t)(~MASK_SCL));
}

/**
 * @brief  Reads the current logic state of the SDA line.
 * @return true if Voltage > V_threshold (Logic 1), false otherwise.
 */
static inline bool I2C_sda_sense() {
  return ((I2C_PIN_REG & MASK_SDA) != 0);
}

/* =========================================================================
 *                     DATA LINK LAYER FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Transmits 8 bits to the bus and reads the ACK bit.
 * @details
 * Implements the "Setup/Sample" model:
 * 1. SCL Low  (Setup):  Master changes SDA.
 * 2. Delay:             Wait for signal to stabilize.
 * 3. SCL High (Sample): Target reads SDA.
 * 4. Delay:             Wait for Target hold time.
 *
 * @param  byte_val  The byte to transmit (MSB First).
 * @return true if Target ACKed (pulled SDA Low), false if NACK (SDA High).
 */
static bool I2C_transmit(uint8_t byte_val) {
  uint8_t mask = 0x80U; // Start with MSB (10000000)

  for (uint8_t i = 0; i < 8U; i++) {
    // --- SETUP PHASE ---
    // SCL is currently Low. We are allowed to change SDA.
    if ((byte_val & mask) != 0)
      I2C_sda_high(); // Send '1'
    else
      I2C_sda_low();  // Send '0'

    // Wait for signal rise/fall time
    I2C_delay();

    // --- SAMPLE PHASE ---
    // Drive SCL High. Target samples the bit now.
    I2C_scl_high();
    I2C_delay(); // Hold the clock High

    // --- END PHASE ---
    // Drive SCL Low. Target prepares for next bit.
    I2C_scl_low();

    mask = (uint8_t)(mask >> 1U); // Shift mask to next bit
  }

  // --- ACKNOWLEDGE PHASE (9th Bit) ---
  // 1. Master releases SDA (High-Z) to let Target control it.
  I2C_sda_high();
  I2C_delay();

  // 2. Master pulses SCL High to read the ACK.
  I2C_scl_high();
  I2C_delay();

  // 3. Master reads SDA.
  // Logic 0 = ACK (Target pulled low). Logic 1 = NACK (Target did nothing).
  bool ack_received = !I2C_sda_sense();

  // 4. End Cycle.
  I2C_scl_low();
  I2C_delay();

  return ack_received;
}

/**
 * @brief  Receives 8 bits from the bus and sends an ACK or NACK.
 * @details
 * READ TIMING:
 * 1. Previous SCL Falling Edge: Target asserts Data Bit.
 * 2. Master Delay:              Wait for Target signal to stabilize.
 * 3. Master SCL High:           Freeze data.
 * 4. Master Sample:             Read SDA.
 *
 * @param  send_ack  true to send ACK (Request more data), false for NACK (End).
 * @return The byte received from the Target.
 */
static uint8_t I2C_receive(bool send_ack) {
  uint8_t received_byte = 0;

  // Ensure Master is not driving the bus (Input Mode)
  I2C_sda_high();

  for (uint8_t i = 0; i < 8U; i++) {
    // Wait for Target to setup data (triggered by previous SCL Low)
    I2C_delay();

    // --- SAMPLE PHASE ---
    I2C_scl_high(); // Freeze data
    I2C_delay();

    // Shift existing bits to make room for new LSB
    received_byte = (uint8_t)(received_byte << 1U);

    // Sample the line
    if (I2C_sda_sense())
      received_byte = (uint8_t)(received_byte | 1U);

    // --- END PHASE ---
    // SCL Falling Edge triggers Target to assert next bit
    I2C_scl_low();
  }

  // --- ACKNOWLEDGE PHASE ---
  // Master drives SDA to answer the Target.
  if (send_ack)
    I2C_sda_low(); // ACK: "I received it, send next byte"
  else
    I2C_sda_high(); // NACK: "I received it, stop sending"

  I2C_delay();
  I2C_scl_high(); // Pulse Clock
  I2C_delay();
  I2C_scl_low();  // End Cycle
  I2C_delay();

  // Always release SDA at the end of a byte read to return to Idle/Input
  I2C_sda_high();

  return received_byte;
}

/**
 * @brief  Generates the I2C START Condition and transmits the Address.
 * @details
 * PROTOCOL:
 * 1. START: SDA transitions High->Low while SCL is High.
 * 2. ADDRESS: 7-bit Dev Address + R/W Bit (0=Write, 1=Read).
 *
 * @param  dev_addr   7-bit I2C Address.
 * @param  read_mode  true for READ operation, false for WRITE.
 * @return true if Target ACKed the address, false if NACK.
 */
static bool I2C_start(uint8_t dev_addr, bool read_mode) {
  // Ensure bus is nominally Idle (High-Z) before generating Start
  I2C_sda_high();
  I2C_delay();
  I2C_scl_high();
  I2C_delay();

  // --- GENERATE START ---
  I2C_sda_low(); // SDA Falls (Start Condition)
  I2C_delay();

  I2C_scl_low(); // SCL Falls (Bus Claimed)
  I2C_delay();

  // Prepare Address Byte: (Addr << 1) | R/W bit
  uint8_t packet = (uint8_t)(dev_addr << 1U);

  if (read_mode)
    packet = (uint8_t)(packet | 0x01U); // Set LSB to 1 (READ)
  else
    packet = (uint8_t)(packet & 0xFEU); // Clear LSB to 0 (WRITE)

  // Transmit the address byte
  return I2C_transmit(packet);
}

/**
 * @brief  Generates the I2C STOP Condition.
 * @details
 * PROTOCOL: SDA transitions Low->High while SCL is High.
 * This signals the end of the transaction and releases the bus to IDLE.
 */
static void I2C_stop() {
  I2C_sda_low(); // Ensure SDA is Low first
  I2C_delay();

  I2C_scl_high(); // Clock goes High
  I2C_delay();

  I2C_sda_high(); // STOP: SDA Rises while SCL is High
  I2C_delay();
}

/* =========================================================================
 *                     API FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Forces the bus into a known IDLE state.
 * @details
 * Used during initialization or error recovery.
 * If a Target was interrupted mid-byte, it might be holding SDA Low.
 * We toggle SCL 9 times to force the Target to shift out the remaining bits
 * and see a NACK/STOP.
 */
void I2C_recover_bus() {
  // 1. Release lines to check state
  I2C_sda_high();
  I2C_scl_high();
  I2C_delay();

  // 2. Check if SDA is stuck Low
  if (I2C_sda_sense() == false) {
    // 3. Toggle SCL 9 times
    for (uint8_t i = 0; i < 9U; i++) {
      I2C_scl_low();
      I2C_delay();
      I2C_scl_high();
      I2C_delay();
    }
    // 4. Force Stop condition
    I2C_stop();
  }
}

/**
 * @brief  Initializes the I2C Bus.
 * @details
 * 1. Runs Bus Recovery to unstick any Targets.
 * 2. Sets SDA/SCL to Input (High-Z).
 * 3. Disables internal MCU pull-ups.
 */
void I2C_init() {
  I2C_recover_bus();

  // Set Idle State
  I2C_sda_high();
  I2C_scl_high();
}

/**
 * @brief  Reads data from a Target Register.
 * @details
 * Transaction: [START] [ADDR+W] [REG_ADDR] [RESTART] [ADDR+R] [DATA...] [STOP]
 * Note: Uses Repeated Start (RESTART) to change direction without losing bus control.
 *
 * @param  dev_addr  7-bit Target Address.
 * @param  reg_addr  Register address to read from.
 * @param  p_data    Pointer to destination buffer.
 * @param  length    Number of bytes to read.
 * @return bool      Result false=error/NACK true=ACK
 */
bool I2C_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint8_t length) {
  bool status = true;

  if (p_data == NULL)
    return false;
  if (length == 0)
    return false;

  // 1. Write Phase: Set Register Pointer
  if (!I2C_start(dev_addr, false)) {
    I2C_stop();
    return false;
  }

  // Transmit Register Address
  if (!I2C_transmit(reg_addr)) {
    // NACK
    I2C_stop();
    return false;
  }

  // 2. Read Phase: Restart (Start without previous Stop)
  // We send START again to switch to Read Mode
  if (!I2C_start(dev_addr, true)) {
    I2C_stop();
    return false;
  }

  // Receive Payload
  for (uint8_t i = 0; i < length; i++) {
    // Send ACK for all bytes except the last one.
    // The last byte gets a NACK to signal "End of Read".
    bool send_ack = (i < (length - 1U));

    p_data[i] = I2C_receive(send_ack);
  }

  // 3. Stop
  I2C_stop();

  return status;
}

/**
 * @brief  Writes a buffer of data to a specific Target.
 * @details
 * Transaction: [START] [ADDR+W] [DATA 0] ... [DATA N] [STOP]
 *
 * @param  dev_addr  7-bit Target Address.
 * @param  p_data    Pointer to source buffer.
 * @param  length    Number of bytes to write.
 * @return bool      Result false=error/NACK true=ACK
 */
bool I2C_write(uint8_t dev_addr, const uint8_t *p_data, uint8_t length) {
  bool status = true;

  // Rule 17.x: Parameter Checks
  if (p_data == NULL)
    return false;
  if (length == 0)
    return false;

  // 1. Start + Address
  // 'false' indicates Write Mode (R/W bit = 0)
  if (!I2C_start(dev_addr, false)) {
    I2C_stop();
    return false;
  }

  // 2. Data Payload
  for (uint8_t i = 0; i < length; i++) {
    if (!I2C_transmit(p_data[i])) {
      // NACK
      status = false;
      break; // Stop transmitting immediately on error
    }
  }

  // 3. Stop
  // Always generate STOP to release the bus, even if NACK occurred.
  I2C_stop();

  return status;
}

/*
 * =========================================================================================
 * BATTERY MONITOR SUBSYSTEM (PORTABLE)
 * =========================================================================================
 *
 * 1. THEORY OF OPERATION: THE "SECRET VOLTMETER"
 * -----------------------------------------------------------------------------------------
 * Standard ADC usage measures an Unknown Input against a Known Reference (VCC).
 * Formula: ADC = (Input / VCC) * 1024
 *
 * Here, we flip the equation. We measure the "Internal 1.1V Bandgap Reference"
 * using VCC as the "Reference".
 * Formula: ADC = (1.1V / VCC) * 1024
 *
 * Solving for VCC:
 * VCC = (1.1V * 1024) / ADC
 *
 * 2. HARDWARE DIFFERENCES
 * -----------------------------------------------------------------------------------------
 * While both chips use the same logic, the internal Multiplexer (MUX) addresses differ:
 *
 * ATtiny85:
 * - Reference: VCC (REFS[2:0] = 000)
 * - Input:     1.1V Bandgap (MUX[3:0] = 1100 / 0x0C)
 *
 * ATmega328P (Arduino Uno):
 * - Reference: AVCC (REFS[1:0] = 01)
 * - Input:     1.1V Bandgap (MUX[3:0] = 1110 / 0x0E)
 *
 * 3. CALIBRATION
 * -----------------------------------------------------------------------------------------
 * The Internal Bandgap is nominally 1.100V but has a factory tolerance of +/- 10%.
 * It might be 1.05V or 1.18V.
 *
 * CALIBRATION CONSTANT = Bandgap_Voltage_mV * 1024
 * Default: 1100 * 1024 = 1126400
 *
 * To Calibrate:
 * 1. Measure actual VCC with a multimeter (e.g., 3050 mV).
 * 2. Read the raw ADC value from this function (e.g., 375).
 * 3. New Constant = 3050 * 375 = 1143750.
 * =========================================================================================
 */

/* =========================================================================
 *                        CONFIGURATION MACROS
 * ========================================================================= */

/*
 * Factory default constant.
 * Formula: 1100mV * 1024 ADC Steps = 1126400
 * Tune this value if your voltage reading is consistently off.
 */
#define BATTERY_CALIB_CONST  1126400L

/*
 * MUX CONFIGURATION
 * Selects VCC as Reference and 1.1V Bandgap as Input.
 */
#if defined(__AVR_ATtiny85__)
    /*
     * ATtiny85:
     * REFS[2:0] = 000 (VCC used as Ref, disconnect PB0)
     * MUX[3:0]  = 1100 (Measure Vbg)
     * Register ADMUX: [0 0 0 0] [1 1 0 0] -> 0x0C
     */
    #define ADC_MUX_SETTING  0x0C
#else
    /*
     * ATmega328P (Arduino Uno):
     * REFS[1:0] = 01 (AVCC with external cap at AREF)
     * MUX[3:0]  = 1110 (Measure Vbg)
     * Register ADMUX: [0 1 0 0] [1 1 1 0] -> 0x4E
     */
    #define ADC_MUX_SETTING  0x4E
#endif

/* =========================================================================
 *                           PUBLIC API
 * ========================================================================= */

/**
 * @brief  Measures the Power Supply Voltage (VCC).
 * @return uint16_t Voltage in millivolts (e.g., 3005 = 3.005V).
 */
uint16_t ADC_get_voltage() {
  uint16_t adc_val;
  uint32_t vcc_calc;

  /* 1. Save previous ADMUX state?
   * In this bare-metal project, we assume we own the ADC.
   * If mixing with other ADC libs, save SREG/ADMUX here.
   */

  /* 2. Configure Multiplexer */
  ADMUX = (uint8_t) ADC_MUX_SETTING;

  /* 3. Enable ADC + Set Prescaler
   * ADEN = 1 (Enable)
   * ADPS = 110 (Prescaler 64).
   * 8MHz / 64 = 125kHz ADC Clock (Ideal range is 50-200kHz).
   */
  ADCSRA = (uint8_t)((1U << ADEN) | (1U << ADPS2) | (1U << ADPS1));

  /* 4. Stabilization Delay
   * The Bandgap reference takes a noticeable time to charge the
   * internal sample capacitor and stabilize (~2ms - 10ms).
   */
  _delay_ms(2);

  /* 5. Dummy Conversion (Warm-up)
   * The first reading after changing the Reference is often garbage.
   * We perform one conversion and discard it.
   */
  ADCSRA |= (uint8_t)(1U << ADSC);     /* Start Conversion */
  while ((ADCSRA & (1U << ADSC)) != 0U) {
    /* Wait for bit to clear */
  }

  /* 6. Real Measurement */
  ADCSRA |= (uint8_t)(1U << ADSC);     /* Start Conversion */
  while ((ADCSRA & (1U << ADSC)) != 0U) {
    /* Wait for bit to clear */
  }

  /* 7. Read ADC Result
   * Order matters: Read Low byte, then High byte (Compiler handles this).
   */
  adc_val = ADC;

  /* 8. Disable ADC
   * Crucial for Coin Cell life. The ADC consumes ~200uA if left enabled.
   */
  ADCSRA &= (uint8_t) ~(1U << ADEN);

  /* 9. Calculate VCC
   * Avoid divide-by-zero if something catastrophic happened.
   */
  if (adc_val == 0U)
    return 0U;

  /* Math: CONST / ADC = VCC_mV */
  vcc_calc = (uint32_t) BATTERY_CALIB_CONST / adc_val;

  return (uint16_t) vcc_calc;
}

/**
 * @brief  Calculates battery health percentage for CR2032.
 * @note   Based on discharge curve under load (OLED on).
 *
 * @param  mv Voltage in millivolts.
 * @return uint8_t Percentage (0-100).
 */
uint8_t ADC_get_percentage(uint16_t mv) {
  /* CR2032 Curve (Approximate linear region under load)
   * > 3.00V : 100% (Fresh)
   * < 2.40V : 0%   (Dead/Brownout risk)
   */
  if (mv >= 3000U) {
    return 100U;
  }
  if (mv <= 2400U) {
    return 0U;
  }

  /* Map Range: 2400..3000 (span 600) -> 0..100
   * Formula: (Voltage - Min) / 6
   */
  return (uint8_t)((mv - 2400U) / 6U);
}

/*
 * =============================================================================
 *                    ATTiny85 sleep/standby for low power consumption
 * BATTERY LIFE ESTIMATION (ATtiny85 + SSD1306 + QMC5883P + BME280 + LED)
 * =============================================================================
 *
 * Components:
 *   - ATtiny85 MCU
 *   - SSD1306 OLED
 *   - Status LED
 *   - QMC5883P magnetometer
 *   - BME280 environmental sensor
 *
 * Typical currents at 3V:
 * -----------------------
 * Sleep Mode (power-down, minimal usage):
 *   - ATtiny85: 0.5 µA
 *   - SSD1306: 0 µA (powered down)
 *   - LED: 0 µA (off)
 *   - QMC5883P: 1 µA (shutdown)
 *   - BME280: 0.1 µA (low-power)
 *   -> Total sleep current: I_sleep ≈ 1.6 µA ≈ 0.0016 mA
 *
 * Active Mode (everything running):
 *   - ATtiny85: 5 mA
 *   - SSD1306: 15 mA
 *   - LED: 5 mA
 *   - QMC5883P: 0.1 mA
 *   - BME280: 0.03 mA
 *   -> Total active current: I_active ≈ 25.13 mA
 *
 * Battery: CR2032, typical capacity 225 mAh
 *
 * ============================================================================
 * MAXIMUM BATTERY LIFE (full sleep, everything off except MCU + sensors in low power)
 * ============================================================================
 * I_sleep = 0.0016 mA
 * Battery life ≈ Capacity / Current
 * Battery life ≈ 225 mAh / 0.0016 mA ≈ 140,625 hours
 * Convert to years: 140,625 / 24 / 365 ≈ 16 years (!!)
 * Note: This is theoretical; real-world factors like self-discharge, leakage,
 *       and temperature reduce practical life.
 *
 * ============================================================================
 * MINIMUM BATTERY LIFE (full active, everything running continuously)
 * ============================================================================
 * I_active = 25.13 mA
 * Battery life ≈ 225 mAh / 25.13 mA ≈ 8.95 hours
 *
 * =============================================================================
 */

// Note: On ATtiny85, all pin changes (PCINT0 to PCINT5)
// trigger the SAME vector: PCINT0_vect.
ISR(PCINT0_vect) {
  // This code runs immediately upon wake-up.
  // You can leave this empty if you just want to wake up.
}

void PWR_power_down() {
  byte buttonState;

  do {
    ADCSRA &= ~(1 << ADEN);               // Disable ADC
    ACSR |= (1 << ACD);                   // Disable analog comparator
    power_all_disable();                  // Disable all peripherals
    PCMSK |= (1 << PCINT1);               // Enable PCINT1
    GIMSK |= (1 << PCIE);                 // Enable pin change interrupts globally
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // lowest power mode
    sleep_enable();                       // allow sleep
    sleep_bod_disable();                  // disable brown-out detection
    sei();                                // ensure interrupts enabled
    sleep_cpu();                          // MCU sleeps here
    sleep_disable();                      // resumes here after wake
    power_all_enable();                   // Re-enable after wake
    PCMSK &= ~(1 << PCINT1);              // Disable PCINT1

    // probe pin6
    buttonState = digitalRead(PB1); // wait until  HIGH to LOW
  } while (buttonState != LOW);
}

/*
 * =========================================================================================
 * SSD1306 OLED SUBSYSTEM (128x64 I2C)
 * =========================================================================================
 *
 * 1. HARDWARE OVERVIEW
 * -----------------------------------------------------------------------------------------
 * - Controller: SSD1306 (Solomon Systech)
 * - Resolution: 128 columns x 64 rows.
 * - Interface:  I2C (Address 0x3C).
 *
 * 2. MEMORY LAYOUT (PAGE ADDRESSING MODE)
 * -----------------------------------------------------------------------------------------
 * The GDDRAM (Graphic Display Data RAM) is organized into 8 PAGES (Page 0 to Page 7).
 * - Each Page represents 8 horizontal pixel rows.
 * - Each Byte written to the display represents a vertical column of 8 pixels.
 * - LSB (Bit 0) is the Top pixel; MSB (Bit 7) is the Bottom pixel of that page.
 *
 * Example: Writing 0x03 to Page 0, Column 0 lights up pixels (0,0) and (0,1).
 *
 * 3. OPTIMIZATION STRATEGY (STREAMING)
 * -----------------------------------------------------------------------------------------
 * The I2C overhead (Start/Address/Stop) is significant.
 * - BAD:  Start -> Data(1 byte) -> Stop.
 * - GOOD: Start -> Data(128 bytes) -> Stop.
 *
 * This driver groups writes into streams wherever possible (e.g., clearing the screen,
 * drawing icons, printing strings) to maximize frame rate and minimize bus lockups.
 *
 * 4. MATH OPTIMIZATION (COMPASS ROTATION)
 * -----------------------------------------------------------------------------------------
 * The compass rotation uses Fixed Point Math (Q8.8 format) to avoid floating point libraries.
 * - 256 represents 1.0.
 * - This allows the ATtiny85 to perform rotation transforms using simple bit-shifts
 *   and additions rather than expensive trigonometric calculations.
 * =========================================================================================
 */

/* =========================================================================
 *                        CONFIGURATION & CONSTANTS
 * ========================================================================= */

#define SSD1306_I2C_ADDR        0x3C
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

/* I2C Control Bytes */
#define SSD1306_CTRL_CMD        0x00 // Co=0, D/C=0 (Command Stream)
#define SSD1306_CTRL_DATA       0x40 // Co=0, D/C=1 (Data Stream)

/* SSD1306 Command Set */
typedef enum {
  SSD1306_CMD_DISPLAY_OFF     = 0xAE,
  SSD1306_CMD_DISPLAY_ON      = 0xAF,
  SSD1306_CMD_SET_CONTRAST    = 0x81,
  SSD1306_CMD_DISPLAY_ALL_ON  = 0xA5,
  SSD1306_CMD_DISPLAY_NORMAL  = 0xA6,
  SSD1306_CMD_DISPLAY_INVERSE = 0xA7,
  SSD1306_CMD_MEM_MODE        = 0x20,
  SSD1306_CMD_COL_LOW         = 0x00,
  SSD1306_CMD_COL_HIGH        = 0x10,
  SSD1306_CMD_PAGE_ADDR       = 0xB0,
  SSD1306_CMD_SCAN_DIR_NORM   = 0xC0,
  SSD1306_CMD_SCAN_DIR_REM    = 0xC8,
  SSD1306_CMD_SEG_REMAP_NORM  = 0xA0,
  SSD1306_CMD_SEG_REMAP_REM   = 0xA1,
  SSD1306_CMD_SET_MUX_RATIO   = 0xA8,
  SSD1306_CMD_SET_DISP_OFFSET = 0xD3,
  SSD1306_CMD_SET_START_LINE  = 0x40,
  SSD1306_CMD_CHARGE_PUMP     = 0x8D
} SSD1306_cmd_t;

/* =========================================================================
 *                           ASSETS (FONTS & BITMAPS)
 * ========================================================================= */

/* Init Sequence (Command Stream) */
static const uint8_t SSD1306_init_data[] PROGMEM = {
  SSD1306_CMD_DISPLAY_OFF,
  SSD1306_CMD_MEM_MODE, 0x02,       // Page Addressing
  SSD1306_CMD_PAGE_ADDR,            // Start Page 0
  SSD1306_CMD_SCAN_DIR_REM,         // Flip Y
  SSD1306_CMD_COL_LOW,
  SSD1306_CMD_COL_HIGH,
  SSD1306_CMD_SET_START_LINE,
  SSD1306_CMD_SET_CONTRAST, 0x7F,   // Medium Brightness
  SSD1306_CMD_SEG_REMAP_REM,        // Flip X
  SSD1306_CMD_DISPLAY_NORMAL,
  SSD1306_CMD_SET_MUX_RATIO, 0x3F,  // 1/64 Duty
  SSD1306_CMD_DISPLAY_ALL_ON - 1,   // Output follows RAM (0xA4)
  SSD1306_CMD_SET_DISP_OFFSET, 0x00,
  0xD5, 0xF0,                       // Osc Freq
  0xD9, 0x22,                       // Pre-charge
  0xDA, 0x12,                       // COM Hardware config
  0xDB, 0x20,                       // VCOMH
  SSD1306_CMD_CHARGE_PUMP, 0x14,    // Enable DC-DC
  SSD1306_CMD_DISPLAY_ON
};

/* =========================================================================
 *                           PRIVATE HELPERS
 * ========================================================================= */

/**
 * @brief  Sends a single byte command to the OLED.
 * @details Wraps I2C Start/Write/Stop.
 * @param  cmd  Command byte.
 */
static void SSD1306_cmd(uint8_t cmd) {
  uint8_t buffer[2];

  buffer[0] = SSD1306_CTRL_CMD;
  buffer[1] = cmd;
  // Uses the I2C subsystem to safely handle Start/Stop/Ack
  I2C_write(SSD1306_I2C_ADDR, buffer, 2);
}

/**
 * @brief  Initializes the SSD1306 OLED.
 * @details Sends the initialization command sequence defined in INIT_SEQ.
 */
void SSD1306_init() {
  // Send init sequence byte-by-byte
  // Optimization: Could stream this, but init is done once.
  for (uint8_t i = 0; i < sizeof(SSD1306_init_data); i++)
    SSD1306_cmd(pgm_read_byte(&SSD1306_init_data[i]));
}

/**
 * @brief  Puts the OLED into deep sleep (< 10µA).
 */
void SSD1306_sleep() {
  SSD1306_cmd(SSD1306_CMD_DISPLAY_OFF);
}

/**
 * @brief  Wakes the OLED from sleep (GDDRAM is retained!)
 */
void SSD1306_wake() {
  SSD1306_cmd(SSD1306_CMD_DISPLAY_ON); // Display ON
}

/**
 * @brief  Sets the GDDRAM Page/Column pointer.
 * @param  page  Page (0-7).
 * @param  col   Column (0-127).
 */
void SSD1306_set_cursor(uint8_t page, uint8_t col) {
  // Ensure bounds
  if (page > 7) page = 7;
  if (col > 127) col = 127;

  uint8_t cmds[3];
  cmds[0] = SSD1306_CMD_PAGE_ADDR | (page & 0x07);       // Set Page (0-7)
  cmds[1] = SSD1306_CMD_COL_LOW | (col & 0x0F);          // Set Lower Column Start
  cmds[2] = SSD1306_CMD_COL_HIGH | ((col >> 4) & 0x0F); // Set Higher Column Start

  // We can't use I2C_WriteBuffer blindly for a pure command stream
  // because I2C_WriteBuffer sends [00][CMD]...
  // Here we send 3 separate commands.
  SSD1306_cmd(cmds[0]);
  SSD1306_cmd(cmds[1]);
  SSD1306_cmd(cmds[2]);
}

/**
 * @brief  Clears the entire display buffer.
 * @details Writes 0x00 to all pages/columns.
 */
void SSD1306_clear() {
  for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) {
    SSD1306_set_cursor(page, 0);

    // We need to write 128 bytes
    I2C_start(SSD1306_I2C_ADDR, false);
    I2C_transmit(SSD1306_CTRL_DATA); // Data Mode

    for (uint8_t i = 0; i < SSD1306_WIDTH; i++)
      I2C_transmit(0x00);

    I2C_stop();  // Send the packet to flush the buffer
  }
}

/*
 * =========================================================================================
 * QMC5883P MAGNETOMETER SUBSYSTEM
 * =========================================================================================
 *
 * 1. DEVICE OVERVIEW
 * -----------------------------------------------------------------------------------------
 * - Part Number: QST QMC5883P (Note: 'P' variant is distinct from 'L')
 * - I2C Address: 0x2C (7-bit)
 * - Datasheet:   Rev C (13-52-19)
 *
 * 2. POWER MANAGEMENT STRATEGY (SINGLE SHOT)
 * -----------------------------------------------------------------------------------------
 * To maximize battery life on a CR2032, this driver avoids "Continuous Mode".
 * We utilize the hardware's native "Single Mode" (Forced Mode).
 *
 * - IDLE:   The sensor sits in SUSPEND mode (~3uA current).
 * - ACTIVE: The driver writes 'MODE_SINGLE' to Control Register 1.
 * - ACTION: The sensor Wakes -> Measures -> Updates Data -> Auto-Suspends.
 *
 * This provides a fail-safe mechanism: if the microcontroller hangs or crashes,
 * the sensor will not drain the battery because it automatically returns to sleep.
 *
 * 3. STARTUP DEGAUSSING (HARD IRON / DOMAIN ALIGNMENT)
 * -----------------------------------------------------------------------------------------
 * Upon initialization, the driver performs a specific "Set/Reset" sequence.
 * By forcing a "Set Only" pulse followed by restoring normal "Set/Reset", we generate
 * a strong magnetic field internally. This re-aligns the magnetic domains of the
 * Permalloy film, canceling out offsets caused by nearby magnetic components (speaker).
 *
 * 4. REGISTER MAPPING (0x2C Variant)
 * -----------------------------------------------------------------------------------------
 * 0x00: Chip ID
 * 0x01-0x06: Data Output (X, Y, Z)
 * 0x09: Status Register
 * 0x0A: Control Register 1 (Mode, ODR, OSR, OSR2)
 * 0x0B: Control Register 2 (Soft Reset, Rollover, Interrupt)
 * =========================================================================================
 */

// I2C Address
#define QMC5883P_I2C_ADDR            0x2c

/* =========================================================================
 *                        REGISTER & ENUM DEFINITIONS
 * ========================================================================= */

/**
 * @brief Register Map (Datasheet Page 14)
 * Note: QMC5883P Control registers are at 0x0A/0x0B (shifted vs QMC5883L)
 */
typedef enum {
  QMC5883P_REG_CHIPID   = 0x00, // Chip ID register
  QMC5883P_REG_X_LSB    = 0x01, // X-axis output LSB register
  QMC5883P_REG_X_MSB    = 0x02, // X-axis output MSB register
  QMC5883P_REG_Y_LSB    = 0x03, // Y-axis output LSB register
  QMC5883P_REG_Y_MSB    = 0x04, // Y-axis output MSB register
  QMC5883P_REG_Z_LSB    = 0x05, // Z-axis output LSB register
  QMC5883P_REG_Z_MSB    = 0x06, // Z-axis output MSB register
  QMC5883P_REG_STATUS   = 0x09, // Status register
  QMC5883P_REG_CONTROL1 = 0x0A, // Control register 1 (Mode/ODR/OSR)
  QMC5883P_REG_CONTROL2 = 0x0B, // Control register 2 (Reset/Config)
} QMC5883P_register_t;

/**
 * @brief Status Register Bits (0x09)
 */
typedef enum {
  QMC5883P_STATUS_DRDY = (1 << 0), // Data Ready
  QMC5883P_STATUS_OVFL = (1 << 1), // Overflow
} QMC5883P_mode_t;

/**
 * @brief Control register 1
 */
typedef enum {
  // Operating Mode (Bits 1:0)
  QMC5883P_MODE_SUSPEND    = 0, // Suspend mode (Low Power, <3uA)
  QMC5883P_MODE_NORMAL     = 1, // Normal mode (Old term)
  QMC5883P_MODE_SINGLE     = 2, // Single Measurement (Auto-Sleep)
  QMC5883P_MODE_CONTINUOUS = 3, // Continuous Read Mode

  // Output Data Rate (Bits 3:2)
  // In Single Mode, this determines the speed of the one-shot measurement.
  QMC5883P_ODR_10HZ  = (0 << 2), // 10 Hz output data rate
  QMC5883P_ODR_50HZ  = (1 << 2), // 50 Hz output data rate
  QMC5883P_ODR_100HZ = (2 << 2), // 100 Hz output data rate
  QMC5883P_ODR_200HZ = (3 << 2), // 200 Hz output data rate (Fastest)

  // Over Sample Ratio 1 (Bits 5:4)
  // Controls bandwidth/noise. Higher OSR = Lower Noise, Higher Current.
  QMC5883P_OSR_8 = (0 << 4), // OSR = 8 (Best Filtering)
  QMC5883P_OSR_4 = (1 << 4), // OSR = 4
  QMC5883P_OSR_2 = (2 << 4), // OSR = 2
  QMC5883P_OSR_1 = (3 << 4), // OSR = 1 (Lowest Power)

  // Over Sample Ratio 2 / Downsample (Bits 7:6)
  // Secondary internal filter setting.
  QMC5883P_OSR2_1 = (0 << 6), // Downsample ratio = 1
  QMC5883P_OSR2_2 = (1 << 6), // Downsample ratio = 2
  QMC5883P_OSR2_4 = (2 << 6), // Downsample ratio = 4
  QMC5883P_OSR2_8 = (3 << 6), // Downsample ratio = 8
} QMC5883P_control1_t;

/**
 * @brief Control register 2
 */
typedef enum {
  // Set/Reset Logic (Bits 1:0)
  // Used for the Degaussing / Domain Alignment sequence.
  QMC5883P_SETRESET_ON      = 0, // Normal: Set and Reset Pulse On
  QMC5883P_SETRESET_SETONLY = 1, // Pulse: Set Only (Force logic)
  QMC5883P_SETRESET_OFF     = 2, // Off: Set and Reset Off

  // Field Range (Bits 3:2)
  QMC5883P_RANGE_30G = (0 << 2), // +/- 30 Gauss range
  QMC5883P_RANGE_12G = (1 << 2), // +/- 12 Gauss range
  QMC5883P_RANGE_8G  = (2 << 2), // +/- 8 Gauss range (Balanced)
  QMC5883P_RANGE_2G  = (3 << 2), // +/- 2 Gauss range (High Sensitivity)

  // Self test
  QMC5883P_SELF_TEST  = (1 << 6), // Self test
  QMC5883P_SOFT_RESET = (1 << 7), // Soft Reset (Reboots chip logic)
} QMC5883P_control2_t;

/**
 * @brief Sensor sensitivity LSB/G
 */
enum {
  QMC5883P_SENSITIVITY_2G  = 15000,
  QMC5883P_SENSITIVITY_8G  = 3750,
  QMC5883P_SENSITIVITY_12G = 2500,
  QMC5883P_SENSITIVITY_30G = 1000,
};

enum {
  QMC5883P_CONFIG_CONTROL1 = QMC5883P_OSR_8 |   // Lower current moderate noise filtering
                             QMC5883P_OSR2_8 |  // Light secondary downsampling
                             QMC5883P_ODR_50HZ, // 50 Hz is visually smooth enough

  QMC5883P_CONFIG_CONTROL2 = QMC5883P_RANGE_2G, // ±8 Gauss range (balanced)

  QMC5883P_CONFIG_SENSITIVITY = QMC5883P_SENSITIVITY_2G // 15000 LSB/G
};

/* =========================================================================
 *                            DATA STRUCTURES
 * ========================================================================= */

/**
 * @brief Container for the 3-axis magnetic data.
 *        units are in milli Gauss
 *
 * @date 2026-02-15 23:10:07
 *   Calculate the angle with raw sensor values as they are larger and more stable
 */
struct QMC5883P_data {
  int16_t x;
  int16_t y;
  int16_t z;
  uint16_t heading;
};

/* =========================================================================
 *                           PRIVATE FUNCTIONS
 * ========================================================================= */

/**
 * @brief  Writes a byte to a specific register on the QMC5883P.
 * @details Wrapper for the I2C Write transaction.
 * @param  reg   Register Address.
 * @param  data  Value to write.
 */
void QMC5883P_cmd(uint8_t reg, uint8_t data) {
  uint8_t buffer[2];

  buffer[0] = reg;
  buffer[1] = data;
  // Uses the I2C subsystem to safely handle Start/Stop/Ack
  I2C_write(QMC5883P_I2C_ADDR, buffer, 2);
}

/* =========================================================================
 *                           PUBLIC API FUNCTIONS
 * ========================================================================= */

/**
 * @brief Initializes the QMC5883P sensor for continuous measurement mode.
 * @details
 * 1. Performs Soft Reset to clear stuck states.
 * 2. Runs Degaussing (Set Only -> Set/Reset On) to align magnetic domains.
 * 3. Configuration of output data rate and oversampling.
 * 4. Starts continuous measurement mode.
 *
 * Continuous mode is used during active compass display windows
 * because it provides:
 *  - Stable output
 *  - No repeated suspend transitions
 *  - Reduced risk of internal state-machine lockups
 *
 * @return true if initialization succeeds, false otherwise.
 */
void QMC5883P_init() {
  // SOFT RESET
  // Resets registers to default. Chip enters Suspend Mode automatically.
  QMC5883P_cmd(QMC5883P_REG_CONTROL2, QMC5883P_SOFT_RESET);

  // Wait for POR (Power On Reset) to complete (~250us per datasheet)
  _delay_ms(20);

  /*
   * @date 2026-02-16 01:59:24
   * Take the chip out of the suspend mode before writing control2,
   *   this is not clear from the data sheet
   */
  QMC5883P_cmd(QMC5883P_REG_CONTROL2, QMC5883P_CONFIG_CONTROL2);
  QMC5883P_cmd(QMC5883P_REG_CONTROL1, QMC5883P_CONFIG_CONTROL1 | QMC5883P_MODE_CONTINUOUS);

  // SET SENSITIVITY
  QMC5883P_cmd(QMC5883P_REG_CONTROL2, QMC5883P_CONFIG_CONTROL2);

  // CONFIGURE AGAIN
  QMC5883P_cmd(QMC5883P_REG_CONTROL1, QMC5883P_CONFIG_CONTROL1 | QMC5883P_MODE_CONTINUOUS);
}

/**
 * @brief Places the sensor into low-power suspend mode.
 * @details
 * In suspend mode:
 *   - Measurement engine is stopped
 *   - Current consumption drops to a few microamps
 *   - Registers retain configuration
 *
 * This function should be called when:
 *   - Compass display is no longer needed
 *   - System enters long idle period
 *
 * @return true if command succeeds, false otherwise.
 */
void QMC5883P_sleep() {
  QMC5883P_cmd(QMC5883P_REG_CONTROL1, QMC5883P_MODE_SUSPEND);
}

/**
 * @brief  Performs a Single-Shot measurement and reads data.
 * @details
 * 1. Wakes sensor by setting MODE_SINGLE.
 * 2. Waits for measurement completion (Fixed delay).
 * 3. Reads Data registers.
 * 4. Sensor automatically returns to SUSPEND mode (Hardware feature).
 *
 * @date 2026-02-15 23:27:05
 *   read 7 bytes from register 0 to avoid possible register sync issues
 *
 * @param[out] pData  Pointer to QMC5883P_data_t structure.
 * @return true if read successful, false if I2C error or device not ready.
 */
bool QMC5883P_read(struct QMC5883P_data *pData) {
  if (pData == NULL)
    return false;

  uint8_t status;
  uint8_t buffer[7]; // Buffer for ID, X(2), Y(2), Z(2)

  // Wait for DRDY
  if (!I2C_read(QMC5883P_I2C_ADDR, QMC5883P_REG_STATUS, &status, 1))
    return false;
  if (!(status & QMC5883P_STATUS_DRDY))
    return false;

  // 3. READ DATA REGISTERS
  // We read 7 bytes starting from 0x00 (REG_CHIPID).
  // This forces the internal pointer to align correctly,
  if (!I2C_read(QMC5883P_I2C_ADDR, QMC5883P_REG_CHIPID, buffer, 7))
    return false;

  // 4. PARSE DATA
  // Data is Little Endian (LSB at lower address).
  // Explicit casting ensures 16-bit reconstruction before signed interpretation.
  // Order: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB

  int16_t raw_x = (int16_t) (buffer[1] | ((uint16_t) buffer[2] << 8));
  int16_t raw_y = (int16_t) (buffer[3] | ((uint16_t) buffer[4] << 8));
  int16_t raw_z = (int16_t) (buffer[5] | ((uint16_t) buffer[6] << 8));

  // convert units to milli Gauss
  pData->x = (int32_t) raw_x * 1000 / QMC5883P_CONFIG_SENSITIVITY;
  pData->y = (int32_t) raw_y * 1000 / QMC5883P_CONFIG_SENSITIVITY;
  pData->z = (int32_t) raw_z * 1000 / QMC5883P_CONFIG_SENSITIVITY;

  // calculate heading on raw values as the are larger and more stable.
  pData->heading = MATH_atan2(raw_y, raw_x);

  return true;
}

////////////////////////////

#define BME280_ADDR         0x76 // Check your module: 0x76 (SDO=GND) or 0x77 (SDO=VCC)

// Registers
#define BME280_REG_DIG_T1   0x88 // Start of Temp Calibration
#define BME280_REG_ID       0xD0
#define BME280_REG_CTRL     0xF4 // Control Measurement
#define BME280_REG_TEMP     0xFA // Temp MSB
// Add Pressure Registers
#define BME280_REG_DIG_P1   0x8E
#define BME280_REG_PRESS    0xF7 // Pressure MSB
// Add Humidity Registers
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_HUM_MSB  0xFD

typedef struct {
  // Temp (0x88)
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  // Pressure (0x8E)
  uint16_t dig_P1;
  int16_t  dig_P2; int16_t  dig_P3; int16_t  dig_P4;
  int16_t  dig_P5; int16_t  dig_P6; int16_t  dig_P7;
  int16_t  dig_P8; int16_t  dig_P9;

  // Humidity (Fragmented)
  uint8_t  dig_H1; // 0xA1
  int16_t  dig_H2; // 0xE1
  uint8_t  dig_H3; // 0xE3
  int16_t  dig_H4; // 0xE4 + 0xE5[3:0]
  int16_t  dig_H5; // 0xE6 + 0xE5[7:4]
  int8_t   dig_H6; // 0xE7
} BME280_calib_t;

static BME280_calib_t calib;
static int32_t t_fine; // Needs to be static global now

void BME280_init() {
  uint8_t buffer[26];

  // --- 1. Load T/P Calibration (0x88 - 0x9F) ---
  I2C_read(BME280_ADDR, 0x88, buffer, 24);
  // ... [Parse T1-T3, P1-P9 as before] ...
  calib.dig_T1 = (uint16_t)((buffer[1] << 8) | buffer[0]);
  calib.dig_T2 = (int16_t)((buffer[3] << 8) | buffer[2]);
  calib.dig_T3 = (int16_t)((buffer[5] << 8) | buffer[4]);
  calib.dig_P1 = (uint16_t)((buffer[7] << 8) | buffer[6]);
  calib.dig_P2 = (int16_t)((buffer[9] << 8) | buffer[8]);
  calib.dig_P3 = (int16_t)((buffer[11]<< 8) | buffer[10]);
  calib.dig_P4 = (int16_t)((buffer[13]<< 8) | buffer[12]);
  calib.dig_P5 = (int16_t)((buffer[15]<< 8) | buffer[14]);
  calib.dig_P6 = (int16_t)((buffer[17]<< 8) | buffer[16]);
  calib.dig_P7 = (int16_t)((buffer[19]<< 8) | buffer[18]);
  calib.dig_P8 = (int16_t)((buffer[21]<< 8) | buffer[20]);
  calib.dig_P9 = (int16_t)((buffer[23]<< 8) | buffer[22]);

  // --- 2. Load Humidity Calibration (Fragmented) ---
  // H1 is alone at 0xA1
  I2C_read(BME280_ADDR, 0xA1, &calib.dig_H1, 1);

  // H2-H6 are at 0xE1 to 0xE7 (7 bytes)
  uint8_t h_buff[7];
  I2C_read(BME280_ADDR, 0xE1, h_buff, 7);

  calib.dig_H2 = (int16_t)((h_buff[1] << 8) | h_buff[0]); // 0xE1-E2
  calib.dig_H3 = h_buff[2];                               // 0xE3

  // H4 is 0xE4 (MSB) + 0xE5 (lower 4 bits)
  // FIX: Cast MSB to (int8_t) first to enforce sign extension
  calib.dig_H4 = (int16_t)((((int8_t)h_buff[3]) << 4) | (h_buff[4] & 0x0F));

  // H5 is 0xE6 (MSB) + 0xE5 (upper 4 bits)
  // FIX: Cast MSB to (int8_t) first
  calib.dig_H5 = (int16_t)((((int8_t)h_buff[5]) << 4) | (h_buff[4] >> 4));

  calib.dig_H6 = (int8_t)h_buff[6]; // 0xE7 (Signed char)

  // --- 3. Configure (Strict Order) ---

  // Step A: Set Humidity Oversampling (x1) to Reg 0xF2
  uint8_t ctrl_hum[] = { 0xF2, 0x01 };
  I2C_write(BME280_ADDR, ctrl_hum, 2);

  // Step B: Set Temp/Press Oversampling + Mode to Reg 0xF4
  // Note: Writing this register activates the changes in ctrl_hum
  uint8_t ctrl_meas[] = { 0xF4, 0x27 }; // Normal Mode, x1 Temp, x1 Press
  I2C_write(BME280_ADDR, ctrl_meas, 2);

  // Step C: Config (Standby 1000ms)
  uint8_t config[] = { 0xF5, 0xA0 };
  I2C_write(BME280_ADDR, config, 2);
}

void BME280_sleep() {
  // Register 0xF4 (ctrl_meas)
  // Mode 00 = Sleep
  // We write 0x00 to turn off Oversampling and Mode
  uint8_t data[] = { 0xF4, 0x00 };
  I2C_write(BME280_ADDR, data, 2);
}

void BME280_wake() {
  // We do NOT need to reload calibration.
  // Just restore the config to run a measurement.
  // Reg 0xF4: x1 Temp (001), x1 Press (001), Normal Mode (11) -> 0x27
  // Or Forced Mode (01) -> 0x25
  uint8_t data[] = { 0xF4, 0x27 };
  I2C_write(BME280_ADDR, data, 2);
}

int16_t BME280_read_temp() {
  uint8_t buffer[3];
  int32_t adc_T, var1, var2, T;

  // 1. Read Raw ADC (FA, FB, FC)
  I2C_read(BME280_ADDR, BME280_REG_TEMP, buffer, 3);

  // 2. Combine bytes (20-bit resolution)
  // MSB << 12 | LSB << 4 | XLSB >> 4
  adc_T = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4);

  // 3. Compensation Formula (Source: Bosch Datasheet)
  // This math is unavoidable. It maps the raw ADC to actual voltage/temp curves.

  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
	    ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
	  ((int32_t)calib.dig_T3)) >> 14;

  t_fine = var1 + var2; // t_fine carries the fine resolution temp for Pressure calc

  T = (t_fine * 5 + SSD1306_WIDTH) >> 8;

  return (int16_t)T; // Returns temperature in degC * 100
}

uint32_t BME280_read_pressure() {
  uint8_t buffer[3];
  int32_t adc_P, var1, var2;
  uint32_t p;

  // 1. Read Raw Pressure (F7, F8, F9)
  I2C_read(BME280_ADDR, BME280_REG_PRESS, buffer, 3);

  adc_P = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4);

  // 2. Compensation (Bosch 32-bit formula)
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;

  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)calib.dig_P6);
  var2 = var2 + ((var1 * ((int32_t)calib.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)calib.dig_P4) << 16);

  var1 = (((calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) +
	  ((((int32_t)calib.dig_P2) * var1) >> 1)) >> 18;

  var1 = ((((32768 + var1)) * ((int32_t)calib.dig_P1)) >> 15);

  // Division by zero protection (if P1 is empty/error)
  if (var1 == 0) {
    return 0;
  }

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t)var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }

  var1 = (((int32_t)calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)calib.dig_P8)) >> 13;

  p = (uint32_t)((int32_t)p + ((var1 + var2 + calib.dig_P7) >> 4));

  return p; // Returns Pressure in Pascals (e.g., 100000)
}

uint16_t BME280_read_humidity() {
  uint8_t buffer[2];
  int32_t adc_H;
  int32_t v_x1_u32r;

  // 1. Read Raw Humidity
  I2C_read(BME280_ADDR, BME280_REG_HUM_MSB, buffer, 2);
  adc_H = ((int32_t)buffer[0] << 8) | ((int32_t)buffer[1]);

  // 2. Compensation (Bosch Formula)
  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
		  (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	       (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
		    (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		  ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
			     ((int32_t)calib.dig_H1)) >> 4));

  // Clamp to 0..100% range in standard Q22 format
  if (v_x1_u32r < 0) v_x1_u32r = 0;
  if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;

  // 3. SCALING: Convert Q22.10 (1024=1%) to Centi-percent (100=1%)
  // Logic: (Value >> 12) gets us the 1024-scale.
  // Multiply by 25 and divide by 256 (>>8) converts 1024 -> 100.

  uint32_t raw_1024 = (uint32_t)(v_x1_u32r >> 12);
  uint32_t final_h = (raw_1024 * 25) >> 8;

  return (uint16_t)final_h; // Returns 0 to 10000
}

/* =========================================================================
 *                    COMPASS Bitmaps
 * ========================================================================= */

enum { COMPASS_WIDTH = 32, COMPASS_HEIGHT = 32 };

/**
 * Compass Bezel Bitmap 32x32 - Transposed for SSD1306 Page Addressing
 * 4 Pages (rows of 8px height), 32 Columns wide
 */
const uint8_t OLED_bezel_data[] PROGMEM = {
  // Page 0 (rows 0–7)
  0x00, 0x00, 0x80, 0x40,  0x20, 0x10, 0x08, 0x04,
  0x04, 0x02, 0x02, 0x02,  0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01,  0x02, 0x02, 0x02, 0x04,
  0x04, 0x08, 0x10, 0x20,  0x40, 0x80, 0x00, 0x00,

  // Page 1 (rows 8–15)
  0xf0, 0x0e, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x01, 0x0e, 0xf0,

  // Page 2 (rows 16–23)
  0x0f, 0x70, 0x80, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x80, 0x70, 0x0f,

  // Page 3 (rows 24–31)
  0x00, 0x00, 0x01, 0x02,  0x04, 0x08, 0x10, 0x20,
  0x20, 0x40, 0x40, 0x40,  0x80, 0x80, 0x80, 0x80,
  0x80, 0x80, 0x80, 0x80,  0x40, 0x40, 0x40, 0x20,
  0x20, 0x10, 0x08, 0x04,  0x02, 0x01, 0x00, 0x00,
};

/**
 * Compass Needle Bitmap 32x32 - Transposed for SSD1306 Page Addressing
 * 4 Pages (rows of 8px height), 32 Columns wide
 */
const uint8_t OLED_needle_data[] PROGMEM = {
  // Page 0 (rows 0–7)
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0xc0, 0xf0, 0xfc,
  0xfc, 0xf0, 0xc0, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,

  // Page 1 (rows 8–15)
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0xc0, 0xf0, 0xfc,  0xff, 0xff, 0x7f, 0xff,
  0xff, 0x7f, 0xff, 0xff,  0xfc, 0xf0, 0xc0, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,

  // Page 2 (rows 16–23)
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x03, 0x07, 0x03, 0x01,  0x00, 0x00, 0x00, 0xff,
  0xff, 0x00, 0x00, 0x00,  0x01, 0x03, 0x07, 0x03,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,

  // Page 3 (rows 24–31)
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x3f,
  0x3f, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
};

/**
 * @brief  Draws a rotated 32x32 compass needle using Inverse Texture Mapping.
 *
 * ================================================================================================
 * 1. ALGORITHM: INVERSE TEXTURE MAPPING
 * ================================================================================================
 * To rotate a bitmap, we do NOT iterate through the Source pixels and move them to the Destination.
 * Doing so results in "holes" (artifacts) because integer coordinates rarely map perfectly after rotation.
 *
 * Instead, we perform the INVERSE:
 * 1. We iterate through every pixel $(x,y)$ of the DESTINATION buffer (the screen).
 * 2. We mathematically project that point backwards onto the SOURCE bitmap $(u,v)$.
 * 3. If the projected $(u,v)$ lands on a lit pixel in the source, we light the pixel at $(x,y)$.
 *
 * ================================================================================================
 * 2. MATHEMATICS: AFFINE TRANSFORMATION MATRIX
 * ================================================================================================
 * The mapping from Destination $(x,y)$ to Source $(u,v)$ rotated by angle $\theta$ is:
 *
 *     u = center_u + (x - center_x) * cos(θ) - (y - center_y) * sin(θ)
 *     v = center_v + (x - center_x) * sin(θ) + (y - center_y) * cos(θ)
 *
 * ================================================================================================
 * 3. OPTIMIZATION: Q8.8 FIXED POINT MATH
 * ================================================================================================
 * The ATtiny85 has no FPU (Floating Point Unit). Calculating sines and cosines (0.0 to 1.0)
 * is too slow. We use "Q8.8 Fixed Point" integer math:
 * - 1.0 is represented as 256 (1 << 8).
 * - 0.5 is represented as 128.
 * - Coordinates store the integer part in the upper byte, fractional part in the lower byte.
 * - To get the integer pixel coordinate: `(value >> 8)`.
 *
 * ================================================================================================
 * 4. THE GRADIENTS (dUdX, dUdY, dVdX, dVdY)
 * ================================================================================================
 * Matrix multiplication requires 4 multiplications per pixel. This is too slow for the inner loop.
 * We use "Incremental Calculation" (similar to Bresenham's Line Algorithm).
 *
 * We pre-calculate how much $u$ and $v$ change when we move exactly 1 pixel on the screen.
 * Since we step through the screen linearly (Column by Column, Row by Row), we just ADD these values.
 *
 *   dUdX = cos(θ) * 256   -->  How much 'u' changes when Screen X increases by 1.
 *   dUdY = sin(θ) * 256   -->  How much 'v' changes when Screen X increases by 1.
 *   dVdX = -sin(θ) * 256  -->  How much 'u' changes when Screen Y increases by 1.
 *   dVdY = cos(θ) * 256   -->  How much 'v' changes when Screen Y increases by 1.
 *
 *   Inner Loop Logic:
 *   u_next = u_current + dUdX;
 *   v_next = v_current + dUdY;
 *
 * @param  page0  page for top-left corner compas (0-7)
 * @param  col0  column for top-left corner compas (0-1287)
 * @param  angle  Rotation angle in degrees (0-360).
 */
void OLED_draw_compass(uint8_t page0, uint8_t col0, int16_t angle) {
  // --- 2. TRIGONOMETRY (Q8.8 Base) ---
  // Fetch Sine/Cosine (Result -127 to +127)
  const int8_t isin = -MATH_sin(angle);
  const int8_t icos = MATH_cos(angle);

  // sinInt returns +/- 127. Shift << 1 makes it +/- 254 (approx 256 or 1.0).

  // --- 3. GRADIENTS (Delta accumulators) ---
  int16_t dUdX = icos << 1;    // Change in U per X step
  int16_t dVdX = isin << 1;    // Change in V per X step
  int16_t dUdY = (-isin) << 1; // Change in U per Y step
  int16_t dVdY = icos << 1;    // Change in V per Y step

  // --- 4. CALCULATE ORIGIN (Screen 0,0 maps to Source U,V) ---
  // Formula: Center_Original - Center_Rotated
  // Expanded: CX - (CX*cos - CY*sin), CY - (CX*sin + CY*cos)
  // We keep these in Q8.8 format (<< 8).

  // non-optimised
  // int32_t start_u = (CX * 256) - (CX * icos * 2 - CY * isin * 2);
  // int32_t start_v = (CY * 256) - (CX * isin * 2 + CY * icos * 2);

  // optimised
  int16_t start_u = (COMPASS_WIDTH << 7) - (COMPASS_WIDTH * icos - COMPASS_HEIGHT * isin);
  int16_t start_v = (COMPASS_HEIGHT << 7) - (COMPASS_WIDTH * isin + COMPASS_HEIGHT * icos);

  // --- 5. RENDER LOOP ---
  // Buffer: 1 Control Byte + 32 Data Bytes
  uint8_t i2c_buf[1 + COMPASS_WIDTH];
  i2c_buf[0] = SSD1306_CTRL_DATA;

  // Iterate 4 Pages (Vertical strips of 8 pixels)
  for (uint8_t page = 0; page < 4; page++) {

    // PRE-LOAD BACKGROUND (Optimization: Clears buffer + Draws Ring)
    const uint8_t *ring_ptr = &OLED_bezel_data[page * COMPASS_WIDTH];
    for (uint8_t col = 0; col < COMPASS_WIDTH; col++)
      i2c_buf[1 + col] = pgm_read_byte(ring_ptr++);

    // Initialize Row Accumulators for this Page
    int32_t row_u = start_u;
    int32_t row_v = start_v;

    // Iterate 8 Rows inside the Page
    for (uint8_t row = 0; row < 8; row++) {

      // Initialize Column Accumulators from Row Start
      int32_t u = row_u;
      int32_t v = row_v;

      // Tight Inner Loop: Iterate Columns 0..31
      for (uint8_t col = 0; col < COMPASS_WIDTH; col++) {

        // Downsample Fixed Point -> Integer (Fast Bounds Check)
        // Casting to uint16_t handles negative numbers by wrapping them to > 32
        uint16_t sx = (uint16_t)(u >> 8);
        uint16_t sy = (uint16_t)(v >> 8);

        // Check Bounds
        if (sx < COMPASS_WIDTH && sy < COMPASS_HEIGHT) {
          // Fetch Pixel (Bitmap is Page-Major: 8 rows per byte)
          // Index = (Row_Block * Width) + Col
          uint16_t idx  = ((sy >> 3) * COMPASS_WIDTH) + sx;
          uint8_t  bits = pgm_read_byte(&OLED_needle_data[idx]);

          // Check specific bit in the byte (y % 8)
          if (bits & (1 << (sy & 7))) {
            i2c_buf[1 + col] |= (1 << row);
          }
        }

        // X-STEP: Add Horizontal Gradients
        u += dUdX;
        v += dVdX;
      }

      // Y-STEP: Add Vertical Gradients (Move down 1 pixel)
      row_u += dUdY;
      row_v += dVdY;
    }

    // Advance the "Global Start" for the next Page (8 pixels down)
    // We multiply gradient by 8 (shift << 3)
    start_u += (dUdY << 3);
    start_v += (dVdY << 3);

    // Flush Page to I2C
    SSD1306_set_cursor(page0 + page, col0);
    I2C_write(SSD1306_I2C_ADDR, i2c_buf, COMPASS_WIDTH + 1);
  }
}

/* =========================================================================
 *                   TEXT RENDERING SUBSYSTEM
 * =========================================================================
 *
 * OVERVIEW:
 * ---------
 * This subsystem handles converting primitive data types (char, int, uint)
 * into visual pixels on the OLED.
 *
 * MEMORY OPTIMIZATION:
 * --------------------
 * - No Framebuffer: We write directly to the display hardware (GDDRAM).
 * - No Floating Point: Integer math only for digit extraction.
 * - Flash Storage: Fonts and Look-Up Tables (LUT) are stored in PROGMEM.
 *
 * I2C PERFORMANCE STRATEGY (STREAMING):
 * -------------------------------------
 * To prevent I2C Bus Thrashing (which can lock up sensors on the same bus),
 * we avoid "Stop/Start" sequences between every character.
 *
 * - Bad:  [Start '1' Stop] [Start '2' Stop] [Start '3' Stop]
 * - Good: [Start '1' '2' '3' Stop]
 *
 * The print functions perform a single I2C Transaction for the entire string.
 * =========================================================================
 */

/*
 * Powers of 10 Lookup Table.
 * Used for digit extraction by repeated subtraction.
 * Stored in Flash to save RAM.
 */
static const uint16_t pow10_LUT[] PROGMEM = {
    1, 10, 100, 1000, 10000
};

/*
 * 5x7 ASCII Font Table (0x20 Space to 0x7F Degree).
 * Stored in Flash (PROGMEM) to save RAM.
 */
const uint8_t OLED_font5x7_LUT[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00,  // 0x20 Space
  0x00, 0x00, 0x5F, 0x00, 0x00,  // 0x21 !
  0x00, 0x07, 0x00, 0x07, 0x00,  // 0x22 "
  0x14, 0x7F, 0x14, 0x7F, 0x14,  // 0x23 #
  0x24, 0x2A, 0x7F, 0x2A, 0x12,  // 0x24 $
  0x23, 0x13, 0x08, 0x64, 0x62,  // 0x25 %
  0x36, 0x49, 0x55, 0x22, 0x50,  // 0x26 &
  0x00, 0x05, 0x03, 0x00, 0x00,  // 0x27 '
  0x00, 0x1C, 0x22, 0x41, 0x00,  // 0x28 (
  0x00, 0x41, 0x22, 0x1C, 0x00,  // 0x29 )
  0x14, 0x08, 0x3E, 0x08, 0x14,  // 0x2A *
  0x08, 0x08, 0x3E, 0x08, 0x08,  // 0x2B +
  0x00, 0x50, 0x30, 0x00, 0x00,  // 0x2C ,
  0x08, 0x08, 0x08, 0x08, 0x08,  // 0x2D -
  0x00, 0x60, 0x60, 0x00, 0x00,  // 0x2E .
  0x20, 0x10, 0x08, 0x04, 0x02,  // 0x2F /
  0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0x30 0
  0x00, 0x42, 0x7F, 0x40, 0x00,  // 0x31 1
  0x42, 0x61, 0x51, 0x49, 0x46,  // 0x32 2
  0x21, 0x41, 0x45, 0x4B, 0x31,  // 0x33 3
  0x18, 0x14, 0x12, 0x7F, 0x10,  // 0x34 4
  0x27, 0x45, 0x45, 0x45, 0x39,  // 0x35 5
  0x3C, 0x4A, 0x49, 0x49, 0x30,  // 0x36 6
  0x01, 0x71, 0x09, 0x05, 0x03,  // 0x37 7
  0x36, 0x49, 0x49, 0x49, 0x36,  // 0x38 8
  0x06, 0x49, 0x49, 0x29, 0x1E,  // 0x39 9
  0x00, 0x36, 0x36, 0x00, 0x00,  // 0x3A :
  0x00, 0x56, 0x36, 0x00, 0x00,  // 0x3B ;
  0x08, 0x14, 0x22, 0x41, 0x00,  // 0x3C <
  0x14, 0x14, 0x14, 0x14, 0x14,  // 0x3D =
  0x00, 0x41, 0x22, 0x14, 0x08,  // 0x3E >
  0x02, 0x01, 0x51, 0x09, 0x06,  // 0x3F ?
  0x32, 0x49, 0x79, 0x41, 0x3E,  // 0x40 @
  0x7E, 0x11, 0x11, 0x11, 0x7E,  // 0x41 A
  0x7F, 0x49, 0x49, 0x49, 0x36,  // 0x42 B
  0x3E, 0x41, 0x41, 0x41, 0x22,  // 0x43 C
  0x7F, 0x41, 0x41, 0x22, 0x1C,  // 0x44 D
  0x7F, 0x49, 0x49, 0x49, 0x41,  // 0x45 E
  0x7F, 0x09, 0x09, 0x09, 0x01,  // 0x46 F
  0x3E, 0x41, 0x49, 0x49, 0x7A,  // 0x47 G
  0x7F, 0x08, 0x08, 0x08, 0x7F,  // 0x48 H
  0x00, 0x41, 0x7F, 0x41, 0x00,  // 0x49 I
  0x20, 0x40, 0x41, 0x3F, 0x01,  // 0x4A J
  0x7F, 0x08, 0x14, 0x22, 0x41,  // 0x4B K
  0x7F, 0x40, 0x40, 0x40, 0x40,  // 0x4C L
  0x7F, 0x02, 0x0C, 0x02, 0x7F,  // 0x4D M
  0x7F, 0x04, 0x08, 0x10, 0x7F,  // 0x4E N
  0x3E, 0x41, 0x41, 0x41, 0x3E,  // 0x4F O
  0x7F, 0x09, 0x09, 0x09, 0x06,  // 0x50 P
  0x3E, 0x41, 0x51, 0x21, 0x5E,  // 0x51 Q
  0x7F, 0x09, 0x19, 0x29, 0x46,  // 0x52 R
  0x46, 0x49, 0x49, 0x49, 0x31,  // 0x53 S
  0x01, 0x01, 0x7F, 0x01, 0x01,  // 0x54 T
  0x3F, 0x40, 0x40, 0x40, 0x3F,  // 0x55 U
  0x1F, 0x20, 0x40, 0x20, 0x1F,  // 0x56 V
  0x3F, 0x40, 0x38, 0x40, 0x3F,  // 0x57 W
  0x63, 0x14, 0x08, 0x14, 0x63,  // 0x58 X
  0x07, 0x08, 0x70, 0x08, 0x07,  // 0x59 Y
  0x61, 0x51, 0x49, 0x45, 0x43,  // 0x5A Z
  0x00, 0x7F, 0x41, 0x41, 0x00,  // 0x5B [
  0x02, 0x04, 0x08, 0x10, 0x20,  // 0x5C \ (Backslash)
  0x00, 0x41, 0x41, 0x7F, 0x00,  // 0x5D ]
  0x04, 0x02, 0x01, 0x02, 0x04,  // 0x5E ^
  0x80, 0x80, 0x80, 0x80, 0x80,  // 0x5F
  0x02, 0x01, 0x00, 0x00, 0x00,  // 0x60 ` (Backtick / Grave)
  0x20, 0x54, 0x54, 0x54, 0x78,  // 0x61 a
  0x7F, 0x48, 0x44, 0x44, 0x38,  // 0x62 b
  0x38, 0x44, 0x44, 0x44, 0x20,  // 0x63 c
  0x38, 0x44, 0x44, 0x48, 0x7F,  // 0x64 d
  0x38, 0x54, 0x54, 0x54, 0x18,  // 0x65 e
  0x08, 0x7E, 0x09, 0x01, 0x02,  // 0x66 f
  0x0C, 0x52, 0x52, 0x52, 0x3E,  // 0x67 g
  0x7F, 0x08, 0x04, 0x04, 0x78,  // 0x68 h
  0x00, 0x44, 0x7D, 0x40, 0x00,  // 0x69 i
  0x20, 0x40, 0x44, 0x3D, 0x00,  // 0x6A j
  0x7F, 0x10, 0x28, 0x44, 0x00,  // 0x6B k
  0x00, 0x41, 0x7F, 0x40, 0x00,  // 0x6C l
  0x7C, 0x04, 0x18, 0x04, 0x78,  // 0x6D m
  0x7C, 0x08, 0x04, 0x04, 0x78,  // 0x6E n
  0x38, 0x44, 0x44, 0x44, 0x38,  // 0x6F o
  0x7C, 0x14, 0x14, 0x14, 0x08,  // 0x70 p
  0x08, 0x14, 0x14, 0x18, 0x7C,  // 0x71 q
  0x7C, 0x08, 0x04, 0x04, 0x08,  // 0x72 r
  0x48, 0x54, 0x54, 0x54, 0x20,  // 0x73 s
  0x04, 0x3F, 0x44, 0x40, 0x20,  // 0x74 t
  0x3C, 0x40, 0x40, 0x20, 0x7C,  // 0x75 u
  0x1C, 0x20, 0x40, 0x20, 0x1C,  // 0x76 v
  0x3C, 0x40, 0x30, 0x40, 0x3C,  // 0x77 w
  0x44, 0x28, 0x10, 0x28, 0x44,  // 0x78 x
  0x0C, 0x50, 0x50, 0x50, 0x3C,  // 0x79 y
  0x44, 0x64, 0x54, 0x4C, 0x44,  // 0x7A z
  0x00, 0x08, 0x36, 0x41, 0x00,  // 0x7B {
  0x00, 0x00, 0x7F, 0x00, 0x00,  // 0x7C |
  0x00, 0x41, 0x36, 0x08, 0x00,  // 0x7D }
  0x10, 0x08, 0x08, 0x10, 0x08,  // 0x7E ~
  0x06, 0x09, 0x09, 0x06, 0x00,  // 0x7F ° (DEGREE SYMBOL)
};


/**
 * @brief  Sets cursor position in character size units
 * @param  page  Page (0-7).
 * @param  col   Column (0-21).
 */
void OLED_set_cursor(uint8_t page, uint8_t col) {
  // Ensure bounds
  if (page > 7) page = 7;
  if (col > 21) col = 21;

  // character glyph size is (5+1)*(7+1)
  SSD1306_set_cursor(page, col * 6);
}

/**
 * @brief  Streams pixel data for a character to the I2C Bus.
 * @note   ASSUMES I2C BUS IS ALREADY ACTIVE (Start Condition sent).
 *         Does NOT generate Start or Stop conditions.
 *
 * @param  c  ASCII character to draw.
 */
static void OLED_stream_char(char c) {
  const uint8_t *p;
  uint8_t       i;

  // Bounds check: Map unknown chars to '?'
  if (c < 0x20 || c > 0x7F) {
    c = '?';
  }

  // Convert ASCII to Font Table Index (Offset 0x20)
  c -= 0x20;

  // Calculate pointer to font data in Flash
  // Math: Base Address + (Index * 5 bytes/char)
  p = OLED_font5x7_LUT + ((uint16_t) c * 5);

  // Stream 5 columns of pixel data
  for (i = 0U; i < 5U; i++) {
    I2C_transmit(pgm_read_byte(p));
    p++;
  }

  // Stream 1 column of spacing (Kerning)
  I2C_transmit(0x00);
}

/**
 * @brief  Prints a single character to the OLED.
 * @details Performs a standalone I2C transaction.
 *
 * @param  c  Character to print.
 */
void OLED_char(char c) {
  // 1. Start Transaction
  I2C_start(SSD1306_I2C_ADDR, false);

  // 2. Control Byte: Data Stream (0x40)
  I2C_transmit(SSD1306_CTRL_DATA);

  // 3. Stream Data
  OLED_stream_char(c);

  // 4. Stop Transaction
  I2C_stop();
}

/**
 * @brief  Prints a string to the OLED.
 * @details Performs a standalone I2C transaction.
 *
 * @param  str  Pointer to the null-terminated string in RAM.
 */
void OLED_string(const char *str) {
  // 1. Start Transaction
  I2C_start(SSD1306_I2C_ADDR, false);

  // 2. Control Byte: Data Stream (0x40)
  I2C_transmit(SSD1306_CTRL_DATA);

  // 3. Stream Data
  while (*str)
    OLED_stream_char(*str++);

  // 4. Stop Transaction
  I2C_stop();
}

/**
 * @brief  Prints an unsigned 16-bit integer (0..65535).
 * @details Optimized to use a single I2C transaction for the whole number.
 *
 * @param  num    Number to print.
 * @param  width  Minimum width (0 = auto). Pads with leading spaces.
 */
void OLED_print_uint(uint16_t num, int8_t width = 0) {
  // 1. Start Transaction
  I2C_start(SSD1306_I2C_ADDR, false);
  I2C_transmit(SSD1306_CTRL_DATA); // Data Mode

  // 2. Digit Extraction Loop (10000 down to 10)
  for (int8_t round = 4; round > 0; --round) {
    char     ch  = '0';
    uint16_t pow = pgm_read_word(&pow10_LUT[round]);

    // Subtractive Division (Faster than hardware DIV on ATtiny)
    while (num >= pow) {
      num -= pow;
      ch++;
    }

    // Rendering Logic
    if (ch != '0' || width == 0) {
      // Significant digit found, or we are in "print all" mode
      OLED_stream_char(ch);
      width = 0; // Disable padding once we print a digit
    } else if (round < width) {
      // We are within the fixed width, but digit is 0
      // Print leading space for alignment
      OLED_stream_char(' ');
    }
  }

  // 3. Final Digit (Ones place)
  // Always printed, even if 0
  OLED_stream_char('0' + (uint8_t) num);

  // 4. Stop Transaction //
  I2C_stop();
}

/**
 * @brief  Prints an unsigned 16-bit integer (0..65535).
 * @details Optimized to use a single I2C transaction for the whole number.
 *
 * @param  num    Number to print.
 * @param  width  Minimum width (0 = auto). Pads with leading spaces.
 */
void OLED_sprint_uint(char *str, uint16_t num, int8_t width = 0) {
    // 2. Digit Extraction Loop (10000 down to 10)
    for (int8_t round = 4; round > 0; --round) {
        char     ch  = '0';
        uint16_t pow = pgm_read_word(&pow10_LUT[round]);

        // Subtractive Division (Faster than hardware DIV on ATtiny)
        while (num >= pow) {
            num -= pow;
            ch++;
        }

        // Rendering Logic
        if (ch != '0' || width == 0) {
            // Significant digit found, or we are in "print all" mode
            *str++ = ch;
            width = 0; // Disable padding once we print a digit
        } else if (round < width) {
            // We are within the fixed width, but digit is 0
            // Print leading space for alignment
            *str++ = ' ';
        }
    }

    // 3. Final Digit (Ones place)
    // Always printed, even if 0
    *str++ = '0' + (uint8_t) num;

    // 4. Terminate string
    *str = 0;
}

/**
 * @brief  Prints a signed 16-bit integer (-32768..32767).
 * @details Handles negative signs and padding correctly within the stream.
 *
 * @param  num0   Signed number to print.
 * @param  width  Minimum width. Minus sign consumes 1 character width.
 */
void OLED_print_int(int16_t num0, int8_t width = 0) {
  uint16_t num;
  char     minus = 0;

  // 1. Handle Sign
  if (num0 < 0) {
    // Convert to unsigned magnitude
    // Note: -32768 converts to 32768 (0x8000), which is valid for uint16_t
    num   = (uint16_t)(-num0);
    minus = '-';
    width--; // Decrease available width padding
  } else {
    num = (uint16_t) num0;
  }

  // 2. Start Transaction
  I2C_start(SSD1306_I2C_ADDR, false);
  I2C_transmit(SSD1306_CTRL_DATA); // Data Mode

  // 3. Digit Extraction Loop
  for (int8_t round = 4; round > 0; --round) {
    char     ch  = '0';
    uint16_t pow = pgm_read_word(&pow10_LUT[round]);

    // Subtractive Division (Faster than hardware DIV on ATtiny)
    while (num >= pow) {
      num -= pow;
      ch++;
    }

    // Rendering Logic
    if (ch != '0' || width == 0) {
      // Check if we have a pending minus sign to print first
      if (minus != 0) {
        OLED_stream_char(minus);
        minus = 0; // Clear flag so we don't print it again
      }

      OLED_stream_char(ch);
      width = 0; // no more leading spaces
    } else if (round < width) {
      // Leading Space Padding
      OLED_stream_char(' ');
    }
  }

  // 4. Final Digit & Final Minus Check
  // If the number was just "-0" (impossible) or "-5" where loops skipped:
  if (minus != 0)
    OLED_stream_char(minus);
  OLED_stream_char('0' + (uint8_t) num);

  // 5. Stop Transaction
  I2C_stop();
}

// Draws a string at 2x scale (10x14 pixels)
// Consumes 2 Pages of height.
void OLED_print_big(uint8_t page, uint8_t col, const char* str) {
  while (*str) {
    // Check bounds
    if (col > 118) break;

    // Get standard 5x7 char data
    uint8_t c = *str - 32;
    uint16_t font_idx = c * 5;

    // --- DRAW TOP HALF (Page N) ---
    OLED_set_cursor(page, col);
    I2C_start(SSD1306_I2C_ADDR, false);
    I2C_transmit(0x40);
    for (uint8_t i = 0; i < 5; i++) {
      uint8_t line = pgm_read_byte(&OLED_font5x7_LUT[font_idx + i]);
      // Stretch Lower Nibble (0000ABCD -> AABBCCDD)
      uint8_t expanded = 0;
      if (line & 0x01) expanded |= 0x03;
      if (line & 0x02) expanded |= 0x0C;
      if (line & 0x04) expanded |= 0x30;
      if (line & 0x08) expanded |= 0xC0;
      // Write twice for width doubling
      I2C_transmit(expanded);
      I2C_transmit(expanded);
    }
    I2C_transmit(0x00); I2C_transmit(0x00); // Spacing
    I2C_stop();

    // --- DRAW BOTTOM HALF (Page N+1) ---
    OLED_set_cursor(page + 1, col);
    I2C_start(SSD1306_I2C_ADDR, false);
    I2C_transmit(0x40);
    for (uint8_t i = 0; i < 5; i++) {
      uint8_t line = pgm_read_byte(&OLED_font5x7_LUT[font_idx + i]);
      // Stretch Upper Nibble (0000EFGH -> EEFFGGHH)
      uint8_t expanded = 0;
      if (line & 0x10) expanded |= 0x03;
      if (line & 0x20) expanded |= 0x0C;
      if (line & 0x40) expanded |= 0x30;
      if (line & 0x80) expanded |= 0xC0;
      I2C_transmit(expanded);
      I2C_transmit(expanded);
    }
    I2C_transmit(0x00); I2C_transmit(0x00);
    I2C_stop();

    col += 2; // big glyph is 2 chars wide
    str++;
  }
}

// =================================================================================
// === YOUR CONTROLS - THE ONLY PART YOU NEED TO CHANGE! ===========================
// =================================================================================

/*
================================================================================
ATtiny85 v2 Pin Mapping — Multiplexed Design
================================================================================
Physical Pin | Port/Bit | Mode / Usage           | Attached Device / Notes
-------------|----------|------------------------|------------------------
1            | PB5      | RESET                  | ISP reset; standard programming
2            | PB3      | OUTPUT                 | Piezo (+), differential drive with PB4
3            | PB4      | OUTPUT                 | Piezo (–), complementary to PB3
4            | GND      | —                      | Ground
5            | PB0      | I2C SDA                | I2C bus, pull-ups recommended
6            | PB1      | MULTIPLEXED            | LED drive, Switch read, ISP MISO
7            | PB2      | I2C SCL                | I2C bus, pull-ups recommended
8            | VCC      | —                      | Power 3.3–5 V
================================================================================

PB1 (Physical 6) Multiplexed Logic:
Circuit: VCC - R1 (2K) - LED - pin6 - SWITCH - R2 (1K) - GND

Mode                     | PinMode     | Physics / Current Path                  | Function
-------------------------|-------------|-----------------------------------------|-------------------------
Programming / Idle       | INPUT       | Pin floats HIGH; programmer drives LOW  | ISP MISO
Switch Released          | INPUT       | VCC - R1 - LED - pin6                   | pin6 pullup, Detect VCC - HIGH
Switch Pressed           | INPUT       | VCC - R1 - LED - pin6 - R2 - GND        | Detect R2/(R1+R2) - LOW
LED ON                   | OUTPUT LOW  | VCC - R1 - LED - pin6=GND - R2 - GND    | LED lights up, pin6 sinks current
LED OFF                  | OUTPUT HIGH | VCC - R1 - LED - pin6=VCC - R2 → GND    | LED off, pin6 sources current
================================================================================================================
Notes:
- Piezo differential drive doubles voltage swing without extra hardware.
- I2C lines (PB0/PB2) must be idle during programming.
- R1/R2 + LED forward voltage maintain safe logic levels at 3.3–5 V.
- Dynamic pinMode management allows safe multiplexing between LED, switch, and programming.
*/

const byte PIEZO_PLUS_PIN = PB3;  // chip pin 2
const byte PIEZO_MINUS_PIN = PB4; // chip pin 3
const byte LED_PIN = PB1;         // chip pin 6
const byte SWITCH_PIN = PB1;      // chip pin 6
const byte SDA_PIN = PB0;         // chip pin 5
const byte SCL_PIN = PB2;         // chip pin 7

// --- EFFECT "SEASONING" ---
// These are the fun numbers you can change to be a "sound designer"!
// Try changing them and see what happens to the final effect.
const int cfg_modeTimeout          = 300;      // Short press within this time activate flashlight
const int cfg_flashCycle           = 10;       // flash cycle time to sense deactivate
const int cfg_throbHalfCycle       = 2000 / 2; // How fast the LED cycles
const int cfg_startFrequency       = 800;      // The starting pitch of the sound (in Hertz).
const int cfg_endFrequency         = 1800;     // The ending pitch of the sound.
const int cfg_sweepDuration        = 700;      // How many milliseconds the sound takes to slide up.
const int cfg_warbleDepth          = 30;       // How "wobbly" the sound is. Try 10 for subtle, 100 for crazy!
const int cfg_warbleSpeed          = 25;       // How fast the wobble is. Lower numbers are faster!

/*
 * =========================================================================================
 * SOUND SUBSYSTEM
 * =========================================================================================
 */

//
// DEEP DIVE: HOW THE SONIC SCREWDRIVER SOUND WORKS
//
//
// The function below, updateSonicWarble(), is the heart of our sound effect.
// It's a fantastic example of how you can create complex results from simple tools.
//
// --- The First Tool: The map() Function ---
//
// The map() function is like a magical stretching machine. It takes a number from
// one range and perfectly scales it to fit into a new range.
//
// The command looks like this:
//   newNumber = map(inputValue, fromLow, fromHigh, toLow, toHigh);
//
// For example, if you have a value of 50 in a range of 0-100, and you want to map
// it to a new range of 0-1000, map() would give you 500. It's a translator for numbers.
//
// --- The Second Tool: The tone() Function ---
//
// The tone() function is like a musician who lives inside the chip. You tell it:
//   tone(pin, frequency);
// ...and it will start playing that exact musical note (frequency) on that pin.
// The best part? It plays in the background! Your main loop() code can keep running
// and doing other things while the musician plays their note.
// To stop the sound, you have to command it with noTone(pin);
//
// --- The Recipe: Layering Two "Songs" ---
//
// Our code uses map() twice to create two different "songs" at the same time.
//
//  SONG #1: THE SWEEP
//  This is a slow, single slide up in pitch. It's a "single-cycle sawtooth wave".
//  The code for it is:
//    baseFrequency = map(timeSinceStart, 0, 700, 800, 1800);
//  It takes the time (from 0 to 700ms) and translates it to a frequency
//  (from 800Hz to 1800Hz), creating a smooth upward slide.
//
//  Frequency ^
//            |         /-----------
//            |        /
//            |       /
//            |      /
//   StartFreq +-----/
//            |
//            +----------------> Time (0 to 700ms)
//
//  SONG #2: THE WARBLE
//  This is a fast, repeating "wobble" that rides on top of the sweep.
//  The code for it is:
//    warble = map(timeSinceStart % 25, 0, 25, -30, 30);
//  The "%" is a math trick that creates a counter that repeats very quickly (0-24).
//  We map this fast counter to a range of -30 to +30. This creates the wobble.
//
//  Offset ^
//         |   /\  /\  /\  /\  /
//         |  /  \/  \/  \/  \/
//         | /
//         +/----------------> Time
//         |/
//         |\  /\  /\  /\  /\  /
//         | \/  \/  \/  \/  \/
//
// THE FINAL PERFORMANCE:
// The last line of the function, `tone(SPEAKER_PIN, baseFrequency + warble);`,
// simply ADDS the two songs together! The fast wobble rides on top of the slow
// sweep, and sends the final, complex note to the speaker.
//
// =================================================================================

ISR(TIMER1_COMPA_vect) {
  if (PORTB & (1<<PIEZO_MINUS_PIN)) {
    // PB4 low, PB3 high
    PORTB = (PORTB & ~(1<<PIEZO_MINUS_PIN)) | (1<<PIEZO_PLUS_PIN);
  } else {
    // PB4 high, PB3 low
    PORTB = (PORTB & ~(1<<PIEZO_PLUS_PIN)) | (1<<PIEZO_MINUS_PIN);
  }
}

void sonic_Init() {
  cli();

  /* --- GPIO setup --- */
  DDRB |= (1 << PIEZO_PLUS_PIN) | (1 << PIEZO_MINUS_PIN);

  // Start in opposite phase
  PORTB |=  (1 << PIEZO_MINUS_PIN);
  PORTB &= ~(1 << PIEZO_PLUS_PIN);

  /* --- Stop Timer1 --- */
  TCCR1 = 0;
  GTCCR = 0;
  TCNT1 = 0;

  /* --- CTC mode --- */
  TCCR1 = (1 << CTC1);

  /* --- Hardware toggle on PB4 (OC1B) --- */
  // remove because it is now software driven
  // GTCCR |= (1 << COM1B0);

  /* Clear stale compare flag before enabling ISR */
  TIFR |= (1 << OCF1A);

  /* --- Enable ISR for PB3 toggle --- */
  TIMSK |= (1 << OCIE1A);

  sei();
}

void sonic_tone(uint16_t freq) {
  if (freq == 0) return;

  cli();

  /* --- Frequency calculation --- */
  uint32_t ocr = (uint32_t)F_CPU / (freq * 2);
  uint8_t prescaler = 0b0001;   // clk/1

  while (ocr > 0xFF && prescaler < 15) {
    prescaler++;
    ocr >>= 1;
  }

  // 1. Reset Counter.
  // Crucial to restart the wave phase from 0 immediately.
  TCNT1 = 0;

  // 2. Set Match Value
  OCR1C = (uint8_t)(ocr - 1);   // TOP
  OCR1A = OCR1C;                // Compare A @ TOP

  // 3. Clear any pending interrupt flags from previous tones.
  // If we don't do this, an old flag might fire the ISR immediately
  // with the wrong timing.
  TIFR |= (1 << OCF1A);

  /* --- Start Timer --- */
  // We combine CTC1 mode and Prescaler in one write.
  // This overwrites the previous state, effectively starting the timer.
  TCCR1 = (1 << CTC1) | (prescaler);

  sei();
}

void sonic_noInt() {
  cli();

  /* --- Disable interrupt first --- */
  TIMSK &= ~(1 << OCIE1A);

  /* --- Stop Timer1 clock --- */
  TCCR1 = 0; // &= ~0x0F;   // clear CS13..CS10

  /* --- Disconnect all timer outputs (safety) --- */
  GTCCR = 0;

  /* Clear pending flags */
  TIFR |= (1 << OCF1A);

  /* --- Reset counter (optional but clean) --- */
  TCNT1 = 0;

  /* --- Drive outputs low --- */
  PORTB &= ~((1 << PIEZO_PLUS_PIN) | (1 << PIEZO_MINUS_PIN));

  sei();
}

// This function creates the Doctor Who WARBLE sound.
void update_sonic_warble(long durationSinceStart) {
  // --- Create SONG #1: The Sweep ---+
  long baseFrequency;
  if (durationSinceStart >= cfg_sweepDuration) {
    // If the sweep time is over, just hold the pitch at the end frequency.
    baseFrequency = cfg_endFrequency;
  } else {
    // Otherwise, use map() to calculate the current pitch of the upward slide.
    baseFrequency = map(durationSinceStart, 0, cfg_sweepDuration, cfg_startFrequency, cfg_endFrequency);
  }

  // --- Create SONG #2: The Warble ---
  // Use the fast-repeating counter and map() to create the wobble.
  int warble = map(durationSinceStart % cfg_warbleSpeed, 0, cfg_warbleSpeed, -cfg_warbleDepth, cfg_warbleDepth);

  // --- THE FINAL PERFORMANCE ---
  // Add the two songs together and tell the 'musician' to play the new note.
  sonic_tone(baseFrequency + warble);
}

/*
 * =========================================================================================
 * LED SUBSYSTEM (SOFTWARE PWM via TIMER0)
 * =========================================================================================
 *
 * WHY TIMER0?
 *   The ATtiny85 only has two timers: Timer0 and Timer1.
 *   - Timer1 is being used by the Sound Subsystem for high-speed frequency generation.
 *   - Timer0 is used by the Arduino core for timekeeping (millis(), delay()).
 *
 *   Since we cannot steal Timer0 entirely without breaking timekeeping functions,
 *   we must "piggyback" on it. We enable the "Compare Match B" interrupt, which
 *   fires in parallel with the standard timekeeping interrupt.
 *
 * FREQUENCY CALCULATION:
 *   - CPU Clock: 8 MHz
 *   - Timer0 Prescaler: 64 (Standard Arduino setting)
 *   - Timer0 Bit Depth: 8-bit (Counts 0 to 255)
 *
 *   Timer Tick Frequency = 8,000,000 / 64 = 125,000 Hz
 *   Overflow Frequency   = 125,000 / 256   = 488 Hz
 *
 *   To achieve a full 0-255 brightness range via software without jitter, we use a
 *   "Two-Cycle Latching" strategy. One cycle handles the ON pulse, the next handles
 *   the OFF pulse.
 *
 *   Effective LED Frequency = 488 Hz / 2 = 244 Hz (Flicker-free).
 *
 * THE ENGINEERING PROBLEM: "DOUBLE BUFFERING" IN FAST PWM MODE:
 *
 * We are piggybacking on Timer0, which is configured for Fast PWM to support millis().
 * In Fast PWM mode, the Output Compare Register (OCR0B) is DOUBLE BUFFERED.
 *
 * This means:
 * 1. When we write to OCR0B, the hardware does NOT update the comparator immediately.
 * 2. It saves the value in a temporary buffer.
 * 3. The actual update happens only when the Timer Overflows (reaches TOP/255).
 *
 * WHY STANDARD SOFTWARE PWM FAILS:
 * A standard approach uses one cycle:
 *   - Start at 0: Turn LED ON.
 *   - Interrupt at 'Duty': Turn LED OFF.
 *   - *Goal*: Set next interrupt for '255' (End of Cycle) to reset.
 *
 * *Failure Mode*: Because of Double Buffering, if we try to set the interrupt target
 * to '255' while inside the 'Duty' interrupt, the hardware waits until the NEXT
 * overflow to apply it. We effectively lose control of the timer for the remainder
 * of the cycle, causing severe jitter and limiting brightness to 50%.
 *
 * THE SOLUTION: "TWO-CYCLE LATCHING" STRATEGY:
 *
 * To bypass the buffering limitation, we spread one logic PWM period over TWO
 * hardware timer overflows. We alternate between two phases:
 *
 *   PHASE 0 (The ON Cycle):
 *   - We start with LED ON.
 *   - We set OCR0B = 'Duty'.
 *   - When interrupt fires: We Turn LED OFF.
 *     (The timer continues running to 255, but we don't care, LED is off).
 *
 *   PHASE 1 (The OFF Cycle):
 *   - We start with LED OFF.
 *   - We set OCR0B = '255 - Duty' (The remaining time needed to complete the period).
 *   - When interrupt fires: We Turn LED ON.
 *     (The timer continues running to 255, but we don't care, LED is on).
 *
 * MULTIPLEXING & BIT-BANGING:
 *   Hardware PWM (OC0B pin) cannot be used directly because PB1 is physically shared
 *   with the Switch input and the ISP Programmer (MISO).
 *   - If we let hardware drive the pin automatically, it might interfere with reading
 *     the switch.
 *   - Instead, we "Bit-Bang" (manually toggle) the pin inside the Interrupt Service
 *     Routine (ISR). This gives us code-level control to disable the LED drive
 *     logic instantly if we need to read the switch or enter sleep mode.
 */

// --- Globals ---
/**
 * The requested brightness level from the main loop.
 * Range: 0 (Off) to 255 (Max Brightness).
 * Marked 'volatile' because it is shared between the main loop (writer) and ISR (reader).
 */
volatile uint8_t ledBrightness = 255; // OFF

/**
 * Tracks the current phase of the software PWM cycle.
 * 0 = Phase A (The "Active" Phase)
 * 1 = Phase B (The "Rest" Phase)
 */
volatile uint8_t ledPwmPhase = 0;

// --- Logic ---

/**
 * @brief  Initializes the LED subsystem without disturbing Timer0's millis() settings.
 * @return void
 */
void led_init() {
  // 2. Set default State: HIGH (LED OFF for Active Low)
  // Since the LED is wired Active Low (VCC -> Resistor -> LED -> Pin),
  //    setting the pin HIGH turns the LED OFF.
  PORTB |= (1 << LED_PIN);

  // 3. Enable Timer0 Compare Match B Interrupt.
  // We keep this running continuously to ensure smooth transitions.
  // Even if brightness is 0, the ISR keeps running to latch new values seamlessly.
  TIMSK |= (1 << OCIE0B);

  // 4. Initial Trigger point.
  // Set a safe middle value to start the comparator logic.
  OCR0B = 128;
}

/**
 * Disable led interrupts
 */
void led_noInt() {
  cli();  // Disable global interrupts

  /* --- Disable COMPB interrupt --- */
  TIMSK &= ~(1 << OCIE0B);

  /* --- Clear pending COMPB flags --- */
  TIFR |= (1 << OCF0B);

  /* --- Optional: reset OCR0B counter (clean) --- */
  OCR0B = 0;

  sei();  // Re-enable global interrupts
}

/**
 * @brief  Timer0 Compare Match B Interrupt Service Routine.
 * @desc   Handles the manual pin toggling for Software PWM.
 *         Runs at approx 488 Hz.
 *         Implements a "Phase-Correct" strategy to allow full 0-255 range.
 */
ISR(TIMER0_COMPB_vect) {
  // Static variable preserves value between interrupt calls.
  // This holds the brightness value constant for the duration of one full ON/OFF cycle.
  static uint8_t latched_duty = 0;

  // handle edge cases
  if (ledBrightness < 2) {
    // forced off
    PORTB |= (1 << LED_PIN);
    return;
  } else if (ledBrightness > 253) {
    // forced on
    PORTB &= ~(1 << LED_PIN);
    return;
  }

  if (ledPwmPhase == 0) {
    // =========================================================
    // PHASE A: END OF "ON" PHASE -> STARTING "OFF" PHASE
    // =========================================================

    // 1. Turn LED OFF.
    // Active Low logic: HIGH = OFF.
    // We always force this state here to ensure clean square waves.
    // Note: bitwise OR preserves other pins (like Piezo on PB3/PB4).
    PORTB |= (1 << LED_PIN);

    // 2. Calculate time to stay OFF
    // We want to fill the rest of the 255-tick window.
    // Note: We use the duty we latched at the START of the cycle.
    OCR0B = 255 - latched_duty;

    // 3. Prepare for next phase.
    ledPwmPhase = 1;

  } else {
    // =========================================================
    // PHASE B: END OF "OFF" PHASE -> STARTING "ON" PHASE
    // =========================================================

    // 1. LATCH NEW DATA.
    // This is the only safe moment to update, ensuring no visual glitches.
    // This is the synchronization point. We grab the freshest value
    // from the main loop now, before starting a new visual pulse.
    latched_duty = ledBrightness;

    // 2. Turn LED ON (Set LOW for Active Low)
    // Only if brightness is > 0 (otherwise stay OFF)
    if (latched_duty > 0) {
      PORTB &= ~(1 << LED_PIN);
    }

    // 3. Calculate time to stay ON
    // The interrupt will fire again after 'latched_duty' ticks.
    OCR0B = latched_duty;

    // 4. Prepare for next phase.
    ledPwmPhase = 0;
  }
}

/**
 * @brief  Calculates a throbbing brightness effect based on time.
 * @param  durationSinceStart  Milliseconds elapsed since effect started.
 */
void update_led_throb(long durationSinceStart) {
  // Calculate angle (0-360 degrees) based on cycle duration
  long angle = (durationSinceStart * 180) / cfg_throbHalfCycle;

  // Use sin_deg/cos_deg or pre-calculated table (assumed available in project)
  // +127 shifts the -127..+127 sine wave to 0..254 range.
  // The result is passed to our safe setter function.
  // Add 270 degrees so phase starts at angle(0) -> 0 (OFF)
  ledBrightness = MATH_sin(angle + 90) + 127;  // returns -127..127
}

/*
================================================================================
  ||                                                                       ||
  ||                    PROJECT: ATtiny85 "Sonic Screwdriver"              ||
  ||          A Beginner's Guide to Lights, Sound, and Microcontrollers    ||
  ||                                                                       ||
================================================================================

  AUTHOR: Your Name Here (with an AI Sparring Partner)
  DATE: Today's Date

  --- THE BIG IDEA (THE "BRIDGE") ---

  Welcome! This project is a recipe for turning a tiny computer chip, the ATtiny85,
  into a fun sci-fi gadget. When you press a button, it will create a cool,
  pulsing light and a complex "warbling" sound effect, just like a sonic
  screwdriver from Doctor Who.

  This single file contains everything you need: the wiring guide, the fully
  explained code, and a deep dive into the "magic" that makes it all work.
  Read the comments carefully; they are the lesson!

  --- A VERY IMPORTANT NOTE ON PROGRAMMING (THE "RULES OF THE ROAD") ---

  Our project is so advanced that it uses special pins on the chip that are ALSO
  used for programming! This is like using the mechanic's service port on your
  car as the place to plug in your radio. It works, but you have to be careful.

  CRITICAL OPERATIONAL WARNING:
  To make this project work, you MUST NOT try to upload new code to the chip
  while the push button is being held down. The chip will be "busy" using the
  programming pins, and the upload will fail. Always make sure the button is
  released before you try to program the chip.

*/

/*
================================================================================
Arduino as ISP — ATtiny85 Low-Voltage Programming Tweak
================================================================================
Background:
------------
When programming an ATtiny85 powered at 3.3 V using "Arduino as ISP", uploads may
fail due to the Arduino's 5 V SPI signals being too fast for the slower, low-voltage
ATtiny clock. Typical errors include verification mismatches or "programmer not responding".

Solution:
---------
Modify the "Arduino as ISP" sketch to slow down the SPI clock:

    // Configure SPI clock (in Hz)
    // Must be slow enough for ATtiny @ 1 MHz (or low voltage)
    #define SPI_CLOCK (1000000 / 20)

Explanation:
------------
- The ATtiny85 datasheet specifies that SPI clock pulse width must exceed 2 CPU cycles.
- Using f_CPU / 6 gives a safe margin for low-voltage operation (3.3 V) and slow clock.
- This tweak prevents upload errors like:
      avrdude: verification error, first mismatch at byte 0x0006
      avrdude: programmer is not responding

Implementation:
---------------
1. Open "ArduinoISP" sketch in Arduino IDE.
2. Add or replace the line for SPI_CLOCK as shown above.
3. Upload the sketch to the Arduino board used as programmer.
4. Program the ATtiny85 at 3.3 V VCC.

Notes / Safety:
---------------
- No external level shifters are needed.
- Always release the ATtiny's push-button or multiplexed pins during programming.
- This change only affects the Arduino ISP programmer; your main ATtiny sketch
  does not require modification.
================================================================================
*/

// =================================================================================
// === Screen handler
// =================================================================================

/**
 * @date 2026-02-14 17:39:24
 * This function van take upto 130 mSec which is noticably effecting sound and light effects.
 * Interleave to give sound updates more chance.
 *
 */
void update_ui() {
  static int interleave = 0;
  static struct QMC5883P_data qmc_data;

  switch (interleave) {
    case 0:
      if (!QMC5883P_read(&qmc_data)) {
        interleave = 5;
        return;
      }

      /*
       * @date 2026-02-16 02:16:23
       * For this device the compass is upside down
       */
      qmc_data.heading = 359 - qmc_data.heading;

      OLED_set_cursor(0, 14);
      OLED_string("X");
      OLED_print_int(qmc_data.x, 4);
      OLED_string("mG");
      interleave++;
      return;
    case 1:
      OLED_set_cursor(1, 14);
      OLED_string("Y");
      OLED_print_int(qmc_data.y, 4);
      OLED_string("mG");
      interleave++;
      return;
    case 2:
      OLED_set_cursor(2, 14);
      OLED_string("Z");
      OLED_print_int(qmc_data.z, 4);
      OLED_string("mG");
      interleave++;
      return;
    case 3: {
      char strbuf[10];
      OLED_sprint_uint(strbuf, qmc_data.heading, 3);
      OLED_print_big(1, 6, strbuf);
      OLED_print_big(1, 6 + 3 * 2, "\x7f"); //  above is 3 double cell wide
      interleave++;
      return;
    }
    case 4:
      OLED_draw_compass(0, 0, qmc_data.heading);
      interleave++;
      return;
    case 5: {
      int16_t temp = BME280_read_temp();
      OLED_set_cursor(4, 0);
      OLED_string("Temp:");
      OLED_print_int(temp / 100, 4);
      OLED_string(".");
      OLED_print_int(temp % 100, 2);
      OLED_string(" \x7f" "C"); // split ° and C
      interleave++;
      return;
    }
    case 6: {
      uint32_t p_pa = BME280_read_pressure(); // e.g. 101325 (Pa)
      OLED_set_cursor(5, 0);
      OLED_string("Baro:");
      OLED_print_int(p_pa / 100, 4);
      OLED_string(".");
      OLED_print_int(p_pa % 100, 2);
      OLED_string(" hPa");
      interleave++;
      return;
    }
    case 7: {
      int16_t hum = BME280_read_humidity();
      OLED_set_cursor(6, 0);
      OLED_string("Hum :");
      OLED_print_int(hum / 100, 4);
      OLED_string(".");
      OLED_print_int(hum % 100, 2);
      OLED_string(" %");
      interleave++;
      return;
    }
    case 8: {
      int16_t bat = ADC_get_voltage();
      OLED_set_cursor(7, 0);
      OLED_string("Bat :");
      OLED_print_int(bat / 1000, 3);
      OLED_string(".");
      OLED_print_int(bat % 1000, 3);
      OLED_string(" V");
      interleave = 0;
      return;
    }
    default:
      interleave = 0;
      return;
  }
}

/*
 * =========================================================================================
 * MAIN APPLICATION LOGIC & STATE MACHINE
 * =========================================================================================
 *
 * ARCHITECTURAL OVERVIEW:
 *   The ATtiny85 is a resource-constrained environment with only 6 I/O pins.
 *   To achieve complex behavior (OLED, Sensors, Sound, LED, Button), we must employ
 *   Time-Division Multiplexing on Pin 6 (PB1).
 *
 * PIN 6 (PB1) DUALITY:
 *   1. INPUT MODE (Sleep):   The pin acts as a sensor to detect button presses.
 *   2. OUTPUT MODE (Active): The pin drives the LED using Software PWM.
 *
 * TIMER UTILIZATION:
 *   This project utilizes 100% of the available hardware timers:
 *   - TIMER 0: Handles System Time (millis) AND LED Software PWM (via Interrupt).
 *   - TIMER 1: Handles High-Speed Frequency Generation for the Piezo Sounder.
 *
 */

// --- Global State Variables ---

/**
 * Timestamp (ms) when the switch was FIRST pressed.
 * This determines the absolute phase of the audio warble.
 * It is NOT modified during the sampling loops to ensure continuous sound.
 */
unsigned long soundStartTime;

/**
 * Timestamp (ms) marking the start of the CURRENT LED pulse.
 * This is incremented by 'ledPeriodMs' every cycle to align the
 * sampling window with the "Off" state of the LED.
 */
unsigned long throbStartTime;

/**
 * @brief  Activates the "Sonic Screwdriver" Effects.
 * @desc   1. Reconfigures Pins from High-Z (Input) to Driven (Output).
 *         2. Initializes all external peripherals (OLED, Sensors).
 *         3. Starts the Software PWM and Sound timers.
 */
void turn_effects_on() {
  // --- PIN CONFIGURATION: TAKING CONTROL ---
  // We explicitly drive the pins now.
  // Note: Pin 6 (PB1) is now forcibly taken from the Switch and given to the LED.
  pinMode(PIEZO_PLUS_PIN, OUTPUT);
  pinMode(PIEZO_MINUS_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);

  // --- DEVICE INITIALIZATION ---
  SSD1306_init();  // Configure Screen buffers
  QMC5883P_init(); // Configure Compass settings
  BME280_init();   // Configure Atmospheric Sensor

  // --- UI RESET ---
  SSD1306_clear();
  SSD1306_wake();
}


/**
 * @brief  Deactivates Effects and prepares for Sleep.
 * @desc   1. Shuts down peripherals to save power.
 *         2. Reconfigures Pins to INPUT (High-Z).
 *         This is critical for ISP Programming safety; it ensures the ATtiny
 *         doesn't fight the programmer on the shared pins when idle.
 */
void turn_effects_off() {
  // --- SHUTDOWN PERIPHERALS ---
  SSD1306_sleep();
  QMC5883P_sleep();
  BME280_sleep();

  // --- PIN CONFIGURATION: RELEASING CONTROL ---
  // We set pins to INPUT to float them.
  // This allows the Switch to work again (PB1) and makes ISP programming safe.
  pinMode(PIEZO_PLUS_PIN, INPUT);
  pinMode(PIEZO_MINUS_PIN, INPUT);
  pinMode(LED_PIN, INPUT);
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}

/**
 * @date 2026-02-14 08:46:56
 *
 * It tuens out that the LED brightness when pressing/releasing mismatches the throbbing cycle,
 *   so more half-cycle states are introduced
 */
enum {
  ACTIVATE,   // Activate the device
  FLASHLIGHT, // LED on, sound off
  THROBBING,  // Throbbing mode
  WAIT,       // Wait for led to turn off before deactivating
  DEACTIVATE  // deactivate device
} state = ACTIVATE;

/**
 * @brief  System Setup
 * @desc   Runs once at power-on. Initializes communication lines to a safe state
 *         and allows power voltages to stabilize before talking to peripherals.
 */
void setup() {
  I2C_init(); // Set SDA/SCL to Float/High-Z ONCE to prevent bus contention

  // Hardware Stabilization Delay
  _delay_ms(50);

  // Wait for initial switch press
  state = DEACTIVATE;
}

/**
 * @brief  Main Execution Loop
 * @desc   Handles the Time-Division Multiplexing logic.
 */
void loop() {
  /*
   * Test for device de-activation
   */

  if (state == DEACTIVATE) {
    // shutdown device
    sonic_noInt();
    led_noInt();
    turn_effects_off();

    // CPU HALT. Wait for switch press
    PWR_power_down();

    // Remember start times
    throbStartTime = soundStartTime = millis();

    // after wakeup, activate device
    state = ACTIVATE;
  }

  /*
   * Device is now active.
   * Calculate relative timelines
   */

  // LED Phase: Resets every cycle
  long throbElapsed = (long) (millis() - throbStartTime);

  // Sound Phase: Continuous since press
  long soundElapsed = (long) (millis() - soundStartTime);

  /*
   * Test for activation mode select
   */

  if (state == ACTIVATE) {
    // activate the device

    // wait until selection timer expired
    if (soundElapsed < cfg_modeTimeout)
      return;

    // LED/switch is still set to switch
    byte buttonState = digitalRead(SWITCH_PIN);

    // wakeup device
    turn_effects_on();

    if (buttonState == LOW) {
      // switch pressed, throb mode
      sonic_Init();    // Prepare Timer1 for sound
      led_init();      // Prepare Timer0 for LED PWM

      // Remember start times
      throbStartTime = soundStartTime = millis();

      state = THROBBING;

    } else {
      // switch released, flashlight mode
      digitalWrite(LED_PIN, LOW);// Turn LED ON

      state = FLASHLIGHT;
    }
  }

  /*
   * Test for flashlight mode
   */
  if (state == FLASHLIGHT) {

    if (throbElapsed < cfg_flashCycle) {
      // Update UI (OLED)
      update_ui();
    } else {
      // sense switch to exit flsahlight
      // NOTE: no delay
      pinMode(LED_PIN, INPUT_PULLUP);
      byte buttonState = digitalRead(SWITCH_PIN);
      pinMode(LED_PIN, OUTPUT);
      digitalWrite(LED_PIN, LOW);// Turn LED ON

      if (buttonState == LOW) {
        // switch pressed
        state = DEACTIVATE;
      } else {
        // wait another cycle
        throbStartTime += cfg_flashCycle;
      }
    }

    return;
  }

  /*
   * Test for throbbing continuation
   */
  if (state == THROBBING && throbElapsed >= cfg_throbHalfCycle * 2) {
    // LED=on, sense switch, only if switch is released will it briefly turn off

    // switch multiplexer to Input/Switch
    pinMode(LED_PIN, INPUT_PULLUP);
    _delay_us(40); // Allow voltage to stabilize

    // red pin6 state
    // LOW = Pressed, HIGH = Released.
    byte buttonState = digitalRead(SWITCH_PIN);

    // switch multiplexer to Output/LED
    pinMode(LED_PIN, OUTPUT);

    if (buttonState != LOW) {
      // switch released, wait until LED goes off and deactivate
      state = WAIT;
    } else {
      // We increment the start time by exactly one period length.
      // This ensures the next sine wave starts perfectly in phase,
      // creating seamless visual smoothness despite the interruption.
      throbStartTime += cfg_throbHalfCycle * 2;
    }
  }

  /*
   * Test for timed deactivation after switch release
   */
  /**
   * @date 2026-02-14 16:56:41
   * loopUI() can take up to 125 mSec to complete, which might be so long
   * that the next test might occur on the rising edge causing it to miss.
   * To counter this, test for led OFF, and a maximum wait time
   */

  if (state == WAIT && (ledBrightness == 0 || throbElapsed >= cfg_throbHalfCycle * 3)) {
    // LED is now off, deactivate
    state = DEACTIVATE;
  }

  /*
   * Update device
   */

  // Update Light (Software PWM / Timer0)
  // Driven by the cycling 'lightElapsed' time base.
  update_led_throb(throbElapsed);

  // Update Sound (Hardware Wave / Timer1)
  // Driven by the absolute 'soundElapsed' time base.
  // This ensures no hiccups in the audio even when the LED resets.
  update_sonic_warble(soundElapsed);

  // Update UI (OLED)
  update_ui();
}
