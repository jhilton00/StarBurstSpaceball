// StarBurst_A.ino : The *StarBurst* spaceball 3D mouse firmware.
//
// =================== PRELIMINARY - IN DEVELOPMENT ===================
//
// The *StarBurst* (TM) spaceball 3D mouse project can be found at
//      reddit.com/r/StarBurstSpaceball
// This project seeks to provide the hardware and software for a low
// cost, robust, high quality 3D mouse.
//
// HISTORY
// 9-Mar-26 JH Rev 0.00  Created.

//========== Notes on Scaled Integer Math
//
// The 'floating point' computations in this code are performed using
// scaled integers for the significant space and computational efficiency
// they provide.
//
// Real value types are specified as integer types with the type name
// including the scale factor. For example, the type ONE_0x4000 specifies
// a 16-bit signed integer where the floating point value was scaled by
// 0x4000 then converted to an integer. i.e. 1.0 scales to 0x4000. For a
// ONE_0x4000 the full signed range 0x8000 (-32768) to 0x7fff (+32767)
// correspond to -2.0 precisely to +1.99994.
//
// For development and verification both floating point and scaled integer
// calculations plus an error term comparison can be placed side-by-side
// in an Excel spreadsheet.
//
// Scaled integers can also be used with irrational numbers such as
// ONE_2PI in which case integral overflow from 0x7fff to 0x0000
// corresponds to 360deg (2*PI radians) wrap-around.
//
// Yes, some of the code below can be mind bending at first!

//========== Notes on Input Force/Torque Vectors and Output Pan/Zoom/Spin
//           Speed Responses.
//
// A spaceball senses a simultaneous 3D push and 3D twist. i.e. A 3D force
// and a 3D torque. These inputs are typically used to pan, zoom and spin
// a screen image.
//
// Treating the system as a black box, it's a good idea to define the
// input and output units and specify the input-output response curves.
// Metric input units are N and mNm. Output units need to be in terms of
// the physical user/screen environment. Panning speed units are usefully
// specified as window-sizes-per-second where the window size is the
// larger of the window width and height. The response feels the same for
// all size windows. Zooming speed units are usefully specified as
// doublings/halvings-per-second, a logarithmic behaviour where positive
// zoom speeds grow the image and negative speeds shrink it. Spin speeds
// are more straightforward being radians-per-second or
// revolutions-per-second, take your pick. The physical and virtual spin
// axes should spatially align, of course.
//
// In a similar fashion to mouse warp used with standard 2D mice, the best
// response curves have been found to be a cubic response where double the
// input value produces eight times the output speed. That is an easy and
// efficient transformation with each 3D input vector being scaled by the
// sum of its squares.

//========== Hardware configuration.

// Note: On the Arduino Nano the A6 and A7 pins are input-only pins.

// The three LED tristate pins.
const uint8_t L23H56T14 = A1;
const uint8_t L16H34T25 = A4;
const uint8_t L45H12T36 = A5;

// The five DAC pins.
const uint8_t DAC_ON = 4;
const uint8_t DAC_0 = 8;
const uint8_t DAC_1 = 9;
const uint8_t DAC_2 = 10;
const uint8_t DAC_3 = 11;

// The ADC input.
const uint8_t V_SENSE = A0;

//========== Types.
typedef int16_t ONE_0x2000, ONE_0x4000, ONE_0x8000, ONE_1024, ONE_2048;
typedef ONE_0x4000 Vec3_0x4000[3];

//========== Compile-time parameters.
const int SETTLING_MSEC = 1;  // Time between DAC_ON and analogueRead().
const int SENSING_RATE_HZ = 60;

//========== Constants.

// Specify the DAC values, the HIGH pin and the LOW pin to turn on LEDs 1
// through 6 with their calibrated current.
const struct ASensorDrive {
  uint8_t dacMask, lowPin, highPin;
} sensorDrives[] = {
  { 0x00, L16H34T25, L45H12T36 },  // L1 H1
  { 0x00, L23H56T14, L45H12T36 },  // L2 H2
  { 0x00, L23H56T14, L16H34T25 },  // L3 H3
  { 0x00, L45H12T36, L16H34T25 },  // L4 H4
  { 0x00, L45H12T36, L23H56T14 },  // L5 H5
  { 0x00, L16H34T25, L23H56T14 }   // L6 H6
};

//========== Variables.

// The atRest sensor readings are taken when the ball sensor is 'at rest'
// i.e. not being touched.
typedef uint16_t ONE_U1024;  // 0 to 1023 corresponds to 0.0 to 0.9990
ONE_U1024 atRestReadings[6] = { 512, 512, 512, 512, 512, 512 };

// The six force values being the delta from atRestReadings[].
typedef int16_t ONE_2048;  // -2048 to 2047 corresponds to -1.0 to +0.9995
ONE_2048 sensorForces[6];

// The sensor-axis aligned values.
ONE_0x8000 forcePanZoom[3], torqueSpin[3];

// The loop variable.
int iSensor = 0, iToggleLedBuiltin = 0;

//========== Forward references.

ONE_0x4000 takeForceReading(struct ASensorDrive const& sensorDrive, int16_t atRestReading);
void prepareDac(uint8_t mask);
void configurePin(uint8_t pin, uint8_t mode, uint8_t val);
void onAllSensorsRead(void);
void cubicTransform(Vec3_0x4000& vec3);

//========== Functions.

void setup() {
  // put your setup code here, to run once:

  // MUST call analogReference() before analogueRead() to avoid damaging the
  // microcontroller.
  analogReference(EXTERNAL);

  configurePin(DAC_ON, OUTPUT, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  configurePin(L16H34T25, INPUT, LOW);
  configurePin(L23H56T14, INPUT, LOW);
  configurePin(L45H12T36, INPUT, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Perform the sensor read process.
  sensorForces[iSensor] = takeForceReading(sensorDrives[iSensor], atRestReadings[iSensor]);
  if (++iSensor > 5) {
    iSensor = 0;
    onAllSensorsRead();  // Fire the event.

    // Toggle LED_BUILTIN every eighth reading.
    digitalWrite(LED_BUILTIN, (iToggleLedBuiltin++ & 8) ? HIGH : LOW);

    // Delay to the specified sensing rate.
    const int SENSE_PERIOD_MSEC = (int)(1000.0 / SENSING_RATE_HZ + 0.5);
    delay(SENSE_PERIOD_MSEC - 6 * SETTLING_MSEC);
  }
}

// Turn the LED on, take a reading, turn it off.
ONE_0x4000 takeForceReading(struct ASensorDrive const& sensorDrive, int16_t atRestReading) {

  // Prepare the DAC.
  prepareDac(sensorDrive.dacMask);

  // Select this LED by pulling one pin HIGH and another LOW.
  configurePin(sensorDrive.highPin, OUTPUT, HIGH);
  configurePin(sensorDrive.lowPin, OUTPUT, LOW);

  // Turn on the LED.
  digitalWrite(DAC_ON, HIGH);

  // Wait for the LED current to settle.
  delay(SETTLING_MSEC);

  // Take the reading.
  // The difference is an 11-bit value which will be treated as a ONE_2048.
  // This is transformed to a ONE_0x4000 by a 3-bit left shift.
  ONE_0x4000 sensorForce = (analogRead(V_SENSE) - atRestReading) << 3;

  // Turn off the LED and tristate the LED pins.
  configurePin(sensorDrive.highPin, INPUT, LOW);
  configurePin(sensorDrive.lowPin, INPUT, LOW);

  // Clear the DAC to reduce DAC current to zero.
  digitalWrite(DAC_ON, LOW);
  configurePin(DAC_0, INPUT, LOW);
  configurePin(DAC_1, INPUT, LOW);
  configurePin(DAC_2, INPUT, LOW);
  configurePin(DAC_3, INPUT, LOW);

  return sensorForce;
}

// Prepare the DAC.
void prepareDac(uint8_t mask) {
  // The top 4 bits of mask are the mode and the bottom 4 bits the value.
  //
  // Prior to entry DAC_0 to DAC_3 are configured as INPUTs.

  if (mask & 0x10)
    configurePin(DAC_0, OUTPUT, (mask & 1) ? HIGH : LOW);
  if (mask & 0x20)
    configurePin(DAC_1, OUTPUT, (mask & 2) ? HIGH : LOW);
  if (mask & 0x40)
    configurePin(DAC_2, OUTPUT, (mask & 4) ? HIGH : LOW);
  if (mask & 0x80)
    configurePin(DAC_3, OUTPUT, (mask & 8) ? HIGH : LOW);
}

// Configure a digital or analogue pin.
void configurePin(uint8_t pin, uint8_t mode, uint8_t val) {
  // Note: When configured as an INPUT a pin's internal pull-up resistor is
  // enabled by setting digitalWrite(pin,HIGH).

  pinMode(pin, mode);
  digitalWrite(pin, val);
}

// Event handler for when all sensors have just been read.
void onAllSensorsRead(void) {
  // Preliminary code - untested - also needs the small adjustments

  forcePanZoom[0] = sensorForces[0] + sensorForces[3];
  forcePanZoom[1] = sensorForces[1] + sensorForces[4];
  forcePanZoom[2] = sensorForces[2] + sensorForces[5];
  torqueSpin[3] = sensorForces[0] - sensorForces[3];
  torqueSpin[4] = sensorForces[1] - sensorForces[4];
  torqueSpin[5] = sensorForces[2] - sensorForces[5];
  cubicTransform(forcePanZoom);  // Transform force vector into pan/zoom velocities
  cubicTransform(torqueSpin);    // Transform torque vector in spin velocity
}

void imuleq16sr14(int16_t& val, uint16_t factor) {
  // "val *= factor;"
  val = (int16_t)(((int32_t)val * factor) >> 14);
}

void cubicTransform(Vec3_0x4000& vec3) {
  // A ONE_0x2000 direct assignment to a ONE_0x4000 effectively halves the
  // value.
  ONE_0x4000 halfSumSquares = sumSquares_0x2000(vec3);

  // Mouse warp is a standard and necessary 2D mouse technique everyone
  // uses without realising it. Likewise, for a spaceball a cubic response,
  // where twice the push/twist produces eight times the speed of motion,
  // has proven to deliver the best feel. This is readily and efficiently
  // achieved by scaling each 3D vector by the sum of its squares or, in
  // this case, half the sum of its squares.
  // Note: Vectors larger than the unit sphere are not changed.
  if (halfSumSquares < 0x4000) {
    imuleq16sr14(vec3[0], halfSumSquares);
    imuleq16sr14(vec3[1], halfSumSquares);
    imuleq16sr14(vec3[2], halfSumSquares);
  }
}

int16_t imul16sr9r16(int16_t a, int16_t b) {
  return (int16_t)(((long)a * b) >> 9);
}

ONE_0x2000 sumSquares_0x2000(ONE_2048 vec3[3]) {
  // For the largest vector (-1.0,-1.0,-1.0) the sum of the squares is 3.0 so
  // a ONE_0x2000 is needed.
  // (ONE_2048 * ONE_2048) / ONE_0x2000 = 512 so a 9-bit right shift is used.
  return imul16sr9r16(vec3[0], vec3[0]) + imul16sr9r16(vec3[1], vec3[1]) + imul16sr9r16(vec3[2], vec3[2]);
}
