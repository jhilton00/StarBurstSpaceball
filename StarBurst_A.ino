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
// provided by integral CPU operations.
//
// 'Real' value types are specified as integer types with the type name
// including the scale factor. For example, the type ONE_0x4000 specifies
// a 16-bit signed integer where the floating point value was scaled by
// 0x4000 then converted to an integer. i.e. A real value of 1.0 scales to
// and is represented as 0x4000. For a ONE_0x4000 the full signed range
// 0x8000 (-32768) to 0x7fff (+32767) correspond to -2.0 precisely to
// +1.99994.
//
// For development and verification both floating point and scaled integer
// calculations plus an error term comparison can be placed side-by-side
// in an Excel spreadsheet.
//
// Scaled integers can also be used with irrational numbers such as
// ONE_2PI in which case integral overflow from 0x7fff to 0x0000
// corresponds to 360deg (2*PI radians) wrap-around.
//
// Yes, some of the code below can be mind bending to follow!

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

//========== Vector Math calculating the 3D force and 3D torque vectors
//
// The six 'sensor' forces are spatially arranged as three sets of parallel
// pairs all the same distance, 24.9mm, from the middle of their
// corresponding wire rod. Summing a pair provides a 'rod' force that is
// parallel to the two sensor forces and acts through the middle of the
// wire rod. Likewise subtracting a pair provides a 'rod' torque that is
// perpendicular to both the rod axis and the rod force. This torque
// calculation treats the 24.9mm lever arm distance as a unit length.
//
// In the mechanism's coordinate system the middle point of each wire rod
// is at (-a,+a,0) rodZ/forceX/torqueY
// (0,-a,+a) rodX/forceY/torqueZ
// and (+a,0,-a) rodY/forceZ/torqueX
// The middle point of each rod's axes is in one of the X-Y, Y-Z and Z-X
// planes and 0.57mm distant from the other two. Transforming the rod's
// force and torque vectors to act through the mechanism's centre uses the
// 0.57/24.9 ratio as another much smaller lever arm distance.


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
typedef int16_t ONE_0x2000, ONE_0x4000, ONE_0x8000, ONE_0x0400, ONE_0x0800;
typedef uint16_t ONE_U0x0800;
typedef ONE_0x4000 Vec3_0x4000[3];
typedef ONE_0x8000 Mat33_0x8000[3][3];

//========== Compile-time parameters.
const int SETTLING_MSEC = 1;  // Time between DAC_ON and analogueRead().
const int SENSING_RATE_HZ = 60;
const int NULL_RADIUS = 10;  // The smallest output vector length squared.

//========== Constants.

// Subtracting each arm force pairs to obtain a torque value uses the
// distance from the centre of the ball centre to the centre of the
// spherical tip as a unit distance. To include the small torques
//
// Distances from the centre of the mechanism:
//    - to the middle of the 0.64mm dia wire rod axes is 0.57mm
//    - to the centre of the spherical ball tip is 24.9mm
const ONE_0x4000 rodOffset = (ONE_0x4000)((0.25 + 0.64 / 2) / 24.9 * 0x4000);

// The centre of the spherical ball tips is 24.9mm from the centre of the
// sensor ball.

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

// Provide the rotation transformation from the mechanism's coordinate
// system to the world's +X right, +Y up and +Z back.
#define IVAL(a) (ONE_0x8000)(a * 0x8000)
static const ONE_0x8000 ballToWorld[3][3] = {
  { IVAL(0), IVAL(-0.333333333), IVAL(-0.942809042) },
  { IVAL(0.816496581), IVAL(-0.333333333), IVAL(0.471404521) },
  { IVAL(-0.816496581), IVAL(-0.333333333), IVAL(0.471404521) }
};
#undef IVAL


//========== Variables.

// The atRest sensor readings are taken when the mechanism is 'at rest'
// i.e. not being touched.
// 0x2000 corresponds to 0.5
ONE_0x4000 atRestReadings[6] = { 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000 };

// The six force values being the delta from atRestReadings[].
typedef int16_t ONE_0x0800;  // -2048 to 2047 corresponds to -1.0 to +0.9995
ONE_0x0800 armForces[6];

// The sensor-axis aligned values.
ONE_0x8000 ballForce[3], ballTorque[3];

// Calculate 1000000usec/SENSING_RATE (60Hz --> 16667 usec).
const uint16_t PERIOD_USEC = (uint16_t)(1000000.0 / SENSING_RATE_HZ + 0.5);
uint16_t timeout_usec;

//========== Functions.

void setup() {
  // put your setup code here, to run once:

  // Initialise the serial port and print the firmware title.
  Serial.begin(115200);
  Serial.println("*StarBurst* Spaceball v0.00");

  // MUST call analogReference() before analogueRead() to avoid damaging the
  // microcontroller.
  analogReference(EXTERNAL);

  // Configure the relevant pins.
  configurePin(DAC_ON, OUTPUT, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  configurePin(L16H34T25, INPUT, LOW);
  configurePin(L23H56T14, INPUT, LOW);
  configurePin(L45H12T36, INPUT, LOW);

  // Timeout in PERIOD_USEC time.
  timeout_usec = (uint32_t)micros() + PERIOD_USEC;
}

void loop() {
  // put your main code here, to run repeatedly:

  uint16_t remaining_usec = timeout_usec - (uint16_t)micros();
  if (remaining_usec & 0x8000) {  // Already timed out!?
    Serial.println("Unexpected timeout");
    takeReadingsAndGenerateResults();
    timeout_usec = micros() + PERIOD_USEC;
  } else if (remaining_usec < 2000) {  // Is there less than 2msec to go?
    delayMicroseconds(remaining_usec);
    takeReadingsAndGenerateResults();
    timeout_usec += PERIOD_USEC;
  }
}

void takeReadingsAndGenerateResults() {

  ONE_0x4000 armForces[6];    // The six sensor readings.
  ONE_0x4000 panZoomSpin[6];  // The 2D pan, 1D zoom and 3D spin values.

  // Take the six readings.
  for (uint8_t i = 0; i < 6; i++)
    armForces[i] = takeForceReading(sensorDrives[i], atRestReadings[i]);

  // Process the readings.
  processReadings(armForces, panZoomSpin);

  // Determine the period.
  static unsigned long previous_msec = 0;
  unsigned long period_msec = millis() - previous_msec;
  previous_msec += period_msec;
  if (period_msec > 999)
    period_msec = 999;

  // Show the results.
  static int16_t counter = 100;
  Serial.print(counter++);
  if (counter >= 1000)  // repetitively count from 100 to 999
    counter = 100;
  Serial.print(period_msec < 100 ? period_msec < 10 ? ":   " : ":  " : ": ");
  Serial.print(period_msec);
  Serial.print(' ');
  printInt16Array(panZoomSpin, 6);
  Serial.print('\n');

  // Toggle LED_BUILTIN every eighth sensing cycle.
  static int8_t iToggleLedBuiltin = 0;
  digitalWrite(LED_BUILTIN, (iToggleLedBuiltin++ & 8) ? HIGH : LOW);
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

  // Take the reading converting it to a scaled integer with a value
  // between 0.0 to almost 1.0. The maximum 10-bit ADC value is 0x03ff.
  ONE_0x4000 reading = analogRead(V_SENSE) << 4;

  // Turn off the LED and tristate the LED pins.
  configurePin(sensorDrive.highPin, INPUT, LOW);
  configurePin(sensorDrive.lowPin, INPUT, LOW);

  // Clear the DAC to reduce DAC current to zero.
  digitalWrite(DAC_ON, LOW);
  configurePin(DAC_0, INPUT, LOW);
  configurePin(DAC_1, INPUT, LOW);
  configurePin(DAC_2, INPUT, LOW);
  configurePin(DAC_3, INPUT, LOW);

  // Subtract the rest reading.
  ONE_0x4000 sensorForce = reading - atRestReading;

  // The ADC value is treated as being between 0.0 and almost 1.0.
  // A rest reading around 0.5 leads to a force range of about -0.5 to +0.5.
  // A poorly built unit could create values much larger so limit the force
  // reading to 0.8 to avoid any potential overflow issues later on.
  const ONE_0x4000 MAX_READING = (ONE_0x4000)(0.8 * 0x4000);
  if (sensorForce < -MAX_READING)
    sensorForce = -MAX_READING;
  else if (sensorForce > MAX_READING)
    sensorForce = MAX_READING;

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

int16_t imul16sr14r16(int16_t a, int16_t b) {
  return (int16_t)(((int32_t)a * b) >> 14);
}

void printVec3(char const* s, Vec3_0x4000 vec3) {
  Serial.print(s);
  Serial.print(vec3[0]);
  Serial.print(vec3[1]);
  Serial.print(vec3[2]);
  Serial.println();
}

// Use the six arm readings to calculate the panZoomSpin output values.
void processReadings(ONE_0x4000 armForces[6], ONE_0x4000 panZoomSpin[6]) {
  // Preliminary code - untested and needs checking to ensure the calculations
  // match the actual device.

  // These vector calculations use the three pairs of parallel sensor forces
  // to determine the force and torque vectors acting through the middle point
  // of the wire rods then shifts the line of actions to pass through the
  // mechanism's centre.

  ballForce[0] = armForces[0] + armForces[3];
  ballForce[1] = armForces[1] + armForces[4];
  ballForce[2] = armForces[2] + armForces[5];
  ONE_0x4000 a = imul16sr14r16(ballForce[0], rodOffset);
  ONE_0x4000 b = imul16sr14r16(ballForce[1], rodOffset);
  ONE_0x4000 c = imul16sr14r16(ballForce[2], rodOffset);
  ballTorque[3] = armForces[0] - armForces[3] + b - c;
  ballTorque[4] = armForces[1] - armForces[4] + c - a;
  ballTorque[5] = armForces[2] - armForces[5] + a - b;

  ONE_0x4000 worldForce[3], worldTorque[3];

  // Transform into world coordinates +X right, +Y up, +Z back.
  v3MulM33(ballForce, ballToWorld, worldForce);
  v3MulM33(ballTorque, ballToWorld, worldTorque);

  cubicTransform(worldForce, panZoomSpin + 0);   // Transform force vector into pan/zoom velocities
  cubicTransform(worldTorque, panZoomSpin + 3);  // Transform torque vector in spin velocity
}

// If the input vector has a length less than 1.0 then scale it by its
// length squared to provide a cubic length transformation.
// A vector's length squared is the sum of its squares.
void cubicTransform(ONE_0x4000 const inVec3[3], ONE_0x4000 outVec3[3]) {

  // A ONE_0x2000 direct assignment to a ONE_0x4000 effectively halves the
  // value.
  ONE_0x4000 halfSumSquares = (ONE_0x4000)sumSquares_0x2000(inVec3);

  // Mouse warp is a standard and necessary 2D mouse technique everyone
  // uses without realising it. Likewise, for a spaceball a cubic response,
  // where twice the push/twist produces eight times the speed of motion,
  // has proven to deliver the best feel. This is readily and efficiently
  // achieved by scaling each 3D vector by the sum of its squares or, in
  // this case, half the sum of its squares.
  // Note: Vectors larger than the unit sphere are not changed.
  if (halfSumSquares < 0x4000) {
    if (halfSumSquares < NULL_RADIUS) {
      // Null out very small vectors
      memset(outVec3, 0, sizeof(ONE_0x4000[3]));
    } else {
      outVec3[0] = imul16sr14r16(inVec3[0], halfSumSquares);
      outVec3[1] = imul16sr14r16(inVec3[1], halfSumSquares);
      outVec3[2] = imul16sr14r16(inVec3[2], halfSumSquares);
    }
  }
}

int16_t imul16sr9r16(int16_t a, int16_t b) {
  return (int16_t)(((int32_t)a * b) >> 9);
}

int16_t imul16sr15r16(int16_t a, int16_t b) {
  return (int16_t)(((int32_t)a * b) >> 15);
}

int16_t isqr16sr9r16(int16_t a) {
  return (int16_t)(((int32_t)a * a) >> 9);
}

ONE_0x2000 sumSquares_0x2000(ONE_0x0800 const vec3[3]) {
  // For the largest vector (-1.0,-1.0,-1.0) the sum of the squares is 3.0 so
  // a ONE_0x2000 is needed.
  // (ONE_0x0800 * ONE_0x0800) / ONE_0x2000 = 512 so a 9-bit right shift is used.
  return isqr16sr9r16(vec3[0]) + isqr16sr9r16(vec3[1]) + isqr16sr9r16(vec3[2]);
}

ONE_0x4000 vec3TimesColumn3(Vec3_0x4000 const vec3, ONE_0x8000 const* p) {
  // Note: The length of the largest vector, (-1.0,-1.0,-1.0), is 1.73 which
  // does not overflow a ONE_0x4000 having a maximum value of almost +2.0.
  // But the values are
  return imul16sr15r16(vec3[0], p[0])
         + imul16sr15r16(vec3[1], p[3]) + +imul16sr15r16(vec3[2], p[6]);
}

void v3MulM33(Vec3_0x4000 const v3, Mat33_0x8000 const m33, Vec3_0x4000 resultV3) {

  // Matrix multiply v3 x m33
  resultV3[0] = vec3TimesColumn3(v3, m33[0] + 0);
  resultV3[1] = vec3TimesColumn3(v3, m33[0] + 1);
  resultV3[2] = vec3TimesColumn3(v3, m33[0] + 2);
}

void printInt16Array(int16_t const* p, uint8_t count) {
  // Print count int16_t values as
  //    "snnnnn  snnnnn  ...  snnnnn  snnnnn"
  for (uint8_t i = 0; i < count; i++) {
    char buf[10];
    sprintf(buf, i == 0 ? "%+07d" : "  %+07d", *p++);
    Serial.print(buf);
  }
}
