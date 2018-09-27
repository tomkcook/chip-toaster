#include <Arduino.h>
#include <max6675.h>
#include <PID_v1.h>

/*
 * Serial Protocol
 * ===============
 *
 * Plan To the Toaster
 * -------------------------------
 * 0x85 int-count
 *   (word-rate word-temp word-t) ...
 * 0x85
 * words are two-byte integers, least-significant byte first
 * Temperatures are in 0.01 degree units.
 *
 * 0x95 - cancel plan
 *
 * Responses
 * ---------
 * 0x11 - Starting ramp
 * 0xFF - Error reading serial data
 * 0x12 - Cancelled ramp
 * 0x14 - Ramp complete
 * 0x18 - Plan received
 * 0x19 - Plan complete
 * 0x21 float - measured temp
 * 0x22 float - filtered temp
 * 0x24 float - measured rate
 * 0x28 float - filtered rate
 * 0x31 float - control error
 * 0x32 float - control output
 * 0x55 string newline - string message
 */

MAX6675 thermocouple(13, 8, 12);

struct Filter1st {
        double dt;
        double tau;
        double y;

        Filter1st(double dt, double f) {
                tau = dt / (1 / f + dt);
                y = 0;
                this->dt = dt;
        }

        void operator()(double in) {
                y = tau * in + (1 - tau) * y;
        }

        operator double() {
                return y;
        }
};

static const double dt = 1.0;

uint32_t out_pin = 5;
Filter1st in_filt(dt, 0.1);
Filter1st rate_filt(dt, 0.1);
double filtered_rate, output_fraction,
  set_rate, target_temp, filtered_temp;
bool active;
double Kp = 2, Ki = 0.01, Kd = 0;
double timer;

PID rate_control(&filtered_rate, &output_fraction, &set_rate, Kp, Ki, Kd, DIRECT);
PID pos_control(&filtered_temp, &output_fraction, &target_temp, Kp/10, Ki/10, Kd/10, DIRECT);

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.write(0x55);
  Serial.println("MAX6675");
  output_fraction = 0;
  set_rate = 0;
  target_temp = 0;
  rate_control.SetOutputLimits(0, 1);
  pos_control.SetOutputLimits(0, 1);
  rate_control.SetSampleTime(int(dt * 1000));
  pos_control.SetSampleTime(int(dt * 1000));
  active = false;
  delay(1000);
  double temp = thermocouple.readCelsius();
  in_filt.y = temp;
  timer = 0;
}

void sendStatus(uint8_t code) {
  Serial.write(code);
}

void sendFloat(float val, uint8_t code) {
  Serial.write(code);
  uint8_t* ptr = (uint8_t*)&val;
  for(int ii = 0; ii < 4; ii++) {
    Serial.write(*(ptr+ii));
  }
}

float readFloat() {
  float val;
  int count = Serial.readBytes((uint8_t*)&val, 4);
  if(count == 4) {
    return val;
  } else {
    return NAN;
  }
}

#include "plan.h"

Step plan[20];
int32_t len;
Step* current_step;

void loop() {
  double input = thermocouple.readCelsius();
  if(input != input) return;
  static double last_input = input;
  double rate = (input - last_input) / dt;
  last_input = input;

  rate_filt(rate);
  filtered_rate = rate_filt;
  in_filt(input);
  filtered_temp = in_filt;

  int try_len;
  receivePlan(plan, try_len);
  if(try_len != 0) {
    len = try_len;
    current_step = plan;
    set_rate = current_step->rate;
    target_temp = current_step->target;
  }

  uint32_t byte = Serial.peek();
  if (byte == 0x95) {
    len = 0;
    pos_control.SetMode(MANUAL);
    rate_control.SetMode(MANUAL);
  }


  if(len != 0) {
    Step& s = *current_step;
    if((set_rate > 0 && filtered_temp > s.target - 10) ||
       (set_rate < 0 && filtered_temp < s.target)) {
      pos_control.SetMode(AUTOMATIC);
      rate_control.SetMode(MANUAL);
      timer += dt;
      sendStatus(0x55);
      Serial.print("Timer ");
      Serial.println(timer);
      if(timer > s.t) {
        current_step++;
        if( current_step - plan >= len ) {
          len = 0;
        } else {
          set_rate = current_step->rate;
          target_temp = current_step->target;
        }
      }
    } else {
      pos_control.SetMode(MANUAL);
      rate_control.SetMode(AUTOMATIC);
        sendStatus(0x55);
        Serial.println("Timer reset 1");
      timer = 0;
    }

    pos_control.Compute();
    rate_control.Compute();

    analogWrite(out_pin, int(output_fraction * 255));
  }
  else
  {
    analogWrite(out_pin, 0);
  }
  sendFloat(input, 0x21);
  sendFloat(filtered_temp, 0x22);
  sendFloat(rate, 0x24);
  sendFloat(filtered_rate, 0x28);
//  sendFloat(rate_control.error, 0x31);
  sendFloat(output_fraction, 0x32);

  delay(int(dt*1000));
}
