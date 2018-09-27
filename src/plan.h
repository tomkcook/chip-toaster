struct Step {
  Step() : rate(0), target(0), t(0) {}
  Step(double rate, double target, double t) : rate(rate), target(target), t(t) {}

  double rate;
  double target;
  double t;
};

typedef Step* Plan;

void receivePlan(Plan plan, int& len) {
  int32_t byte = Serial.peek();
  if (byte != 0x85) {
    len = 0;
    return;
  }

  byte = Serial.read();
  int32_t number_of_steps = Serial.read();
  len = number_of_steps;

  sendStatus(0x55);
  Serial.print("Receiving plan with ");
  Serial.print(number_of_steps);
  Serial.println(" steps.");

  for(int ii = 0; ii < number_of_steps; ii++) {
    int32_t rate = Serial.read() + (Serial.read() << 8);
    int32_t temp = Serial.read() + (Serial.read() << 8);
    int32_t t    = Serial.read() + (Serial.read() << 8);
    sendStatus(0x55);
    Serial.print("Step: ");
    Serial.print(rate);
    Serial.print(", ");
    Serial.print(temp);
    Serial.print(", ");
    Serial.print(t);
    Serial.println("");
    plan[ii] = Step(double(rate)/100, double(temp)/100, t / 100);
  }

  byte = Serial.read();
  if(byte != 0x85) {
    sendStatus(0xFF);
    sendStatus(0x55);
    Serial.print("Invalid end character ");
    Serial.print(byte);
    Serial.println(" received at end of plan.");
    len = 0;
  }

  sendStatus(0x18);
  return;
}
