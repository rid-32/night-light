#include <Arduino.h>

#define R_LED 9
#define G_LED 10
#define B_LED 11
#define CONTROLLER 2
#define DELAY 5

class Controller {
private:
  uint8_t pin;
  bool prev_state = false;

public:
  Controller(uint8_t pin) { this->pin = pin; }

  bool is_pressed = false;

  void debounce_read() {
    bool state = digitalRead(this->pin);

    if (state != this->prev_state) {
      delay(5);

      state = digitalRead(this->pin);
    }

    this->is_pressed = (state == LOW) && (prev_state == HIGH);
    this->prev_state = state;
  }
};

class Light {
private:
  uint8_t red_pin, green_pin, blue_pin;
  enum Light_modes { OFF, RED, GREEN, BLUE, PURPLE, TURQUOISE, ORANGE, WHITE };
  uint8_t current_mode = OFF;
  uint8_t last_mode = WHITE;

  void set_current_mode() {
    switch (this->current_mode) {
    case RED:
      digitalWrite(red_pin, HIGH);
      digitalWrite(green_pin, LOW);
      digitalWrite(blue_pin, LOW);
      break;
    case GREEN:
      digitalWrite(red_pin, LOW);
      digitalWrite(green_pin, HIGH);
      digitalWrite(blue_pin, LOW);
      break;
    case BLUE:
      digitalWrite(red_pin, LOW);
      digitalWrite(green_pin, LOW);
      digitalWrite(blue_pin, HIGH);
      break;
    case PURPLE:
      analogWrite(red_pin, 127);
      analogWrite(green_pin, 0);
      analogWrite(blue_pin, 127);
      break;
    case TURQUOISE:
      analogWrite(red_pin, 0);
      analogWrite(green_pin, 127);
      analogWrite(blue_pin, 127);
      break;
    case ORANGE:
      analogWrite(red_pin, 127);
      analogWrite(green_pin, 127);
      analogWrite(blue_pin, 0);
      break;
    case WHITE:
      analogWrite(red_pin, 85);
      analogWrite(green_pin, 85);
      analogWrite(blue_pin, 85);
      break;
    case OFF:
      digitalWrite(red_pin, LOW);
      digitalWrite(green_pin, LOW);
      digitalWrite(blue_pin, LOW);
      break;
    }
  }

public:
  Light(uint8_t R_PIN, uint8_t G_PIN, uint8_t B_PIN) {
    this->red_pin = R_PIN;
    this->green_pin = G_PIN;
    this->blue_pin = B_PIN;

    this->set_current_mode();
  }

  void set_next_mode() {
    if (this->current_mode < this->last_mode) {
      this->current_mode++;
    } else {
      this->current_mode = OFF;
    }

    this->set_current_mode();
  }
};

void setup() {
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(CONTROLLER, INPUT_PULLUP);
}

void loop() {
  static Controller controller(CONTROLLER);
  static Light light(R_LED, G_LED, B_LED);

  controller.debounce_read();

  if (controller.is_pressed) {
    light.set_next_mode();
  }
}
