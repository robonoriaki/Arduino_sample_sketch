// led_brightness_sample1

const int led_pin = 3;

void setup() {
}

void loop() {
  //led_value = PWM    0 -> 100%    255 -> 0%
  
  for ( int led_value = 0; led_value < 256; led_value += 10 ) {
    analogWrite( led_pin, led_value );
    delay( 30 );
  }
  for ( int led_value = 255; led_value > -1; led_value -= 10 ) {
    analogWrite( led_pin, led_value );
    delay( 30 );
  }
}
