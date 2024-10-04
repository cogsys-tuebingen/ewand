const int LEDPin = 21;
const int LEDBoard = 2;

int dutyCycle;

const int PWMFreq = 500;
const int PWMChannel = 0;
const int PWMResolution = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);


const int pin_1 = 4;
const int pin_2 = 17;
const int pin_3 = 18;
const int pin_4 = 21;
const int pin_5 = 2;

//const int freq_1 = 100;
//const int freq_2 = 200;
//const int freq_3 = 80;
//const int freq_4 = 150;


const int freq_1 = PWMFreq;
const int freq_2 = PWMFreq;
// const int freq_3 = 1000;
const int freq_4 = PWMFreq;


// const int freq_5 = 100;

const int channel_1 = 0;
const int channel_2 = 2;
// const int channel_3 = 4;
const int channel_4 = 6;
// const int channel_5 = 8;

void setup() {
  // put your setup code here, to run once:
  start_pin(pin_1, freq_1, channel_1);
  start_pin(pin_2, freq_2, channel_2);
  // start_pin(pin_3, freq_3, channel_3);
  start_pin(pin_4, freq_4, channel_4);
  // start_pin(pin_5, freq_5, channel_5);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void start_pin(int pin, int freq, int channel) {
  ledcSetup(channel, freq, PWMResolution);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 127); 
}