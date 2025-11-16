#include "arm_math.h"
#include <math.h>
#include <WiFiS3.h>  // For Arduino UNO R4 WiFi

// ---------- Wi-Fi Settings ----------

const char* ssid = "Remi";
const char* password = "qwerty123";
const char* serverIP = "10.62.184.179"; // Replace with your ESP32’s IP
int serverPort = 80;

// ---------- EEG Settings ----------
#define SAMPLE_RATE 512
#define FFT_SIZE 256
#define BAUD_RATE 115200
#define INPUT_PIN A0

#define DELTA_LOW 0.5
#define DELTA_HIGH 4.0
#define THETA_LOW 4.0
#define THETA_HIGH 8.0
#define ALPHA_LOW 8.0
#define ALPHA_HIGH 13.0
#define BETA_LOW 13.0
#define BETA_HIGH 30.0
#define GAMMA_LOW 30.0
#define GAMMA_HIGH 45.0

#define BetaThreshold 10
#define SMOOTHING_FACTOR 0.63
const float EPS = 1e-6f;

// ---------- Bandpower Structures ----------
typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults, SmoothedBandpower;

SmoothedBandpower smoothedPowers = {0};

// ---------- Wi-Fi Client ----------
WiFiClient client;

// ---------- Filters ----------
float Notch(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - (-1.56858163f * z1) - (0.96424138f * z2);
    output = 0.96508099f * x + (-1.56202714f * z1) + (0.96508099f * z2);
    z2 = z1; z1 = x;
  }
  {
    static float z1 = 0, z2 = 0;
    float x = output - (-1.61100358f * z1) - (0.96592171f * z2);
    output = 1.00000000f * x + (-1.61854514f * z1) + (1.00000000f * z2);
    z2 = z1; z1 = x;
  }
  return output;
}

float EEGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.22465158 * z1 - 0.45044543 * z2;
    output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
    z2 = z1; z1 = x;
  }
  return output;
}

// ---------- FFT Variables ----------
float inputBuffer[FFT_SIZE];
float fftOutputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];
arm_rfft_fast_instance_f32 S;
volatile uint16_t sampleIndex = 0;
volatile bool bufferReady = false;

// ---------- Utility Functions ----------
void smoothBandpower(BandpowerResults *raw, SmoothedBandpower *smoothed) {
  smoothed->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * smoothed->delta;
  smoothed->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * smoothed->theta;
  smoothed->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * smoothed->alpha;
  smoothed->beta  = SMOOTHING_FACTOR * raw->beta  + (1 - SMOOTHING_FACTOR) * smoothed->beta;
  smoothed->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * smoothed->gamma;
  smoothed->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * smoothed->total;
}

BandpowerResults calculateBandpower(float *powerSpectrum, float binResolution, uint16_t halfSize) {
  BandpowerResults r = {0};
  for (uint16_t i = 1; i < halfSize; i++) {
    float freq = i * binResolution;
    float power = powerSpectrum[i];
    r.total += power;
    if (freq >= DELTA_LOW && freq < DELTA_HIGH) r.delta += power;
    else if (freq >= THETA_LOW && freq < THETA_HIGH) r.theta += power;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH) r.alpha += power;
    else if (freq >= BETA_LOW && freq < BETA_HIGH) r.beta += power;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH) r.gamma += power;
  }
  return r;
}

// ---------- Motor Command ----------
void sendCommandToESP32(String cmd) {
  if (client.connect(serverIP, serverPort)) {
    client.print(String("GET ") + cmd + " HTTP/1.1\r\nHost: " + serverIP + "\r\nConnection: close\r\n\r\n");
    client.stop();
  }
}

// ---------- FFT Processing ----------
bool motorActive = false;

void processFFT() {
  arm_rfft_fast_f32(&S, inputBuffer, fftOutputBuffer, 0);

  uint16_t halfSize = FFT_SIZE / 2;
  for (uint16_t i = 0; i < halfSize; i++) {
    float real = fftOutputBuffer[2 * i];
    float imag = fftOutputBuffer[2 * i + 1];
    powerSpectrum[i] = real * real + imag * imag;
  }

  float binResolution = (float)SAMPLE_RATE / FFT_SIZE;
  BandpowerResults raw = calculateBandpower(powerSpectrum, binResolution, halfSize);
  smoothBandpower(&raw, &smoothedPowers);

  float betaPercent = (smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100;

  // Beta detection and motor trigger
  if (betaPercent > BetaThreshold && !motorActive) {
    motorActive = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("🧠 Beta detected! Sending MOTOR=ON");
    sendCommandToESP32("/MOTOR=ON");
  } else if (betaPercent <= BetaThreshold && motorActive) {
    motorActive = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("😌 Beta dropped. Sending MOTOR=OFF");
    sendCommandToESP32("/MOTOR=OFF");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  arm_rfft_fast_init_f32(&S, FFT_SIZE);

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Connected to Wi-Fi!");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  static unsigned long lastMicros = 0;
  unsigned long currentMicros = micros();
  unsigned long interval = currentMicros - lastMicros;
  lastMicros = currentMicros;

  static long timer = 0;
  timer -= interval;
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;

    int rawSample = analogRead(INPUT_PIN);
    float filteredSample = EEGFilter(Notch(rawSample));

    if (sampleIndex < FFT_SIZE) inputBuffer[sampleIndex++] = filteredSample;
    if (sampleIndex >= FFT_SIZE) bufferReady = true;
  }

  if (bufferReady) {
    processFFT();
    sampleIndex = 0;
    bufferReady = false;
  }
}
