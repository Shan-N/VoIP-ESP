#include <WiFi.h>
#include <driver/i2s.h>
#include <WiFiUdp.h>
#include <math.h>

// WiFi
const char* ssid = "Shannn";
const char* password = "Jayant117";

// UDP
const char* udpHost = "10.250.52.111";
const uint16_t udpPort = 5678;
WiFiUDP udp;

// I2S Pins
#define I2S_WS 25
#define I2S_SD 22
#define I2S_SCK 26
#define I2S_PORT I2S_NUM_0

// Audio Settings
#define SAMPLE_RATE 8000
#define BUFFER_LEN 512 // PCM samples per read

// ADPCM tables (keep your original)
static const int indexTable[16] = { -1,-1,-1,-1,2,4,6,8, -1,-1,-1,-1,2,4,6,8 };
static const int stepSizeTable[89] = {
  7,8,9,10,11,12,13,14,16,17,
  19,21,23,25,28,31,34,37,41,45,
  50,55,60,66,73,80,88,97,107,118,
  130,143,157,173,190,209,230,253,279,307,
  337,371,408,449,494,544,598,658,724,796,
  876,963,1060,1166,1282,1411,1552,1707,1878,2066,
  2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
  5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
  15289,16818,18500,20350,22385,24623,27086,29794,32767
};

struct AdpcmState {
  int16_t predictor;
  int index;
};
AdpcmState stateAdpcm = {0,0};

uint8_t adpcm_encode_sample(int16_t sample, AdpcmState &s) {
  int step = stepSizeTable[s.index];
  int diff = sample - s.predictor;
  uint8_t code = 0;
  if(diff < 0){ code = 8; diff = -diff; }
  int delta = step >> 3;
  if(diff >= step){ code |= 4; diff -= step; delta += step; }
  step >>= 1;
  if(diff >= step){ code |= 2; diff -= step; delta += step; }
  step >>= 1;
  if(diff >= step){ code |= 1; delta += step; }
  if(code & 8) s.predictor -= delta; else s.predictor += delta;
  if(s.predictor > 32767) s.predictor = 32767;
  if(s.predictor < -32768) s.predictor = -32768;
  s.index += indexTable[code];
  if(s.index < 0) s.index = 0;
  if(s.index > 88) s.index = 88;
  return code & 0x0F;
}

// -------------------- IIR (biquad cascade) --------------------
#define NUM_BIQUADS 9

typedef struct {
  float b0, b1, b2;
  float a1, a2;   // denominator coefficients (use the convention matched below)
  float z1, z2;   // states
} Biquad;

Biquad biquads[NUM_BIQUADS] = {
  // Pasteed from your MATLAB output (as printed).
  // Format in MATLAB output: { b0, b1, b2, a1, a2 }.
  // If filtering behaves badly, toggle invertA_sign to try opposite sign convention.
  { 0.4414413f, 0.8742580f, 0.4414413f, -1.0334043f, -0.4225162f, 0.0f, 0.0f },
  { 1.0000000f, -1.9995100f, 1.0000000f, 1.8475143f, -0.8621854f, 0.0f, 0.0f },
  { 1.0000000f, 1.9018625f, 1.0000000f, -1.6948304f, -0.8888405f, 0.0f, 0.0f },
  { 1.0000000f, 1.8643931f, 1.0000000f, -1.8179008f, -0.9766887f, 0.0f, 0.0f },
  { 1.0000000f, -1.9974903f, 1.0000000f, 1.9752681f, -0.9806138f, 0.0f, 0.0f },
  { 1.0000000f, 1.8546986f, 1.0000000f, -1.8439679f, -0.9959174f, 0.0f, 0.0f },
  { 1.0000000f, -1.9964995f, 1.0000000f, 1.9919635f, -0.9961291f, 0.0f, 0.0f },
  { 1.0000000f, -1.9962398f, 1.0000000f, 1.9953833f, -0.9993291f, 0.0f, 0.0f },
  { 0.9994393f, -1.9973376f, 0.9994393f, 1.9973376f, -0.9988786f, 0.0f, 0.0f }
};

// If filtering sounds unstable, set this to true to flip the sign of a1/a2 at runtime.
// (Some SOS->export conventions require negation for embedded DF2T implementation.)
bool invertA_sign = false;

// Per-section sanity normalization: if a0 != 1 in the SOS that generated these,
// you should divide b0,b1,b2 and a1,a2 by a0. (We don't have a0 here; check MATLAB export.)
void normalize_sections_if_needed() {
  // placeholder: if you detect saturating outputs, consider reproducing normalization in MATLAB.
}

// DF2 Transposed per-section sample processing
float iir_process_sample(float x) {
  // process through each biquad (DF2T)
  for (int i = 0; i < NUM_BIQUADS; ++i) {
    float b0 = biquads[i].b0;
    float b1 = biquads[i].b1;
    float b2 = biquads[i].b2;
    float a1 = biquads[i].a1;
    float a2 = biquads[i].a2;
    if (invertA_sign) { a1 = -a1; a2 = -a2; }

    float y = b0 * x + biquads[i].z1;
    float z1_new = b1 * x + biquads[i].z2 - a1 * y;
    float z2_new = b2 * x - a2 * y;

    biquads[i].z1 = z1_new;
    biquads[i].z2 = z2_new;

    x = y;
  }
  return x;
}

// -------------------- AGC --------------------
float targetRMS = 5000.0f; // tune this on target device
float agcGain = 1.0f;
float agc_env = 0.0f;
const float agc_attack = 0.01f;   // faster attack -> smaller number (tweak)
const float agc_release = 0.0008f; // slower release -> smaller number
const float agc_min_gain = 0.1f;
const float agc_max_gain = 10.0f;

float agc_process_sample(float x) {
  float absx = fabsf(x);
  // simple peak/envelope detector
  if (absx > agc_env) {
    agc_env = (1.0f - agc_attack) * agc_env + agc_attack * absx;
  } else {
    agc_env = (1.0f - agc_release) * agc_env + agc_release * absx;
  }
  if (agc_env < 1e-6f) agc_env = 1e-6f;
  float desired = targetRMS / agc_env;
  if (desired < agc_min_gain) desired = agc_min_gain;
  if (desired > agc_max_gain) desired = agc_max_gain;
  // smooth gain (slow)
  agcGain = 0.98f * agcGain + 0.02f * desired;
  return x * agcGain;
}

// -------------------- I2S setup --------------------
void i2s_setup(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

// -------------------- Stream & TX --------------------
void streamAudio(){
  int16_t buffer[BUFFER_LEN];
  uint8_t outByte = 0;
  bool highNibble = true;

  while(true){
    size_t bytesRead;
    // blocking read
    i2s_read(I2S_PORT, buffer, BUFFER_LEN * sizeof(int16_t), &bytesRead, portMAX_DELAY);
    uint32_t samplesRead = bytesRead / sizeof(int16_t);

    // Compute input RMS for debug
    float rms_in = 0.0f;
    for (uint32_t i=0;i<samplesRead;i++){ float v = (float)buffer[i]; rms_in += v*v; }
    rms_in = sqrtf(rms_in / (float)samplesRead);

    // Process samples in-place: IIR -> AGC -> quantize back to int16
    for(uint32_t i = 0; i < samplesRead; i++){
      float x = (float)buffer[i];
      // IIR
      x = iir_process_sample(x);
      // AGC
      x = agc_process_sample(x);
      // clamp
      if (x > 32767.0f) x = 32767.0f;
      if (x < -32768.0f) x = -32768.0f;
      buffer[i] = (int16_t)lroundf(x);
    }

    // Compute output RMS for debug
    float rms_out = 0.0f;
    for (uint32_t i=0;i<samplesRead;i++){ float v = (float)buffer[i]; rms_out += v*v; }
    rms_out = sqrtf(rms_out / (float)samplesRead);

    // ADPCM encode & UDP send
    uint8_t adpcmPacket[BUFFER_LEN / 2 + 16];
    int pktIndex = 0;
    outByte = 0; highNibble = true;
    for(uint32_t i = 0; i < samplesRead; i++){
      uint8_t code = adpcm_encode_sample(buffer[i], stateAdpcm);
      if(highNibble){
        outByte = code << 4;
        highNibble = false;
      } else{
        outByte |= code;
        adpcmPacket[pktIndex++] = outByte;
        highNibble = true;
      }
    }
    if(!highNibble) adpcmPacket[pktIndex++] = outByte;

    // send
    udp.beginPacket(udpHost, udpPort);
    udp.write(adpcmPacket, pktIndex);
    udp.endPacket();

    // Debug print (reduced rate): print once per N packets to avoid flooding serial
    static int dbgCounter = 0;
    dbgCounter++;
    if ((dbgCounter & 31) == 0) {
      Serial.printf("rms_in: %.1f  rms_out: %.1f  gain: %.2f  pkt: %d\n",
                    rms_in, rms_out, agcGain, pktIndex);
    }
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while(WiFi.status() != WL_CONNECTED){
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi Connected: ");
  Serial.println(WiFi.localIP());
  i2s_setup();
  udp.begin(0); // dynamic local port
  normalize_sections_if_needed(); // placeholder
  streamAudio(); // blocking loop
}

void loop() {
  // empty (work happens in streamAudio)
}
