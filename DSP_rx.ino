#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/i2s.h>

// ===== Wi-Fi Settings =====
const char* ssid = "Shannn";
const char* password = "Jayant117";

// ===== UDP Settings =====
const uint16_t udpPort = 5004;
WiFiUDP udp;

// ===== I2S Settings =====
#define I2S_WS   25
#define I2S_SD   22
#define I2S_SCK  26
#define SAMPLE_RATE 16000
#define FRAME_SAMPLES 320

int16_t pcmBuffer[FRAME_SAMPLES];

// ===== FIR Filter =====
#define FIR_TAP_NUM 5
float fir_coeffs[FIR_TAP_NUM] = {0.2,0.2,0.2,0.2,0.2};
int16_t fir_buffer[FIR_TAP_NUM] = {0};

int16_t applyFIR(int16_t sample){
  for(int i=FIR_TAP_NUM-1;i>0;i--) fir_buffer[i]=fir_buffer[i-1];
  fir_buffer[0]=sample;
  float out=0;
  for(int i=0;i<FIR_TAP_NUM;i++) out+=fir_buffer[i]*fir_buffer[i];
  return (int16_t)out;
}

// ===== ADPCM Decoder =====
const int stepTable[89] = {7,8,9,10,11,12,13,14,16,17,
19,21,23,25,28,31,34,37,41,45,
50,55,60,66,73,80,88,97,107,118,
130,143,157,173,190,209,230,253,279,307,
337,371,408,449,494,544,598,658,724,796,
876,963,1060,1166,1282,1411,1552,1707,1878,2066,
2272,2499,2749,3024,3327,3660,4026,4428,
4871,5358,5894,6484,7132,7845,8630,9493,
10442,11487,12635,13899,15289,16818,18500,
20350,22385,24623,27086,29794,32767};
const int indexTable[16] = {-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8};

int16_t adpcm_prev = 0;
int8_t adpcm_index = 0;

int16_t adpcmDecode(uint8_t nibble){
  int step = stepTable[adpcm_index];
  int diff = step >> 3;
  if(nibble & 4) diff += step;
  if(nibble & 2) diff += step>>1;
  if(nibble & 1) diff += step>>2;
  if(nibble & 8) diff = -diff;
  adpcm_prev += diff;
  if(adpcm_prev>32767) adpcm_prev=32767;
  if(adpcm_prev<-32768) adpcm_prev=-32768;
  adpcm_index += indexTable[nibble];
  if(adpcm_index<0) adpcm_index=0;
  if(adpcm_index>88) adpcm_index=88;
  return adpcm_prev;
}

// ===== I2S Setup =====
void setupI2S(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags=0,
    .dma_buf_count=8,
    .dma_buf_len=128,
    .use_apll=false
  };
  i2s_pin_config_t pin_config = {.bck_io_num=I2S_SCK,.ws_io_num=I2S_WS,.data_out_num=I2S_SD,.data_in_num=-1};
  i2s_driver_install(I2S_NUM_0,&i2s_config,0,NULL);
  i2s_set_pin(I2S_NUM_0,&pin_config);
}

// ===== Setup =====
void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected: "+WiFi.localIP().toString());

  udp.begin(udpPort);
  Serial.printf("Listening on UDP port %d\n", udpPort);

  setupI2S();
}

// ===== Loop =====
void loop(){
  int packetSize = udp.parsePacket();
  if(packetSize){
    uint8_t packet[512];
    int len = udp.read(packet,sizeof(packet));
    if(len < 12) return; // RTP header missing

    // RTP payload starts at byte 12
    int payload_len = len - 12;
    int pcmIndex = 0;
    for(int i=0;i<payload_len;i++){
      uint8_t byte = packet[12+i];
      pcmBuffer[pcmIndex++] = applyFIR(adpcmDecode(byte>>4));
      pcmBuffer[pcmIndex++] = applyFIR(adpcmDecode(byte & 0x0F));
    }

    // Output PCM to I2S
    size_t bytesWritten;
    i2s_write(I2S_NUM_0, pcmBuffer, pcmIndex*2, &bytesWritten, portMAX_DELAY);
  }
}
