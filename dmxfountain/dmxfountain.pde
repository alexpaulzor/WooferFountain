

#include <avr/pgmspace.h>
#include <ffft.h>

/*
Pin configs:
D2: DMX In [because of interrupt]
D1, D3 - D13, A2 - A5: LED channel control output
A0: audio signal in
A1: power for address DIP switch; when HIGH, D3 - D11 are read as DIP for address [only used in setup()]
*/

#define DMX_IN_PIN 2
#define DMX_IN_INTERRUPT 0  //pin 2 = interrupt 0

#define NUM_LED_CHANNELS 18
#define LED_PINS {0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A1, A2, A3, A4, A5}    //skip 2 because it's DMX in.

#define AUDIO_PIN 0

#define NUM_ADDRESS_PINS 9    //2^9 = 512
#define ADDRESS_PINS {3, 4, 5, 6, 7, 8, 9, 10, 11}    //least significat -> most significant


/*
DMX functions:

Channel 0: mode
0 - 19 (or no DMX detected): Sound active (strobe) - Automatically allocate LED channels to frequencies that are present in the signal, strobe at frequency when they are present.
20 - 39: Sound active (solid) - Automatically allocate LED channels to frequencies that are present in the signal, turn on when they are present.
40 - 59: Sound active chase (auto) - automaticaly identify beat frequency and advance chase when present.
60 - 79: Sound active chase (given) - advance chase when ANY of the frequencies given by channels 5+ (in 1-based addressing) appear.
80 - 99: DMX sound active (strobe) - Strobe at freqency when signal contains frequency given by the value of channels 5+, off otherwise.
100 - 119: DMX sound active (solid) - Turn on when signal contains frequency given by the value of channels 5+, off otherwise
120 - 139: DMX chase - use channel 4 as speed (slow -> fast)
140 - 159: DMX manual - Strobe according to channels 5+ literal value

NOTE: in all cases, 'frequency' is (channel_value / 2) Hz, thus has range [0, 127] Hz with resolution 0.5 Hz.
FURTHER NOTE: due to processor speed, the minimum window for sampling is 4.6ms (because of fft computation), so sampling is limited to 200/sec, or 100Hz resolution.

*/

#define NUM_DMX_CHANNELS (NUM_LED_CHANNELS + 4)

#define DMX_CHANNEL_MODE 0
#define DMX_CHANNEL_MIN_FREQ 1
#define DMX_CHANNEL_MAX_FREQ 2
#define DMX_CHANNEL_SPEED 3
#define DMX_CHANNEL_FIRST_LED 4

typedef enum DMX_MODE
{
  auto_sound_strobe = 19,
  auto_sound_solid = 39,
  auto_sound_chase = 59,
  dmx_sound_chase = 79,
  dmx_sound_strobe = 99,
  dmx_sound_solid = 119,
  dmx_chase = 139,
  dmx_manual = 159
} dmx_mode_t;
/*
DMX Protocol:

In the case of DMX a bit is 4 Microseconds long.
A DMX byte contains 11 bits, so a byte is 44us long.
The byte starts with a Start Bit, which is LOW (0).
After the Start Bit follows a Box of 8 Bits, this Box contains a Value between 0-255.
After this 8 Bit follows a Stop Bit and a Release Bit, both are High (1).
Every byte is storing one Value (Intensity) for one channel. Because DMX is controlling 512 channels we have to send 512 Bytes in a stream (packet). 

Note, all timing is for a transmitter.
DMX commands are sent in a stream. The start of an update is signified by a break with a minimum duration of 92us and a mark (a logical one) with a minimum length of 12us. 
Followed by a start code, the standard value being 0x00 but other codes can be used. Then data is sent in a number of slots. 
The standard doesn't specify a minimum number of data slots but the minimum time between breaks is 1204us, and a maximum of 1s. 
If only one receiver is being used hence the need for only one slot, the packet must be stretched or extra empty slots added to meet the minimum timing. 
No maximum timing is specified so long as a packet is sent at-least once per second.
*/

#define DMX_BIT_MICROS 4
#define DMX_START_BIT LOW
#define DMX_BITS_IN_BOX 8
#define DMX_STOP_BIT HIGH
#define DMX_RELEASE_BIT HIGH
#define DMX_BREAK_MICROS 92
#define DMX_MARK_MICROS 12
#define DMX_START_CODE 0x00
#define DMX_MICROS_BETW_BREAKS 1204
#define DMX_BOXES_PER_PACKET 512

/*
Sampling audio:

Audio signal has dc offset so it's centered at 512, range 0 <-> 1024.
We want 128 samples per FFT, resulting in 64 analyzed bands.
The most we care about is 127 Hz, requiring 256 samples/second.



FFT involves 3 steps: 
 void fft_input (const int16_t *array_src, complex_t *array_bfly);
 void fft_execute (complex_t *array_bfly);
 void fft_output (complex_t *array_bfly, uint16_t *array_dst);

  <array_src>: Wave form to be processed.
  <array_bfly>: Complex array for butterfly operations.
  <array_dst>: Spectrum output buffer.

; These functions must be called in sequence to do a DFT in FFT algorithm.
; fft_input() fills the complex array with a wave form to prepare butterfly
; operations. A hamming window is applied at the same time.
; fft_execute() executes the butterfly operations.
; fft_output() re-orders the results, converts the complex spectrum into
; scalar spectrum and output it in linear scale.
;
; The number of points FFT_N is defined in "ffft.h" and the value can be
; power of 2 in range of 64 - 1024.
;
;----------------------------------------------------------------------------;
; 16bit fixed-point FFT performance with MegaAVRs
; (Running at 16MHz/internal SRAM)
;
;  Points:   Input, Execute,  Output,    Total:  Throughput
;   64pts:   .17ms,   2.0ms,   1.2ms,    3.4ms:   19.0kpps
;  128pts:   .33ms,   4.6ms,   2.4ms,    7.3ms:   17.5kpps
;  256pts:   .66ms,  10.4ms,   4.9ms,   15.9ms:   16.1kpps
;  512pts:   1.3ms,  23.2ms,   9.7ms,   34.2ms:   14.9kpps
; 1024pts:   2.7ms,  51.7ms,  19.4ms,   73.7ms:   13.9kpps
;----------------------------------------------------------------------------;
*/

#define FFT_N 128
#define MAX_FREQ 100
#define SAMPLE_INTERVAL_MICROS (5000000 / MAX_FREQ)      //1e6 us / (2 * MAX_FREQ)        /////The 2 * is because we must sample at twice the desired rate of resolution.

typedef struct led_channel
{
  int pin;
  int dmx_address;
  int frequency;
  boolean is_strobing;
  boolean is_on;
} led_channel_t;



//////////////////////////////////////////////////// GLOBALS //////////////////////////////////////////////////// 

//For program state
led_channel_t channels[NUM_LED_CHANNELS];

//For dmx
int dmx_address = 0;
dmx_mode_t dmx_mode = auto_sound_strobe;
unsigned long last_dmx_low_micros = 0;
volitile boolean in_dmx_packet = false;
byte dmx_values[NUM_DMX_CHANNELS];

//For sampling/FFT
unsigned long next_sample_micros = 0;
int sample_index = 0;
int16_t samples[FFT_N << 1];          //double length buffer for countinuous FFTs
complex_t fft_buff[FFT_N];	      //FFT buffer
uint16_t spectrum[FFT_N >> 1];	      //Spectrum output buffer
/*
Activate the DIP switch and read the starting DMX address.
*/
void setup_dmx()
{
  int address_pins[NUM_ADDRESS_PINS] = ADDRESS_PINS;
  int dmx_address = 0;
  for (int i = 0; i < NUM_ADDRESS_PINS; i++)
  {
    pinMode(address_pins[i], INPUT);
    digitalWrite(address_pins[i], HIGH);  //enable 20K pullup resistor.  The DIP should have 1K <= R <= 10K resistor to ground afterwards, so if DIP is closed, then the read will show LOW.
    if (digitalRead(address_pins[i]) == LOW)
    {
      dmx_address |= (1 << i);
    }
    //turn off pullup resistor and repair pin mode.
    digitalWrite(address_pins[i], LOW);
    pinMode(address_pins[i], OUTPUT);
  }
  
  for (int i = 0; i < NUM_DMX_CHANNELS; i++)
  {
    dmx_values[i] = 0;
  }
}

void build_channels()
{
  int led_pins[NUM_LED_CHANNELS] = LED_PINS;
  for (int i = 0; i < NUM_LED_CHANNELS; i++)
  {
    channels[i].pin = led_pins[i];
    pinMode(channels[i].pin, OUTPUT);
    digitalWrite(channels[i].pin, LOW);
    channels[i].dmx_address = dmx_address + DMX_CHANNEL_FIRST_LED + i;
    channels[i].frequency = -1;
    channels[i].is_strobing = false;
    channels[i].is_on = false;
  }
}

/*
value is pointer to byte to read into.
Return false if successful, true if error.
*/
boolean read_dmx_byte(byte * value)
{
  *value = 0;
  for (int i = 0; i < DMX_BITS_IN_BOX; i++)
  {
    delayMicroseconds(DMX_BIT_MICROS);
    if (digitalRead(DMX_IN_PIN) == HIGH)
    {
      *value |= (1 << i);
    }
  }
  //stop bit
  delayMicroseconds(DMX_BIT_MICROS);
  if (digitalRead(DMX_IN_PIN) != HIGH) return true;
  //release bit
  delayMicroseconds(DMX_BIT_MICROS);
  if (digitalRead(DMX_IN_PIN) != HIGH) return true;
  
  //success
  return false;
}

/*
Return false if successful, true if error.
*/
boolean read_dmx_packet()
{
  byte box = 0;
  //read start code
  if (read_dmx_byte(&box)) return true;
  
  for (int channel = 0; channel < DMX_BOXES_PER_PACKET; channel++)
  {
    if (read_dmx_byte(&box)) return true;
    if (channel >= dmx_address && channel < dmx_address + NUM_DMX_CHANNELS)
    {
      dmx_values[channel - dmx_address] = box;
    }
  }
  return false;
}
   

void read_dmx()
{
  if (in_dmx_packet) return;
  unsigned long mark_start = micros();
  if (mark_start - last_dmx_low_micros < DMX_BREAK_MICROS) return;    //break wasn't long enough for this to be a mark, so we're in the middle, or something's mangled
  in_dmx_packet = true;
  //read mark
  unsigned long now = micros();
  int last_bit = digitalRead(DMX_IN_PIN);
  while (last_bit == HIGH && now - mark_start < DMX_MICROS_BETW_BREAKS)
  {
    now = micros();
    last_bit = digitalRead(DMX_IN_PIN);
    delayMicroseconds(DMX_BIT_MICROS);
  }
  now = micros();
  if (last_bit == HIGH || now - mark_start < DMX_MARK_MICROS)
  {
    //timeout in mark, fail
    last_dmx_low_micros = now;
    in_dmx_packet = false;
    return;
  }
  //ok, no we're in the first start bit, get ready to receive some packets.
  if (read_dmx_packet()) return;
  last_dmx_low_micros = micros();
  in_dmx_packet = false;
}

void setup()
{
  pinMode(AUDIO_PIN, INPUT);
  pinMode(DMX_IN_PIN, INPUT);
  setup_dmx();
  build_channels();
  attachInterrupt(DMX_IN_INTERRUPT, read_dmx, RISING);
}

/*
Analyze dmx_values
*/
void check_mode()
{
  //TODO
}

/*
capture a sample from the audio source.
*/
void sample()
{
  unsigned long now = micros();
  while (now < next_sample_micros)
  {
    delayMillis(next_sample_micros - now);
    now = micros();
  }
  next_sample_micros = now + SAMPLE_INTERVAL_MICROS;  
  
  sample_index++;
  if (sample_index >= FFT_N)
  {
    sample_index = 0;
  }
  samples[sample_index] = analogRead(SIGNAL_PIN) - DC_OFFSET;
  samples[sample_index + FFT_N] = samples[sample_index];
}

/*
Turn on/off whatever LED's are relevant
*/
void strobe()
{
  //TODO
}

/*
Do whatever step is next for fft (if applicable)
*/
void analyze()
{
  //TODO
}

void loop()
{
  check_mode();
  sample();
  strobe();
  analyze();
}
