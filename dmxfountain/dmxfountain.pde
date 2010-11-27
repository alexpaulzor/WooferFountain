#include "dmxfountain.h"

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
    channels[i].dmx_channel = DMX_CHANNEL_FIRST_LED + i;
    channels[i].frequency = -1;
    channels[i].is_on = false;
    channels[i].last_on = 0;
    channels[i].max_intensity = 0;
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
 *
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
*/
void set_dmx_mode(dmx_mode_t new_dmx_mode)
{
 //TODO: special stuff by mode

 dmx_mode = new_dmx_mode;
}

/*
Analyze dmx_values
*/
void check_mode()
{
  dmx_mode_t cur_mode;

  byte modevalue = dmx_values[DMX_CHANNEL_MODE];

  if (modevalue < auto_sound_strobe) cur_mode = auto_sound_strobe;
  else if (modevalue < auto_sound_solid) cur_mode = auto_sound_solid;
  else if (modevalue < auto_sound_chase) cur_mode = auto_sound_chase;
  else if (modevalue < dmx_sound_chase) cur_mode = dmx_sound_chase;
  else if (modevalue < dmx_sound_strobe) cur_mode = dmx_sound_strobe;
  else if (modevalue < dmx_sound_solid) cur_mode = dmx_sound_solid;
  else if (modevalue < dmx_chase) cur_mode = dmx_chase;
  else if (modevalue < dmx_manual) cur_mode = dmx_manual;
  else cur_mode = dmx_mode;
  
  if (cur_mode != dmx_mode)
  {
   set_dmx_mode(cur_mode);
  }
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
 * auto_sound_* relies on frequency field, fft
 * dmx_sound_* sets frequency field based on slider, relies on fft
 * dmx_manual_* relies on slider
 */
void update_leds()
{
  unsigned long now = micros();
  for (int i = 0; i < NUM_LED_CHANNELS; i++)
  {
    if (InDmxMode && InSoundMode)
    {
      channels[i].frequency = DmxValue2Band(dmx_values[channels[i].dmx_channel]);
    }
    if (InSoundMode && spectrum[channels[i].frequency] < SPECTRUM_THRESHOLD)
    {
     //signal not present
     digitalWrite(channels[i].pin, LOW);
     channels[i].is_on = false;
     //TODO: chases
    }
    else
    {
      if (InStrobeMode)
      {
        if (channels[i].is_on)
        {
          if (now > channels[i].last_on + STROBE_DURATION)
          {
           digitalWrite(channels[i].pin, LOW);
           channels[i].is_on = false;
          }
        }
        else
        {
         unsigned long delay;
         if (InDmxMode) delay = DmxValue2Micros(dmx_values[channels[i].dmx_channel]);
         else delay = Band2Micros(channels[i].frequency);
         if (now > channels[i].last_on + delay)
         {
           digitalWrite(channels[i].pin, HIGH);
           channels[i].is_on = true;
           channels[i].last_on = now;
         }
        }
      }
      else
      {
       //solid mode
       digitalWrite(channels[i].pin, HIGH);
       channels[i].is_on = true;
       channels[i].last_on = now;
      }
    }
  }
}


/*
 * Find the channel using given frequency, or NULL if none is.
 */
led_channel_t * find_channel(int frequency)
{
  for (int i = 0; i < NUM_LED_CHANNELS; i++)
  {
    if (channels[i].frequency == frequency)
    {
     return &(channels[i]);
    }
  }

  return NULL;
}

/*
 * Find the weakest channel.
 */
led_channel_t * weakest_channel()
{
  int weakest_index = 0;
  for (int i = 1; i < NUM_LED_CHANNELS; i++)
  {
    if (channels[i].max_intensity < channels[weakest_index].max_intensity)
    {
     weakest_index = i;
    }
  }

  return &(channels[weakest_index]);
}

/*
 * Find the oldest channel.
 */
led_channel_t * oldest_channel()
{
  int oldest_index = 0;
  for (int i = 1; i < NUM_LED_CHANNELS; i++)
  {
    if (channels[i].last_on < channels[oldest_index].last_on)
    {
     oldest_index = i;
    }
  }

  return &(channels[oldest_index]);
}

/*
 * Process new fft data
 */
void fft_complete()
{
  //if not automatically assigning frequencies, this does nothing
  if (!InSoundMode) return;

  //now we need to go through the spectrum and identify frequencies to react to.
  for (int i = 0; i < SPECTRUM_SIZE; i++)
  {
    //if signal is too small, it doesn't count.
    if (spectrum[i] < FFT_THRESHOLD) continue;

   //first, look if this one is already assigned to an led_channel
   led_channel_t * channel = find_channel(i);
   if (channel != NULL)
   {
    channel -> last_seen = micros();
    if (spectrum[i] > channel -> max_intensity)
    {
      channel -> max_intensity = spectrum[i];
    }
    continue;
   }
   
   //not assigned already, find the oldest channel.
   channel = oldest_channel();
   //if not old enough, find the weakest channel.
   if ((micros() - (channel -> last_on)) < CHANNEL_TTL)
   {
     channel = weakest_channel();
     // if weakest channel is too strong, no open channel to use.
     if (channel -> max_intensity > spectrum[i])
     {
      return;
     }
   }

   channel -> frequency = i;
   channel -> max_intensity = spectrum[i];
  }
}

/*
Do whatever step is next for fft (if applicable)
*/
void analyze()
{
  // these two don't use fft
  if (dmx_mode == dmx_chase || dmx_mode == dmx_manual) return;

  if (next_fft_op == fft_op_input)
  {
   fft_input(&(samples[sample_index]), fft_buff);
   next_fft_op = fft_op_execute;
  }
  else if (next_fft_op == fft_op_execute)
  {
    fft_execute(fft_buff);
    next_fft_op = fft_op_output;
  }
  else
  {
    fft_output(fft_buff, spectrum);
    next_fft_op = fft_op_input;
    fft_complete();
  }
}

void loop()
{
  check_mode();
  sample();
  analyze();
  update_leds();
}

// vim: set filetype=c:
