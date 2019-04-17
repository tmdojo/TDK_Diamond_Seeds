#include <TDK_Diamond_Seeds.h>

// WaveForm Array: [Amplitude, Freq, Cycles, Envelope]
// Amplitude    --  min:0=50v max: 255=100v
// Frequerncy   --  0-255 or 0x00-0xFF
// Duration     --  Cycles 0-255
// Envelope     --  (Ramp up + down)
// Max 60 waves per array !!
// 
// Refer to DRV2667 datasheet for detail
// Table3. Waveform Chunk Bytes for Synthesizer on page 19
// http://www.ti.com/lit/ds/symlink/drv2667.pdf

byte WaveForm[1][4] = {
                      {255, 200, 50, 0x09}
                      };

void setup()
{
  TDSs.begin();
}

void loop(){
  TDSs.PlayWave(WaveForm, sizeof(WaveForm));
  delay(1000);
}
