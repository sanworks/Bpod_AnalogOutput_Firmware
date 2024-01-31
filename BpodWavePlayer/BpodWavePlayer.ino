/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// **NOTE** previous versions of this firmware required dependencies and modifications to the Teensy core files. 
//          As of firmware v3, these are no longer necessary.
// **NOTE** Requires Arduino 2.2.1 or newer, with Teensy board package 1.58.1 or newer
// **NOTE** As of firmware v4, channel count is set in the setup macros below. Separate firmware for the 8ch board is obsolete.

#include "ArCOM.h"
#include <SPI.h>
#include "SdFat.h"

#define FIRMWARE_VERSION 6

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 2 // Use: 1 = AOM rev 1.0-1.4 (as marked on PCB), 2 = AOM rev 2.0
#define NUM_CHANNELS 4 // Use: 4 for 4-channel AOM, 8 for 8-channel AOM
//-------------------------------------------

// Validate macros
#if (HARDWARE_VERSION < 1) || (HARDWARE_VERSION > 2)
#error Error! HARDWARE_VERSION must be either 1 or 2
#endif

#if !(NUM_CHANNELS == 4 || NUM_CHANNELS == 8)
#error Error! NUM_CHANNELS must be either 4 or 8
#endif

// SD objects
SdFs SDcard;
FsFile Wave0; // File on microSD card, to store waveform data
bool ready = false; // Indicates if SD is busy (for use with SDBusy() funciton)

// Module setup
char moduleName[] = "WavePlayer"; // Name of module for manual override UI and state machine assembler
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer
byte circuitRevision = 0; // Circuit board revision, encoded as bits in an array of pins; 1 = grounded, 0 = floating (no connection)
const byte circuitRevisionArray[5] = {25,26,27,28,29};

// Parameters
#if HARDWARE_VERSION == 2
  const byte maxWaves = 128; // Maximum number of waveforms (used to set up data buffers and to ensure data file is large enough)
  const byte maxTriggerProfiles = 128; // Maximum number of trigger profiles (vectors of waves to play on each channel for different trigger bytes) 
#else
  const byte maxWaves = 64; // Maximum number of waveforms (used to set up data buffers and to ensure data file is large enough)
  const byte maxTriggerProfiles = 64; // Maximum number of trigger profiles (vectors of waves to play on each channel for different trigger bytes) 
#endif
const unsigned long bufSize = 1280; // Buffer size (in samples). Larger buffers prevent underruns, but take up memory.
                                    // Each wave in MaxWaves is allocated 1 buffer worth of sRAM (Teensy 3.6 total sRAM = 256k)
const unsigned long maxWaveSize = 1000000; // Maximum number of samples per waveform
union {
    byte byteArray[4];
    float floatVal;
} timerPeriod; // Default hardware timer period during playback, in microseconds (100 = 10kHz). Set as a Union so it can be read as bytes.
float timerPeriod_Idle = 20; // Default hardware timer period while idle (no playback, awaiting commands; determines playback latency)

// Pin definitions

#if NUM_CHANNELS == 4
  byte RefEnable1 = 32; // External 3V reference enable pin
  byte SyncPin1=14; // AD5754 Pin 7 (Sync)
  byte LDACPin1=39; // AD5754 Pin 10 (LDAC)
#else
  byte RefEnable1 = 31; // External 3V reference enable pin
  byte SyncPin1=34; // AD5754 Pin 7 (Sync)
  byte LDACPin1=32; // AD5754 Pin 10 (LDAC)
  byte RefEnable2 = 33;
  byte SyncPin2=14;
  byte LDACPin2=39; 
#endif

// System objects
SPISettings DACSettings(30000000, MSBFIRST, SPI_MODE2); // Settings for DAC
IntervalTimer hardwareTimer; // Hardware timer to create even sampling

// Playback variables
byte opCode = 0; // Serial inputs access an op menu. The op code byte stores the intended operation.
byte opSource = 0; // 0 = op from USB, 1 = op from UART1, 2 = op from UART2. More op code menu options are exposed for USB.
boolean newOpCode = 0; // true if an opCode was read from one of the ports
byte inByte = 0; // General purpose byte for Serial read
byte BpodMessage = 0; // Stores the current message byte to send to the Bpod State Machine (playback start/stop event, bits indicate channels)
byte rangeIndex = 3; // 0 = '0V:5V', 1 = '0V:10V', 2 = '0V:12V', 3 = '-5V:5V', 4 = '-10V:10V', 5 = '-12V:12V' 
byte triggerMode = 0; // Triggers during playback are: 0 = ignored (default) 1 = handled (new waveform starts)  2 = stop playback
boolean triggerProfileEnable = false; // If enabled, each profile is a vector of 4 waveform indexes to play on output channels, on receiving the profile's trigger byte.
byte triggerProfiles[NUM_CHANNELS][maxTriggerProfiles] = {255}; // waves to play on each channel (trigger profile mode; 255 = no wave)
boolean currentTriggerChannels[NUM_CHANNELS] = {0}; // Vector of channels affected by current trigger event
byte currentTriggerWaves[NUM_CHANNELS] = {0}; // Vector of waves to play for current trigger event
unsigned long nSamples[maxWaves] = {0}; // Number of samples in each waveform
unsigned long currentSample[NUM_CHANNELS] = {0}; // Current position in waveform for each channel
byte waveLoaded[NUM_CHANNELS] = {0}; // Waveform currently loaded on each channel (loading is instant on trigger)
boolean playing[NUM_CHANNELS] = {false}; // True for each channel that is playing a waveform
boolean playbackActive = false; // True if any channel is playing a waveform
boolean playbackActiveLastCycle = false; // True if playback was active on the previous hardware timer callback (for resetting timer to idle refresh rate)
boolean schedulePlaybackStop[NUM_CHANNELS] = {0}; // Reminds the program to set playing = false after next DAC update
byte sendBpodEvents[NUM_CHANNELS] = {0}; // Sends a byte to Bpod state machine, to indicate playback start and stop. Bits of byte indicate which channels.
byte loopMode[NUM_CHANNELS] = {0}; // (for each channel) Loops waveform until loopDuration seconds
unsigned long loopDuration[NUM_CHANNELS] = {0}; // Duration of loop for loop mode (in samples)
unsigned long channelTime[NUM_CHANNELS] = {0}; // Time (in samples) since looping channel was triggered (used to compute looped playback end)
byte waveIndex = 0; // Index of current waveform (1-maxWaves; maxWaves is in the "parameters" section above)
byte channelIndex = 0; // Index of current output channel (1-4)
byte channelBits = 0; // Bits indicate channels to update
const unsigned long maxWaveSizeBytes = maxWaveSize*2; // Maximum size of a waveform in bytes (maxWaveSize is in the "parameters" section above)
const int bufSizeBytes = bufSize*2; // Size of the buffer in bytes (bufSizeBytes is in the "parameters" section above)
byte currentBuffer[NUM_CHANNELS] = {0}; // Current buffer for each channel (a double buffering scheme allows one to be filled while the other is read)
boolean loadFlag[NUM_CHANNELS] = {0}; // Set true when the buffer switches, to trigger filling of the empty buffer
int bufferPos[NUM_CHANNELS] = {0}; // Position of current sample in the current data buffer, for each output channel
unsigned long filePos[NUM_CHANNELS] = {0}; // Position of current sample in the data file, for each output channel
byte preBuffer[maxWaves][bufSizeBytes] = {0}; // The first buffer worth of each waveform is stored here on load, to achieve super-low latency playback.
boolean preBufferActive[NUM_CHANNELS] = {1}; // Each channel begins playback from the pre-buffer
unsigned short DACBits_ZeroVolts = 32768; // Code (in bits) for 0V. For bipolar ranges, this should be 32768. For unipolar, 0.
byte countdown2Play[NUM_CHANNELS] = {0}; // Set to 2 if a channel has been triggered, and needs to begin playback in 2 cycles. Set to 1 on next cycle, etc.
                                      // This ensures that a cycle burdened with serial reads and triggering logic does not also update the channel voltage.
                                      // The phenotype, if too few cycles are skipped, is a short first sample. 
uint16_t fixedVoltage = 0; // Fixed voltage to set on a set of target channels (ops 128-143)
volatile boolean usbLoadFlag = false;
volatile boolean trigProfileLoadFlag = false;
boolean dac2Active = true; // Used to disable DAC2 on the 8ch module if sampling rate is too high for both DACs
// Communication variables
const int BpodSerialBaudRate = 1312500; // Communication rate for Bpod UART channel
byte dacBuffer[3] = {0}; // Holds bytes to be written to the DAC
union {
    byte byteArray[NUM_CHANNELS*2];
    uint16_t uint16[NUM_CHANNELS];
} dacValue; // 16-Bit code of current sample on DAC output channels. A union type allows instant conversion between bytes and 16-bit ints
union {
    byte byteArray[2];
    uint16_t uint16[1];
} sdSample;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel1BufferA; // channel1BufferA and channel1BufferB form a double-buffer - one buffer fills while the other drives the DAC
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel1BufferB;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel2BufferA;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel2BufferB;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel3BufferA;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel3BufferB;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel4BufferA;
union {
    byte byteArray[bufSizeBytes];
    uint16_t uint16[bufSize];
} channel4BufferB;
#if NUM_CHANNELS == 8
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel5BufferA;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel5BufferB;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel6BufferA;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel6BufferB;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel7BufferA;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel7BufferB;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel8BufferA;
  union {
      byte byteArray[bufSizeBytes];
      uint16_t uint16[bufSize];
  } channel8BufferB;
#endif

// Wrap serial interfaces
ArCOM USBCOM(Serial); // Creates an ArCOM object called USBCOM, wrapping Serial (for Teensy 3.6)
#if HARDWARE_VERSION == 1
  ArCOM Serial1COM(Serial3); // Creates an ArCOM object called Serial1COM, wrapping Serial3
  ArCOM Serial2COM(Serial2); 
#else
  ArCOM Serial1COM(Serial1); // Creates an ArCOM object called Serial1COM, wrapping Serial1
  ArCOM Serial2COM(Serial2); 
#endif

// File transfer buffer
const unsigned long fileTransferBufferSize = 32000;
byte fileTransferBuffer[fileTransferBufferSize] = {0};
unsigned long nFullReads = 0;
unsigned long partialReadSize = 0;

void setup() {
  Serial2.begin(1312500);
  #if HARDWARE_VERSION == 1
    Serial3.begin(1312500);
    Serial3.addMemoryForRead(StateMachineSerialBuf, 192);
  #else
    Serial1.begin(1312500);
    Serial1.addMemoryForRead(StateMachineSerialBuf, 192);
  #endif
  // Read hardware revision from circuit board (an array of grounded pins indicates revision in binary, grounded = 1, floating = 0)
  circuitRevision = 0;
  for (int i = 0; i < 5; i++) {
    pinMode(circuitRevisionArray[i], INPUT_PULLUP);
    delay(1);
    circuitRevision += pow(2, i)*digitalRead(circuitRevisionArray[i]);
    pinMode(circuitRevisionArray[i], INPUT);
  }
  circuitRevision = 31-circuitRevision;  
  
  pinMode(RefEnable1, OUTPUT); // Reference enable pin sets the external reference IC output to 3V (RefEnable=high) or high impedence (RefEnable = low)
  digitalWrite(RefEnable1, LOW); // Disabling external reference IC allows other voltage ranges with DAC internal reference
  pinMode(SyncPin1, OUTPUT); // Configure SPI bus pins as outputs
  pinMode(LDACPin1, OUTPUT);
  #if NUM_CHANNELS == 8
    pinMode(RefEnable2, OUTPUT); // Reference enable pin sets the external reference IC output to 3V (RefEnable=high) or high impedence (RefEnable = low)
    digitalWrite(RefEnable2, LOW); // Disabling external reference IC allows other voltage ranges with DAC internal reference
    pinMode(SyncPin2, OUTPUT); // Configure SPI bus pins as outputs
    pinMode(LDACPin2, OUTPUT);
  #endif
  
  SDcard.begin(SdioConfig(FIFO_SDIO));
  SDcard.remove("Wave0.wfm");
  Wave0 = SDcard.open("Wave0.wfm", O_RDWR | O_CREAT);
  Wave0.preAllocate(maxWaves*maxWaveSize*2);
  while (sdBusy()) {}
  Wave0.seek(0);
  SPI.begin(); // Initialize SPI interface
  SPI.beginTransaction(DACSettings); // Set SPI parameters to DAC speed and bit order
  digitalWrite(LDACPin1, LOW); // Ensure DAC load pin is at default level (low)
  #if NUM_CHANNELS == 8
    digitalWrite(LDACPin2, LOW);
  #endif
  ProgramDAC(24, 0, 0); // NOP
  zeroDAC(); // Set all DAC channels to 0V
  ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
  ProgramDAC(12, 0, 3); // Set output range to +/- 5V
  zeroDAC(); // Set all DAC channels to 0V
  
  timerPeriod.floatVal = 100; // Set a default sampling rate (10kHz)
  hardwareTimer.begin(handler, timerPeriod.floatVal); // hardwareTimer is an interval timer object - Teensy 3.6's hardware timer
}

void loop() { // loop runs in parallel with hardware timer, at lower interrupt priority. Its function is to fill playback buffers from the microSD card.
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (playing[i]) {
      if (loadFlag[i]) {
        loadFlag[i] = 0;
        Wave0.seek(filePos[i]);
        switch(i) {
          case 0:
            if (currentBuffer[i] == 1) {
              Wave0.read(channel1BufferA.byteArray, bufSizeBytes);
            } else {
              Wave0.read(channel1BufferB.byteArray, bufSizeBytes);
            }
          break;
          case 1:
            if (currentBuffer[i] == 1) {
              Wave0.read(channel2BufferA.byteArray, bufSizeBytes);
            } else {
              Wave0.read(channel2BufferB.byteArray, bufSizeBytes);
            }
          break;
          case 2:
            if (currentBuffer[i] == 1) {
              Wave0.read(channel3BufferA.byteArray, bufSizeBytes);
            } else {
              Wave0.read(channel3BufferB.byteArray, bufSizeBytes);
            }
          break;
          case 3:
            if (currentBuffer[i] == 1) {
              Wave0.read(channel4BufferA.byteArray, bufSizeBytes);
            } else {
              Wave0.read(channel4BufferB.byteArray, bufSizeBytes);
            }
          break;
          #if NUM_CHANNELS == 8
            case 4:
              if (currentBuffer[i] == 1) {
                Wave0.read(channel5BufferA.byteArray, bufSizeBytes);
              } else {
                Wave0.read(channel5BufferB.byteArray, bufSizeBytes);
              }
            break;
            case 5:
              if (currentBuffer[i] == 1) {
                Wave0.read(channel6BufferA.byteArray, bufSizeBytes);
              } else {
                Wave0.read(channel6BufferB.byteArray, bufSizeBytes);
              }
            break;
            case 6:
              if (currentBuffer[i] == 1) {
                Wave0.read(channel7BufferA.byteArray, bufSizeBytes);
              } else {
                Wave0.read(channel7BufferB.byteArray, bufSizeBytes);
              }
            break;
            case 7:
              if (currentBuffer[i] == 1) {
                Wave0.read(channel8BufferA.byteArray, bufSizeBytes);
              } else {
                Wave0.read(channel8BufferB.byteArray, bufSizeBytes);
              }
            break;
          #endif
        }
        filePos[i] += bufSizeBytes;
      }
    }
  }
  if (usbLoadFlag) {
    loadWaveform();
    usbLoadFlag = false;
  }
  if (trigProfileLoadFlag) {
    loadTriggerProfiles();
    trigProfileLoadFlag = false;
  }
}

void handler(){ // The handler is triggered precisely every timerPeriod microseconds. It processes serial commands and playback logic.
  if (Serial1COM.available() > 0) {
    opCode = Serial1COM.readByte(); // Read in an op code
    opSource = 1; // UART 1
    newOpCode = true;
  } else if (Serial2COM.available() > 0) {
    opCode = Serial2COM.readByte();
    opSource = 2; // UART 2
    newOpCode = true;
  } else if (USBCOM.available() > 0) {
    if (!usbLoadFlag && !trigProfileLoadFlag) {
      opCode = USBCOM.readByte();
      opSource = 0; // USB
      newOpCode = true;
    }
  }
  if (newOpCode) { // If an op byte arrived from one of the serial interfaces
    newOpCode = false;
    switch(opCode) {
      case 227: // Handshake
        if (opSource == 0) {
          USBCOM.writeByte(228); // Unique reply-byte
          USBCOM.writeUint32(FIRMWARE_VERSION); // Send firmware version
        }
      break;

      case 255: // Return Bpod module info
        if (opSource == 1) { // Only returns this info if requested from state machine device
          returnModuleInfo();
        }
      break;

      case 254: // Relay test byte from USB to echo module, or from echo module back to USB 
        if (opSource == 0) {
          Serial2COM.writeByte(254);
        }
        if (opSource == 2) {
          USBCOM.writeByte(254);
        }
      break;

      case 'H': // Return hardware version information
        USBCOM.writeByte(HARDWARE_VERSION);
        USBCOM.writeByte(circuitRevision);
      break;

      case 'N': // Return hardware setup params
        if (opSource == 0){
          USBCOM.writeByte(NUM_CHANNELS);
          USBCOM.writeUint16(maxWaves);
          USBCOM.writeByte(maxTriggerProfiles);
        }
      break;

      case 'O': // Set loop mode (for each channel)
      if (opSource == 0) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
          loopMode[i] = USBCOM.readByte();
        }
        USBCOM.writeByte(1); // Acknowledge
      }
      break;

      case 'D': // Set loop duration (for each channel)
      if (opSource == 0) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
          loopDuration[i] = USBCOM.readUint32();
        }
        USBCOM.writeByte(1); // Acknowledge
      }
      break;

      case 'V': // Set Bpod event reporting (for each channel)
      if (opSource == 0){
        for (int i = 0; i < NUM_CHANNELS; i++) {
          sendBpodEvents[i] = USBCOM.readByte();
        }
        USBCOM.writeByte(1); // Acknowledge
      }
      break;

      case 'T': // Set trigger mode
        if (opSource == 0){
           triggerMode = USBCOM.readByte(); 
           USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'B': // Set trigger profile enable
        if (opSource == 0){
           triggerProfileEnable = USBCOM.readByte();
           USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'F': // Load trigger profiles
        if (opSource == 0) {
          #if HARDWARE_VERSION == 1
            loadTriggerProfiles();
          #else
            trigProfileLoadFlag = true;
          #endif
        }
      break;

      case 'Y': // Depricated function to set up data file on microSD card, with enough space to store all waveforms
        if (opSource == 0) {
          // ***NOTE*** Firmware now runs microSD setup on boot. This op's ack byte is retained for backwards compatability with old code.
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'R': // Set DAC range (for all outputs)
        if (opSource == 0) {
          rangeIndex = USBCOM.readByte(); // rangeIndex 0 = '0V:5V', 1 = '0V:10V', 2 = '0V:12V', 3 = '-5V:5V', 4 = '-10V:10V', 5 = '-12V:12V'
          switch(rangeIndex) {
            case 0:
              digitalWrite(RefEnable1, LOW); // Disable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, LOW);
              #endif
              ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
              ProgramDAC(12, 0, 0); // Set output range to 0-5V
              DACBits_ZeroVolts = 0; // Update 0V bit code
            break;
            case 1:
              digitalWrite(RefEnable1, LOW); // Disable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, LOW);
              #endif
              ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
              ProgramDAC(12, 0, 1); // Set output range to 0-10V
              DACBits_ZeroVolts = 0; // Update 0V bit code
            break;
            case 2:
              ProgramDAC(16, 0, 15); // Power up all channels without internal reference
              digitalWrite(RefEnable1, HIGH); // Enable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, HIGH);
              #endif
              ProgramDAC(12, 0, 1); // Set output range to 0-10V (using external ref, this setting = 0-12V)
              DACBits_ZeroVolts = 0; // Update 0V bit code
            break;
            case 3:
              digitalWrite(RefEnable1, LOW); // Disable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, LOW);
              #endif
              ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
              ProgramDAC(12, 0, 3); // Set output range to +/- 5V
              DACBits_ZeroVolts = 32768; // Update 0V bit code
            break;
            case 4:
              digitalWrite(RefEnable1, LOW); // Disable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, LOW);
              #endif
              ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
              ProgramDAC(12, 0, 4); // Set output range to +/- 10V
              DACBits_ZeroVolts = 32768; // Update 0V bit code
            break;
            case 5:
              ProgramDAC(16, 0, 15); // Power up all channels without internal reference
              digitalWrite(RefEnable1, HIGH); // Enable external reference IC
              #if NUM_CHANNELS == 8
                digitalWrite(RefEnable2, HIGH);
              #endif
              ProgramDAC(12, 0, 4); // Set output range to +/- 10V (using external ref, this setting = +/- 12V)
              DACBits_ZeroVolts = 32768; // Update 0V bit code
            break;
          }
          zeroDAC();
          USBCOM.writeByte(1); // Acknowledge
        }
      break;

      case 'L': // Load sound
        if (opSource == 0) {
          #if HARDWARE_VERSION == 1
            loadWaveform();
          #else
            usbLoadFlag = true;
          #endif
        }
      break;

      case 'P': // Play a waveform (1 max; any subset of channels)
        if (triggerProfileEnable) {
          switch(opSource) {
            case 0:
              inByte = USBCOM.readByte(); // Profile ID to trigger
            break;
            case 1:
              inByte = Serial1COM.readByte();
            break;
            case 2:
              inByte = Serial2COM.readByte();
            break;
          }
          for (int i = 0; i<NUM_CHANNELS; i++) {
            currentTriggerWaves[i] = triggerProfiles[i][inByte];
            currentTriggerChannels[i] = 0;
            if (currentTriggerWaves[i] != 255) {
              currentTriggerChannels[i] = 1;
            }
          }
        } else {
          switch(opSource) {
            case 0:
              channelIndex = USBCOM.readByte(); // Bits specifying channels to trigger
              waveIndex = USBCOM.readByte();
            break;
            case 1:
              channelIndex = Serial1COM.readByte(); // Bits specifying channels to trigger
              waveIndex = Serial1COM.readByte();
            break;
            case 2:
              channelIndex = Serial2COM.readByte(); // Bits specifying channels to trigger
              waveIndex = Serial2COM.readByte();
            break;
          }
          for (int i = 0; i<NUM_CHANNELS; i++) {
            if bitRead(channelIndex, i) {
              currentTriggerChannels[i] = 1;
              currentTriggerWaves[i] = waveIndex;
            } else {
              currentTriggerChannels[i] = 0;
            }
          }
        }
        triggerNewWaveforms(); // Triggers waves in currentTriggerWaves[] on channels currentTriggerChannels[]
      break;

      case '>': // Play a list of waveforms on specific channels
        switch(opSource) {
            case 0:
              USBCOM.readByteArray(currentTriggerWaves, NUM_CHANNELS);
            break;
            case 1:
              Serial1COM.readByteArray(currentTriggerWaves, NUM_CHANNELS);
            break;
            case 2:
              Serial2COM.readByteArray(currentTriggerWaves, NUM_CHANNELS);
            break;
          }
          for (int i = 0; i < NUM_CHANNELS; i++) {
            currentTriggerChannels[i] = 0;
            if (currentTriggerWaves[i] <  maxWaves) {
              currentTriggerChannels[i] = 1;
            }
          }
          triggerNewWaveforms(); // Triggers waves in currentTriggerWaves[] on channels currentTriggerChannels[]
      break;

      case 'S': // Set sampling rate
      if (opSource == 0) {
        USBCOM.readByteArray(timerPeriod.byteArray, 4);
        dac2Active = true;
        if (timerPeriod.floatVal < 33) {
          dac2Active = false; // Disable DAC2 on 8-ch module if sampling rate is too fast
        } 
        hardwareTimer.end();
        hardwareTimer.begin(handler, timerPeriod.floatVal);
      }
      break;

      case 'X': // Stop all playback
        zeroDAC();
      break;

      case '!': // Set a fixed voltage on selected output channels
        switch(opSource) {
          case 0:
            channelBits = USBCOM.readByte();
            fixedVoltage = USBCOM.readUint16(); // Voltage bits
          break;
          case 1:
            channelBits = Serial1COM.readByte();
            fixedVoltage = Serial1COM.readUint16(); // Voltage bits
          break;
        }
        setFixedOutput(channelBits, fixedVoltage);
      break;

      case 129 ... 143: // Legacy op: Set a fixed voltage on output channels indicated by lowest 4 bits of op code(will be overridden by next call to play a waveform on the same channel(s))
        switch(opSource) {
          case 0:
            fixedVoltage = USBCOM.readUint16(); // Voltage bits
          break;
          case 1:
            fixedVoltage = Serial1COM.readUint16(); // Voltage bits
          break;
        }
        setFixedOutput(opCode - 128, fixedVoltage);
      break;
    }
  }
  playbackActive = false;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (playing[i]) {
      playbackActive = true;
      if (preBufferActive[i]) {
        dacValue.uint16[i] = word(preBuffer[waveLoaded[i]][bufferPos[i]+1], preBuffer[waveLoaded[i]][bufferPos[i]]);
      } else {
        switch(i) {
          case 0:
          if (currentBuffer[i] == 0) {
            dacValue.uint16[i] = channel1BufferA.uint16[bufferPos[i]];
          } else {
            dacValue.uint16[i] = channel1BufferB.uint16[bufferPos[i]];
          }
          break;
          case 1:
          if (currentBuffer[i] == 0) {
            dacValue.uint16[i] = channel2BufferA.uint16[bufferPos[i]];
          } else {
            dacValue.uint16[i] = channel2BufferB.uint16[bufferPos[i]];
          }
          break;
          case 2:
          if (currentBuffer[i] == 0) {
            dacValue.uint16[i] = channel3BufferA.uint16[bufferPos[i]];
          } else {
            dacValue.uint16[i] = channel3BufferB.uint16[bufferPos[i]];
          }
          break;
          case 3:
          if (currentBuffer[i] == 0) {
            dacValue.uint16[i] = channel4BufferA.uint16[bufferPos[i]];
          } else {
            dacValue.uint16[i] = channel4BufferB.uint16[bufferPos[i]];
          }
          break;
          #if NUM_CHANNELS == 8
            case 4:
              if (currentBuffer[i] == 0) {
                dacValue.uint16[i] = channel5BufferA.uint16[bufferPos[i]];
              } else {
                dacValue.uint16[i] = channel5BufferB.uint16[bufferPos[i]];
              }
              break;
              case 5:
              if (currentBuffer[i] == 0) {
                dacValue.uint16[i] = channel6BufferA.uint16[bufferPos[i]];
              } else {
                dacValue.uint16[i] = channel6BufferB.uint16[bufferPos[i]];
              }
              break;
              case 6:
              if (currentBuffer[i] == 0) {
                dacValue.uint16[i] = channel7BufferA.uint16[bufferPos[i]];
              } else {
                dacValue.uint16[i] = channel7BufferB.uint16[bufferPos[i]];
              }
              break;
              case 7:
              if (currentBuffer[i] == 0) {
                dacValue.uint16[i] = channel8BufferA.uint16[bufferPos[i]];
              } else {
                dacValue.uint16[i] = channel8BufferB.uint16[bufferPos[i]];
              }
            break;
          #endif
        }
      }
      if (preBufferActive[i]) {
        bufferPos[i]+= 2;
        if (bufferPos[i] >= bufSizeBytes) {
          preBufferActive[i] = false;
          currentBuffer[i] = 1-currentBuffer[i];
          bufferPos[i] = 0;
          loadFlag[i] = 1;
        }
      } else {
        bufferPos[i]++;
        if (bufferPos[i] >= bufSize) {
          currentBuffer[i] = 1-currentBuffer[i];
          bufferPos[i] = 0;
          loadFlag[i] = 1;
        }
      }
      currentSample[i]++;
      channelTime[i]++;
      if (currentSample[i] == nSamples[waveLoaded[i]]) {
        if (loopMode[i]) {
          resetChannel(i);
        }
      } else if (currentSample[i] == nSamples[waveLoaded[i]]+1) {
        schedulePlaybackStop[i] = true;
        filePos[i] = maxWaveSize*waveLoaded[i];
        dacValue.uint16[i] = DACBits_ZeroVolts;
      }
      if (loopMode[i]) {
        if (channelTime[i] > loopDuration[i]) {
          schedulePlaybackStop[i] = true;
          filePos[i] = maxWaveSize*waveLoaded[i];
          dacValue.uint16[i] = DACBits_ZeroVolts;
        }
      }
      if (schedulePlaybackStop[i]) {
        dacValue.uint16[i] = DACBits_ZeroVolts;
      }
    }
  }
  if (playbackActive) {
    if (playbackActiveLastCycle == false) {
        hardwareTimer.end();
        hardwareTimer.begin(handler, timerPeriod.floatVal);
    }
    dacWrite();
    BpodMessage = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (schedulePlaybackStop[i]) {
        playing[i] = false;
        schedulePlaybackStop[i] = false;
        if (sendBpodEvents[i]) {
          bitSet(BpodMessage, i); 
        }
      }
      if (countdown2Play[i]) {
        countdown2Play[i] = false;
        playing[i] = true;
      }
    }
    if (BpodMessage > 0) {
      Serial1COM.writeByte(BpodMessage);    
    }
    playbackActiveLastCycle = true;
  } else {
    if (playbackActiveLastCycle) {
      playbackActiveLastCycle = false;
      hardwareTimer.end();
      hardwareTimer.begin(handler, timerPeriod_Idle);
    }
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (countdown2Play[i] == 1) {
        countdown2Play[i] = 0;
        playing[i] = true;
        loadFlag[i] = 1;
      }
      if (countdown2Play[i] == 2) {
        countdown2Play[i] = 1;
      }
    }
  }
}

void ProgramDAC(byte Data1, byte Data2, byte Data3) {
  digitalWrite(LDACPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  dacBuffer[0] = Data1;
  dacBuffer[1] = Data2;
  dacBuffer[2] = Data3;
  SPI.transfer(dacBuffer,3);
  digitalWrite(SyncPin1,HIGH);
  digitalWrite(LDACPin1,LOW);
  #if NUM_CHANNELS == 8
    digitalWrite(LDACPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    dacBuffer[0] = Data1;
    dacBuffer[1] = Data2;
    dacBuffer[2] = Data3;
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin2,HIGH);
    digitalWrite(LDACPin2,LOW);
  #endif
}

void dacWrite() {
  digitalWrite(LDACPin1,HIGH);
  if (playing[0]) {
    digitalWrite(SyncPin1,LOW);
    dacBuffer[0] = 3;
    dacBuffer[1] = dacValue.byteArray[1];
    dacBuffer[2] = dacValue.byteArray[0];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin1,HIGH);
  }
  if (playing[1]) {
    digitalWrite(SyncPin1,LOW);
    dacBuffer[0] = 2;
    dacBuffer[1] = dacValue.byteArray[3];
    dacBuffer[2] = dacValue.byteArray[2];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin1,HIGH);
  }
  if (playing[2]) {
    digitalWrite(SyncPin1,LOW);
    dacBuffer[0] = 0;
    dacBuffer[1] = dacValue.byteArray[5];
    dacBuffer[2] = dacValue.byteArray[4];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin1,HIGH);
  }
  if (playing[3]) {
    digitalWrite(SyncPin1,LOW);
    dacBuffer[0] = 1;
    dacBuffer[1] = dacValue.byteArray[7];
    dacBuffer[2] = dacValue.byteArray[6];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin1,HIGH);  
  }
  #if NUM_CHANNELS == 8
    if (dac2Active) {
      digitalWrite(LDACPin2,HIGH);
      if (playing[4]) {
        digitalWrite(SyncPin2,LOW);
        dacBuffer[0] = 3;
        dacBuffer[1] = dacValue.byteArray[9];
        dacBuffer[2] = dacValue.byteArray[8];
        SPI.transfer(dacBuffer,3);
        digitalWrite(SyncPin2,HIGH);
      }
      if (playing[5]) {
        digitalWrite(SyncPin2,LOW);
        dacBuffer[0] = 2;
        dacBuffer[1] = dacValue.byteArray[11];
        dacBuffer[2] = dacValue.byteArray[10];
        SPI.transfer(dacBuffer,3);
        digitalWrite(SyncPin2,HIGH);
      }
      if (playing[6]) {
        digitalWrite(SyncPin2,LOW);
        dacBuffer[0] = 0;
        dacBuffer[1] = dacValue.byteArray[13];
        dacBuffer[2] = dacValue.byteArray[12];
        SPI.transfer(dacBuffer,3);
        digitalWrite(SyncPin2,HIGH);
      }
      if (playing[7]) {
        digitalWrite(SyncPin2,LOW);
        dacBuffer[0] = 1;
        dacBuffer[1] = dacValue.byteArray[15];
        dacBuffer[2] = dacValue.byteArray[14];
        SPI.transfer(dacBuffer,3);
        digitalWrite(SyncPin2,HIGH); 
      }
      digitalWrite(LDACPin2,HIGH); 
      digitalWriteFast(LDACPin2,LOW);
    }
  #else
    digitalWrite(LDACPin1,HIGH); // In 4ch model, this line adds a required delay, added already in 8ch above by  digitalWrite(LDACPin2,HIGH);
  #endif
  digitalWriteFast(LDACPin1,LOW); 
}

void zeroDAC() {
  // Set DAC to resting voltage on all channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    dacValue.uint16[i] = DACBits_ZeroVolts;
    playing[i] = true; // Temporarily set all channels to play-enable, so they get updated
  }
  dacWrite(); // Update the DAC, to set all channels to mid-range (0V)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    playing[i] = false;
  }
}

void triggerChannel(byte channel, byte waveIndex) {
      waveLoaded[channel] = waveIndex; 
      currentSample[channel] = 0;
      channelTime[channel] = 0;
      countdown2Play[channel] = 2;
      currentBuffer[channel] = 1;
      bufferPos[channel] = 0;
      preBufferActive[channel] = true;
      filePos[channel] = (maxWaveSizeBytes*waveIndex) + bufSizeBytes;
}
void triggerChannelASAP(byte channel, byte waveIndex) { // In Master mode, swap waveforms immediately (no countdown2Play)
      waveLoaded[channel] = waveIndex; 
      currentSample[channel] = 0;
      channelTime[channel] = 0;
      playing[channel] = true;
      loadFlag[channel] = 1;
      currentBuffer[channel] = 1;
      bufferPos[channel] = 0;
      preBufferActive[channel] = true;
      filePos[channel] = (maxWaveSizeBytes*waveIndex) + bufSizeBytes;
}

void triggerNewWaveforms() {
  BpodMessage = 0;
  for (int i = 0; i<NUM_CHANNELS; i++) {
    if (currentTriggerChannels[i]) {
      switch(triggerMode) {
        case 0: // Normal mode: Trigger only if not already playing
          if (!playing[i]) {
            if (playbackActive) {
              triggerChannelASAP(i, currentTriggerWaves[i]);
            } else {
              triggerChannel(i, currentTriggerWaves[i]);
            }
            if (sendBpodEvents[i]) {
              bitSet(BpodMessage, i); 
            }
          }
        break;
        case 1: // Master Mode: Trigger even if already playing
          triggerChannelASAP(i, currentTriggerWaves[i]);
          if (sendBpodEvents[i]) {
              bitSet(BpodMessage, i); 
          }
        break;
        case 2:  // Toggle mode: Trigger stops channel if playing
          if (playing[i]) {
            schedulePlaybackStop[i] = true;
          } else {
            if (playbackActive) {
              triggerChannelASAP(i, currentTriggerWaves[i]);
            } else {
              triggerChannel(i, currentTriggerWaves[i]);
            }
          }
          if (sendBpodEvents[i]) {
              bitSet(BpodMessage, i); 
          }
        break;
      }
    }
  }
  if (BpodMessage > 0) {
    Serial1COM.writeByte(BpodMessage); 
  }
}

void resetChannel(byte channel) { // Resets playback to first sample (in loop mode)
      currentSample[channel] = 0;
      currentBuffer[channel] = 1;
      bufferPos[channel] = 0;
      loadFlag[channel] = 1;
      preBufferActive[channel] = true;
      filePos[channel] = (maxWaveSizeBytes*waveLoaded[channel]) + bufSizeBytes;
}

void loadTriggerProfiles() {
  for (int i = 0; i < NUM_CHANNELS; i++){
    for (int j = 0; j < maxTriggerProfiles; j++) {
      triggerProfiles[i][j] = USBCOM.readByte();
    }
  }
  USBCOM.writeByte(1); // Acknowledge
}

void setFixedOutput(byte targetChannelBits, uint16_t targetVoltage) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (bitRead(targetChannelBits, i)) {
        playing[i] = 1;
        dacValue.uint16[i] = targetVoltage;
      }
    }
    dacWrite();
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (bitRead(targetChannelBits,i)) {
        playing[i] = 0;
      }
    }
    if (opSource == 0) {
      USBCOM.writeByte(1);
    }
}

void loadWaveform() {
  waveIndex = USBCOM.readByte();
  if (waveIndex < maxWaves) { // Sanity check
    nSamples[waveIndex] = USBCOM.readUint32();
    while (sdBusy()) {}
    Wave0.seek(maxWaveSizeBytes*waveIndex);
    nFullReads = (unsigned long)(floor((double)nSamples[waveIndex]*2/(double)fileTransferBufferSize));
    for (int i = 0; i < nFullReads; i++) {
      while(Serial.available() == 0) {}
      Serial.readBytes((char*)fileTransferBuffer,fileTransferBufferSize);
      while (sdBusy()) {}
      Wave0.write(fileTransferBuffer,fileTransferBufferSize);
      if (i == 0) {
        memcpy(preBuffer[waveIndex], fileTransferBuffer, bufSizeBytes); 
      }
    }
    partialReadSize = (nSamples[waveIndex]*2)-(nFullReads*fileTransferBufferSize);
    if (partialReadSize > 0) {
      Serial.readBytes((char*)fileTransferBuffer,partialReadSize);
      while (sdBusy()) {}
      Wave0.write(fileTransferBuffer,partialReadSize);
      if (nFullReads == 0) {
        if ((nSamples[waveIndex]*2) > bufSizeBytes) {
          memcpy(preBuffer[waveIndex], fileTransferBuffer, bufSizeBytes); 
        } else {
          memcpy(preBuffer[waveIndex], fileTransferBuffer, nSamples[waveIndex]*2);  
        }
      }
    }         
    USBCOM.writeByte(1);
  }
}

bool sdBusy() {
  return ready ? SDcard.card()->isBusy() : false;
}

void returnModuleInfo() {
  boolean fsmSupportsHwInfo = false;
  delayMicroseconds(100);
  if (Serial1COM.available() == 1) { // FSM firmware v23 or newer sends a second info request byte to indicate that it supports additional ops
    if (Serial1COM.readByte() == 255) {fsmSupportsHwInfo = true;}
  }
  Serial1COM.writeByte(65); // Acknowledge
  Serial1COM.writeUint32(FIRMWARE_VERSION); // 4-byte firmware version
  Serial1COM.writeByte(sizeof(moduleName)-1); // Length of module name
  Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
  if (fsmSupportsHwInfo) {
    Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
    Serial1COM.writeByte('V'); // Op code for: Hardware major version
    Serial1COM.writeByte(HARDWARE_VERSION); 
    Serial1COM.writeByte(1); // 1 if more info follows, 0 if not
    Serial1COM.writeByte('v'); // Op code for: Hardware minor version
    Serial1COM.writeByte(circuitRevision); 
  }
  Serial1COM.writeByte(0); // 1 if more info follows, 0 if not
}
