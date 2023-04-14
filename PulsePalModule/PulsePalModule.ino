/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2022 Sanworks LLC, Rochester, New York, USA

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

// **NOTE** previous versions of this firmware required dependencies and modifications to the Teensy core files. As of firmware v3, these are no longer necessary.
// **NOTE** Requires Arduino 1.8.15 or newer, and Teensyduino 1.5.4 or newer
// **NOTE** As of firmware v4, channel count is set in the setup macros below. Separate firmware for the 8ch board is obsolete.

#include "ArCOM.h"
#include <SPI.h>

#define FIRMWARE_VERSION 4

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 0 // Use: 1 = AOM rev 1.0-1.4 (as marked on PCB), 2 = AOM rev 2.0
#define NUM_CHANNELS 4 // Use: 4 for 4-channel AOM, 8 for 8-channel AOM
//-------------------------------------------

// Validate macros
#if (HARDWARE_VERSION < 1) || (HARDWARE_VERSION > 2)
#error Error! HARDWARE_VERSION must be either 1 or 2
#endif

#if !(NUM_CHANNELS == 4 || NUM_CHANNELS == 8)
#error Error! NUM_CHANNELS must be either 4 or 8
#endif

// Module setup
char moduleName[] = "PulsePal"; // Name of module for the manual override UI and state machine assembler
byte CycleDuration = 100; // in microseconds, time between hardware cycles (each cycle = read trigger channels, update output channels)
byte circuitRevision = 0; // Circuit board revision, encoded as bits in an array of pins; 1 = grounded, 0 = floating (no connection)
const byte circuitRevisionArray[5] = {25,26,27,28,29};

// Create an ArCOM USB serial port object (to streamline transfer of different datatypes and arrays over serial)
byte StateMachineSerialBuf[192] = {0}; // Extra memory for state machine serial buffer

// Wrap serial interfaces
ArCOM USBCOM(Serial); // Creates an ArCOM object called USBCOM, wrapping Serial (for Teensy 3.6)
#if HARDWARE_VERSION == 1
  ArCOM Serial1COM(Serial3); // Creates an ArCOM object called Serial1COM, wrapping Serial3
  ArCOM Serial2COM(Serial2); 
#else
  ArCOM Serial1COM(Serial1); // Creates an ArCOM object called Serial1COM, wrapping Serial1
  ArCOM Serial2COM(Serial2); 
#endif

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
IntervalTimer hardwareTimer; // Hardware timer to ensure even sampling

// Parameters that define pulse trains currently loaded on the 4 output channels
// For a visual description of these parameters, see https://sites.google.com/site/pulsepalwiki/parameter-guide
// The following parameters are times in microseconds:
unsigned long Phase1Duration[NUM_CHANNELS] = {0}; // Pulse Duration in monophasic mode, first phase in biphasic mode
unsigned long InterPhaseInterval[NUM_CHANNELS] = {0}; // Interval between phases in biphasic mode (at resting voltage)
unsigned long Phase2Duration[NUM_CHANNELS] = {0}; // Second phase duration in biphasic mode
unsigned long InterPulseInterval[NUM_CHANNELS] = {0}; // Interval between pulses
unsigned long BurstDuration[NUM_CHANNELS] = {0}; // Duration of sequential bursts of pulses (0 if not using bursts)
unsigned long BurstInterval[NUM_CHANNELS] = {0}; // Interval between sequential bursts of pulses (0 if not using bursts)
unsigned long PulseTrainDuration[NUM_CHANNELS] = {0}; // Duration of pulse train
unsigned long PulseTrainDelay[NUM_CHANNELS] = {0}; // Delay between trigger and pulse train onset
// The following are volts in bits. 16 bits span -10V to +10V.
uint16_t Phase1Voltage[NUM_CHANNELS] = {0}; // The pulse voltage in monophasic mode, and phase 1 voltage in biphasic mode
uint16_t Phase2Voltage[NUM_CHANNELS] = {0}; // Phase 2 voltage in biphasic mode.
uint16_t RestingVoltage[NUM_CHANNELS] = {32768}; // Voltage the system returns to between pulses (32768 bits = 0V)
// The following are single byte parameters
byte CustomTrainID[NUM_CHANNELS] = {0}; // If 0, uses above params. If 1 or 2, pulse times and voltages are played back from CustomTrain1 or 2
byte CustomTrainTarget[NUM_CHANNELS] = {0}; // If 0, custom times define start-times of pulses. If 1, custom times are start-times of bursts.
byte CustomTrainLoop[NUM_CHANNELS] = {0}; // if 0, custom stim plays once. If 1, custom stim loops until PulseTrainDuration.
byte TriggerMode[2] = {0}; // if 0, "Normal mode", low to high transitions on trigger channels start stimulation (but do not cancel it) 
//                            if 1, "Toggle mode", same as normal mode, but low-to-high transitions do cancel ongoing pulse trains

// Variables used in programming
byte OpMenuByte = 213; // This byte must be the first byte in any serial transmission to Pulse Pal. Reduces the probability of interference from port-scanning software
unsigned long CustomTrainNpulses[4] = {0}; // Stores the total number of pulses in the custom pulse train
byte BrokenBytes[4] = {0}; // Used to store sequential bytes when converting bytes to short and long ints

// Variables used in stimulus playback
byte inByte; byte inByte2; byte inByte3; byte inByte4; byte CommandByte;
byte LogicLevel = 0;
unsigned long SystemTime = 0; // Number of cycles since stimulation start
unsigned long MicrosTime = 0; // Actual system time (microseconds from boot, wraps over every 72m
unsigned long BurstTimestamps[NUM_CHANNELS] = {0};
unsigned long PrePulseTrainTimestamps[NUM_CHANNELS] = {0};
unsigned long PulseTrainTimestamps[NUM_CHANNELS] = {0};
unsigned long NextPulseTransitionTime[NUM_CHANNELS] = {0}; // Stores next pulse-high or pulse-low timestamp for each channel
unsigned long NextBurstTransitionTime[NUM_CHANNELS] = {0}; // Stores next burst-on or burst-off timestamp for each channel
unsigned long PulseTrainEndTime[NUM_CHANNELS] = {0}; // Stores time the stimulus train is supposed to end
unsigned long CustomPulseTimes[4][10001] = {0};
uint16_t CustomVoltages[4][10001] = {0};
int CustomPulseTimeIndex[NUM_CHANNELS] = {0}; // Keeps track of the pulse number of the custom train currently being played on each channel
unsigned long LastLoopTime = 0;
byte PulseStatus[NUM_CHANNELS] = {0}; // This is 0 if not delivering a pulse, 1 if phase 1, 2 if inter phase interval, 3 if phase 2.
boolean BurstStatus[NUM_CHANNELS] = {0}; // This is "true" during bursts and false during inter-burst intervals.
boolean StimulusStatus[NUM_CHANNELS] = {0}; // This is "true" for a channel when the stimulus train is actively being delivered
boolean PreStimulusStatus[NUM_CHANNELS] = {0}; // This is "true" for a channel during the pre-stimulus delay
boolean UsesBursts[NUM_CHANNELS] = {0};
unsigned long PulseDuration[NUM_CHANNELS] = {0}; // Duration of a pulse (sum of 3 phases for biphasic pulse)
byte IsBiphasic[NUM_CHANNELS] = {0};
boolean IsCustomBurstTrain[NUM_CHANNELS] = {0};
byte ContinuousLoopMode[NUM_CHANNELS] = {0}; // If true, the channel loops its programmed stimulus train continuously
byte StimulatingState = 0; // 1 if ANY channel is stimulating, 2 if this is the first cycle after the system was triggered. 
byte LastStimulatingState = 0;
boolean WasStimulating = 0; // true if any channel was stimulating on the previous loop. Used to force a DAC write after all channels end their stimulation, to return lines to 0
int nStimulatingChannels = 0; // number of actively stimulating channels
boolean DACFlag = 0; // true if any DAC channel needs to be updated
byte DefaultInputLevel = 0; // 0 for PulsePal 0.3, 1 for 0.2 and 0.1. Logic is inverted by optoisolator

// Other variables
int ConnectedToApp = 0; // 0 if disconnected, 1 if connected
volatile boolean usbLoadFlag = false;
void handler(void);
boolean SoftTriggered[NUM_CHANNELS] = {0}; // If a software trigger occurred this cycle (for timing reasons, it is scheduled to occur on the next cycle)
boolean SoftTriggerScheduled[NUM_CHANNELS] = {0}; // If a software trigger is scheduled for the next cycle
unsigned long callbackStartTime = 0;
byte maxCommandByte = 0;
byte dacBuffer[3] = {0};
uint16_t DACBits_ZeroVolts = 32768;
union {
    byte byteArray[NUM_CHANNELS*2];
    uint16_t uint16[NUM_CHANNELS];
} dacValue;
union {
    byte byteArray[4];
    float floatVal;
} timerPeriod; // Default hardware timer period during playback, in microseconds (100 = 10kHz). Set as a Union so it can be read as bytes.

void setup() {
  pinMode(RefEnable1, OUTPUT); // Reference enable pin sets the external reference IC output to 3V (RefEnable1=high) or high impedence (RefEnable1 = low)
  digitalWrite(RefEnable1, LOW); // Disabling external reference IC allows other voltage ranges with DAC internal reference
  pinMode(SyncPin1, OUTPUT); // Configure SPI bus pins as outputs
  pinMode(LDACPin1, OUTPUT);
  #if NUM_CHANNELS == 8
    pinMode(RefEnable2, OUTPUT); // Reference enable pin sets the external reference IC output to 3V (RefEnable=high) or high impedence (RefEnable = low)
    digitalWrite(RefEnable2, LOW); // Disabling external reference IC allows other voltage ranges with DAC internal reference
    pinMode(SyncPin2, OUTPUT); // Configure SPI bus pins as outputs
    pinMode(LDACPin2, OUTPUT);
  #endif
  SPI.begin(); // Initialize SPI interface
  SPI.beginTransaction(DACSettings); // Set SPI parameters to DAC speed and bit order
  digitalWrite(LDACPin1, LOW); // Ensure DAC load pin is at default level (low)
  #if NUM_CHANNELS == 8
    digitalWrite(LDACPin2, LOW);
  #endif
  ProgramDAC(24, 0, 0); // NOP
  zeroDAC(); // Set all DAC channels to 0V
  ProgramDAC(16, 0, 31); // Power up all channels + internal ref)
  ProgramDAC(12, 0, 4); // Set output range to +/- 10V
  zeroDAC(); // Set all DAC channels to 0V (again)
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
  LoadDefaultParameters();
  maxCommandByte = pow(2,NUM_CHANNELS); // Except for module code 255, command bytes above this are ignored
  SystemTime = 0;
  LastLoopTime = SystemTime;
  timerPeriod.floatVal = CycleDuration; // Set a default sampling rate (10kHz)
  hardwareTimer.begin(handler, timerPeriod.floatVal); // hardwareTimer is an interval timer object - Teensy 3.6's hardware timer
}

void loop() {
  if (usbLoadFlag) {
    loadParams();
    usbLoadFlag = false;
  }
}

void handler(void) {                     
  if (StimulatingState == 0) {
      if (LastStimulatingState == 1) { // The cycle on which all pulse trains have finished
        dacWrite(); // Update DAC to final voltages (should be resting voltage)
        DACFlag = 0;
      }
   } else {
//     if (StimulatingState == 2) {
//                // Place to include code that executes on the first cycle of a pulse train
//     }
       StimulatingState = 1;
       if (DACFlag == 1) { // A DAC update was requested
         dacWrite(); // Update DAC
         DACFlag = 0;
       }
       SystemTime++; // Increment system time (# of hardware timer cycles since stim start)
    }
    for (int i = 0; i<NUM_CHANNELS; i++) {
      if(SoftTriggerScheduled[i]) { // Soft triggers are "scheduled" to be handled on the next cycle, since the serial read took too much time.
        SoftTriggered[i] = 1;
        SoftTriggerScheduled[i] = 0;
      }
    }
    LastStimulatingState = StimulatingState;
  if (Serial1COM.available()) {
     CommandByte = Serial1COM.readByte(); // Read a byte
     switch(CommandByte) {
        case 255: // Code to return module info
          returnModuleInfo();
        break;
        default: // Soft-trigger specific output channels. Which channels are indicated as bits of a single byte read.
          if (CommandByte < maxCommandByte) {
            for (int i = 0; i < NUM_CHANNELS; i++) {
              if (((StimulusStatus[i] == 1) || (PreStimulusStatus[i] == 1))) {
                if ((TriggerMode[0] == 1) && bitRead(CommandByte, i)) {
                  killChannel(i);
                }
              } else {
                SoftTriggerScheduled[i] = bitRead(CommandByte, i); 
              }
            }
          }
        break;
     }
  }
  if (USBCOM.available() && !usbLoadFlag) { // If bytes are available in the serial port buffer
    CommandByte = USBCOM.readByte(); // Read a byte
    if (CommandByte == OpMenuByte) { // The first byte must be 213. Now, read the actual command byte. (Reduces interference from port scanning applications)
      CommandByte = USBCOM.readByte(); // Read the command byte (an op code for the operation to execute)
      switch (CommandByte) {
        case 72: { // Handshake
          USBCOM.writeByte(75); // Send 'K' (as in ok)
          USBCOM.writeUint32(FIRMWARE_VERSION); // Send the firmware version as a 4 byte unsigned integer
          ConnectedToApp = 1;
        } break;
        case 73: { // Program the module - total program (can be faster than item-wise, if many parameters have changed)
          if (HARDWARE_VERSION == 1) {
            loadParams();
          } else {
            usbLoadFlag = true;
          }
        } break;
        
        case 74: { // Program one parameter, one channel
          inByte2 = USBCOM.readByte();
          inByte3 = USBCOM.readByte(); // inByte3 = target channels (1-4)
          inByte3 = inByte3 - 1; // Convert channel for zero-indexing
          switch (inByte2) { 
             case 1: {IsBiphasic[inByte3] = USBCOM.readByte();} break;
             case 2: {Phase1Voltage[inByte3] = USBCOM.readUint16();} break;
             case 3: {Phase2Voltage[inByte3] = USBCOM.readUint16();} break;
             case 4: {Phase1Duration[inByte3] = USBCOM.readUint32();} break;
             case 5: {InterPhaseInterval[inByte3] = USBCOM.readUint32();} break;
             case 6: {Phase2Duration[inByte3] = USBCOM.readUint32();} break;
             case 7: {InterPulseInterval[inByte3] = USBCOM.readUint32();} break;
             case 8: {BurstDuration[inByte3] = USBCOM.readUint32();} break;
             case 9: {BurstInterval[inByte3] = USBCOM.readUint32();} break;
             case 10: {PulseTrainDuration[inByte3] = USBCOM.readUint32();} break;
             case 11: {PulseTrainDelay[inByte3] = USBCOM.readUint32();} break;
             case 14: {CustomTrainID[inByte3] = USBCOM.readByte();} break;
             case 15: {CustomTrainTarget[inByte3] = USBCOM.readByte();} break;
             case 16: {CustomTrainLoop[inByte3] = USBCOM.readByte();} break;
             case 17: {RestingVoltage[inByte3] = USBCOM.readUint16();} break;
             case 128: {TriggerMode[inByte3] = USBCOM.readByte();} break;
          }
          if (inByte2 < 14) {
            if ((BurstDuration[inByte3] == 0) || (BurstInterval[inByte3] == 0)) {UsesBursts[inByte3] = false;} else {UsesBursts[inByte3] = true;}
            if (CustomTrainTarget[inByte3] == 1) {UsesBursts[inByte3] = true;}
            if ((CustomTrainID[inByte3] > 0) && (CustomTrainTarget[inByte3] == 0)) {UsesBursts[inByte3] = false;}
          }
          if (inByte2 == 17) {
            dacValue.uint16[inByte3] = RestingVoltage[inByte3];
            dacWrite();
          }
          PulseDuration[inByte3] = ComputePulseDuration(IsBiphasic[inByte3], Phase1Duration[inByte3], InterPhaseInterval[inByte3], Phase2Duration[inByte3]);
          if ((CustomTrainID[inByte3] > 0) && (CustomTrainTarget[inByte3] == 1)) {
            IsCustomBurstTrain[inByte3] = 1;
          } else {
            IsCustomBurstTrain[inByte3] = 0;
          }
          USBCOM.writeByte(1); // Send confirm byte
        } break;
  
        case 75: { // Program custom pulse train
          inByte2 = USBCOM.readByte(); // train Index
          CustomTrainNpulses[inByte2] = USBCOM.readUint32();
          for (int x = 0; x < CustomTrainNpulses[inByte2]; x++) {
            CustomPulseTimes[inByte2][x] = USBCOM.readUint32();
          }
          for (int x = 0; x < CustomTrainNpulses[inByte2]; x++) {
            CustomVoltages[inByte2][x] = USBCOM.readUint16();
          }
          USBCOM.writeByte(1); // Send confirm byte
        } break;     
        
        case 77: { // Soft-trigger specific output channels. Which channels are indicated as bits of a single byte read.
          inByte2 = USBCOM.readByte();
          for (int i = 0; i < NUM_CHANNELS; i++) {
            if (((StimulusStatus[i] == 1) || (PreStimulusStatus[i] == 1))) {
              if ((TriggerMode[0] == 1) && bitRead(inByte2, i)) {
                killChannel(i);
              }
            } else {
              SoftTriggerScheduled[i] = bitRead(inByte2, i); 
            }
          }
        } break;
        case 79: { // Write specific voltage to an output channel (not a pulse train) 
          byte myChannel = USBCOM.readByte();
          myChannel = myChannel - 1; // Convert for zero-indexing
          uint16_t val = USBCOM.readUint16();
          dacValue.uint16[myChannel] = val;
          dacWrite();
          USBCOM.writeByte(1); // Send confirm byte
        } break;
        case 80: { // Soft-abort ongoing stimulation without disconnecting from client
         for (int i = 0; i < NUM_CHANNELS; i++) {
          killChannel(i);
        }
        dacWrite();
       } break;
       case 81: { // Disconnect from client
          ConnectedToApp = 0;
          for (int i = 0; i < NUM_CHANNELS; i++) {
            killChannel(i);
          }
          dacWrite();
         } break;
        case 82:{ // Set Continuous Loop mode (play the current parametric pulse train indefinitely)
          inByte2 = USBCOM.readByte(); // State (0 = off, 1 = on)
          inByte3 = USBCOM.readByte(); // Channel
          ContinuousLoopMode[inByte3] = inByte2;
          if (inByte2) {
            SoftTriggerScheduled[inByte3] = 1;
          } else {
            killChannel(inByte3);
          }
          dacWrite();
          USBCOM.writeByte(1);
        } break;
        case 86: { // Override Arduino IO Lines (for development and debugging only - may disrupt normal function)
          inByte2 = USBCOM.readByte();
          inByte3 = USBCOM.readByte();
          pinMode(inByte2, OUTPUT); digitalWrite(inByte2, inByte3);
        } break; 
        
        case 87: { // Direct Read IO Lines (for development and debugging only - may disrupt normal function)
          inByte2 = USBCOM.readByte();
          pinMode(inByte2, INPUT);
          delayMicroseconds(10);
          LogicLevel = digitalRead(inByte2);
          USBCOM.writeByte(LogicLevel);
        } break; 
        case 91: { // Program a parameter on all 4 channels
          inByte2 = USBCOM.readByte();
          switch (inByte2) { 
             case 1: {USBCOM.readByteArray(IsBiphasic, NUM_CHANNELS);} break;
             case 2: {USBCOM.readUint16Array(Phase1Voltage, NUM_CHANNELS);} break;
             case 3: {USBCOM.readUint16Array(Phase2Voltage, NUM_CHANNELS);} break;
             case 4: {USBCOM.readUint32Array(Phase1Duration,NUM_CHANNELS);} break;
             case 5: {USBCOM.readUint32Array(InterPhaseInterval,NUM_CHANNELS);} break;
             case 6: {USBCOM.readUint32Array(Phase2Duration, NUM_CHANNELS);} break;
             case 7: {USBCOM.readUint32Array(InterPulseInterval, NUM_CHANNELS);} break;
             case 8: {USBCOM.readUint32Array(BurstDuration, NUM_CHANNELS);} break;
             case 9: {USBCOM.readUint32Array(BurstInterval, NUM_CHANNELS);} break;
             case 10: {USBCOM.readUint32Array(PulseTrainDuration, NUM_CHANNELS);} break;
             case 11: {USBCOM.readUint32Array(PulseTrainDelay, NUM_CHANNELS);} break;
             case 14: {USBCOM.readByteArray(CustomTrainID, NUM_CHANNELS);} break;
             case 15: {USBCOM.readByteArray(CustomTrainTarget, NUM_CHANNELS);} break;
             case 16: {USBCOM.readByteArray(CustomTrainLoop, NUM_CHANNELS);} break;
             case 17: {USBCOM.readUint16Array(RestingVoltage, NUM_CHANNELS);} break;
             case 128: {USBCOM.readByteArray(TriggerMode, 2);} break;
          }
          for (int iChan = 0; iChan < NUM_CHANNELS; iChan++) {
            if (inByte2 < 14) {
              if ((BurstDuration[iChan] == 0) || (BurstInterval[iChan] == 0)) {UsesBursts[iChan] = false;} else {UsesBursts[iChan] = true;}
              if (CustomTrainTarget[iChan] == 1) {UsesBursts[iChan] = true;}
              if ((CustomTrainID[iChan] > 0) && (CustomTrainTarget[iChan] == 0)) {UsesBursts[iChan] = false;}
            }
            if (inByte2 == 17) {
              dacValue.uint16[iChan] = RestingVoltage[iChan];
              dacWrite();
            }
            if (inByte2 == 18) {
              if (ContinuousLoopMode[iChan]) {
                SoftTriggerScheduled[iChan] = 1;
              } else {
                killChannel(iChan);
              }
            }
            PulseDuration[iChan] = ComputePulseDuration(IsBiphasic[iChan], Phase1Duration[iChan], InterPhaseInterval[iChan], Phase2Duration[iChan]);
            if ((CustomTrainID[iChan] > 0) && (CustomTrainTarget[iChan] == 1)) {
              IsCustomBurstTrain[iChan] = 1;
            } else {
              IsCustomBurstTrain[iChan] = 0;
            }
          }
          USBCOM.writeByte(1); // Send confirm byte
          dacWrite();
        } break;
        case 92: {
          sendCurrentParams();
        } break;
        case 95: {
          USBCOM.writeByte(NUM_CHANNELS);
        } break;
      }
     }
    } // End if USBCOM.available()
       
    for (int x = 0; x < NUM_CHANNELS; x++) {
      byte KillChannel = 0;
       // If trigger channels are in toggle mode and a trigger arrived, or in gated mode and line is low, shut down any governed channels that are playing a pulse train
       if (((StimulusStatus[x] == 1) || (PreStimulusStatus[x] == 1))) {
        // Cancel playback if trigger arrived in toggle mode
        
      } else {
       // Adjust StimulusStatus to reflect any new trigger events
       if (SoftTriggered[x]) {
         if (StimulatingState == 0) {SystemTime = 0; StimulatingState = 2;}
         PreStimulusStatus[x] = 1; BurstStatus[x] = 1; PrePulseTrainTimestamps[x] = SystemTime; PulseStatus[x] = 0; 
         SoftTriggered[x] = 0;
       }
      }
    }
    if (StimulatingState != 2) {
     StimulatingState = 0; // null condition, will be overridden in loop if any channels are still stimulating.
    }
    // Check clock and adjust line levels for new time as per programming
    for (int x = 0; x < NUM_CHANNELS; x++) {
      byte thisTrainID = CustomTrainID[x];
      byte thisTrainIDIndex = thisTrainID-1;
      if (PreStimulusStatus[x] == 1) {
          if (StimulatingState != 2) {
           StimulatingState = 1;
          }
        if (SystemTime == (PrePulseTrainTimestamps[x] + PulseTrainDelay[x])) {
          PreStimulusStatus[x] = 0;
          StimulusStatus[x] = 1;
          PulseStatus[x] = 0;
          PulseTrainTimestamps[x] = SystemTime;
          PulseTrainEndTime[x] = SystemTime + PulseTrainDuration[x];
          if (CustomTrainTarget[x] > 0)  {
              NextBurstTransitionTime[x] = SystemTime + CustomPulseTimes[thisTrainIDIndex][0];
              BurstStatus[x] = 0;
          } else {
            NextBurstTransitionTime[x] = SystemTime+BurstDuration[x];
          }
          if (CustomTrainID[x] == 0) {
            NextPulseTransitionTime[x] = SystemTime;
            dacValue.uint16[x] = Phase1Voltage[x]; DACFlag = 1;
          } else {
            NextPulseTransitionTime[x] = SystemTime + CustomPulseTimes[thisTrainIDIndex][0]; 
            CustomPulseTimeIndex[x] = 0;
          }
        }
      }
      if (StimulusStatus[x] == 1) { // if this output line has been triggered and is delivering a pulse train
          if (StimulatingState != 2) {
           StimulatingState = 1; 
          }
        if (BurstStatus[x] == 1) { // if this output line is currently gated "on"
          switch (PulseStatus[x]) { // depending on the phase of the pulse
           case 0: { // if this is the inter-pulse interval
            // determine if the next pulse should start now
            if ((CustomTrainID[x] == 0) || ((CustomTrainID[x] > 0) && (CustomTrainTarget[x] == 1))) {
              if (SystemTime == NextPulseTransitionTime[x]) {
                NextPulseTransitionTime[x] = SystemTime + Phase1Duration[x];
                    if (!((UsesBursts[x] == 1) && (NextPulseTransitionTime[x] >= NextBurstTransitionTime[x]))){ // so that it doesn't start a pulse it can't finish due to burst end
                      PulseStatus[x] = 1;
                      if ((CustomTrainID[x] > 0) && (CustomTrainTarget[x] == 1)) {
                        dacValue.uint16[x] = CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]; DACFlag = 1;
                      } else {
                        dacValue.uint16[x] = Phase1Voltage[x]; DACFlag = 1;
                      }
                    }
                 }
              } else {
               if (SystemTime == NextPulseTransitionTime[x]) {
                     int SkipNextInterval = 0;
                     if ((CustomTrainLoop[x] == 1) && (CustomPulseTimeIndex[x] == CustomTrainNpulses[thisTrainIDIndex])) {
                            CustomPulseTimeIndex[x] = 0;
                            PulseTrainTimestamps[x] = SystemTime;
                     }
                     if (CustomPulseTimeIndex[x] < CustomTrainNpulses[thisTrainIDIndex]) {
                       if ((CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]+1] - CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]]) > Phase1Duration[x]) {
                         NextPulseTransitionTime[x] = SystemTime + Phase1Duration[x];
                       } else {
                         NextPulseTransitionTime[x] = PulseTrainTimestamps[x] + CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]+1];  
                         SkipNextInterval = 1;
                       }
                     }
                     if (SkipNextInterval == 0) {
                        PulseStatus[x] = 1;
                     }
                     dacValue.uint16[x] = CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]; DACFlag = 1;
                     if (IsBiphasic[x] == 0) {
                        CustomPulseTimeIndex[x] = CustomPulseTimeIndex[x] + 1;
                     }
                     if (CustomPulseTimeIndex[x] > (CustomTrainNpulses[thisTrainIDIndex])){
                       CustomPulseTimeIndex[x] = 0;
                       if (CustomTrainLoop[x] == 0) {
                         killChannel(x);
                       }
                     }
                  }
              } 
            } break;
            
            case 1: { // if this is the first phase of the pulse
             // determine if this phase should end now
             if (SystemTime == NextPulseTransitionTime[x]) {
                if (IsBiphasic[x] == 0) {
                  if (CustomTrainID[x] == 0) {
                      NextPulseTransitionTime[x] = SystemTime + InterPulseInterval[x];
                      PulseStatus[x] = 0;
                      dacValue.uint16[x] = RestingVoltage[x]; DACFlag = 1;
                  } else {
                    if (CustomTrainTarget[x] == 0) {
                      NextPulseTransitionTime[x] = PulseTrainTimestamps[x] + CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]];
                    } else {
                      NextPulseTransitionTime[x] = SystemTime + InterPulseInterval[x];
                    }
                    if (CustomPulseTimeIndex[x] == CustomTrainNpulses[thisTrainIDIndex]) {
                      if (CustomTrainLoop[x] == 1) {
                          CustomPulseTimeIndex[x] = 0;
                          PulseTrainTimestamps[x] = SystemTime;
                          dacValue.uint16[x] = CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]; DACFlag = 1;
                          if ((CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]+1] - CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]]) > Phase1Duration[x]) {
                            PulseStatus[x] = 1;
                          } else {
                            PulseStatus[x] = 0;
                          }
                          NextPulseTransitionTime[x] = PulseTrainTimestamps[x] + Phase1Duration[x];
                          CustomPulseTimeIndex[x] = CustomPulseTimeIndex[x] + 1;
                      } else {
                        killChannel(x);
                      }
                    } else {
                      PulseStatus[x] = 0;
                      dacValue.uint16[x] = RestingVoltage[x]; DACFlag = 1;
                    }
                  }
     
                } else {
                  if (InterPhaseInterval[x] == 0) {
                    NextPulseTransitionTime[x] = SystemTime + Phase2Duration[x];
                    PulseStatus[x] = 3;
                    if (CustomTrainID[x] == 0) {
                      dacValue.uint16[x] = Phase2Voltage[x]; DACFlag = 1;
                    } else {
                     if (CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]] < 32768) {
                       dacValue.uint16[x] = 32768 + (32768 - CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]); DACFlag = 1;
                     } else {
                       dacValue.uint16[x] = 32768 - (CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]] - 32768); DACFlag = 1;
                     }
                     if (CustomTrainTarget[x] == 0) {
                         CustomPulseTimeIndex[x] = CustomPulseTimeIndex[x] + 1;
                     }
                    } 
                  } else {
                    NextPulseTransitionTime[x] = SystemTime + InterPhaseInterval[x];
                    PulseStatus[x] = 2;
                    dacValue.uint16[x] = RestingVoltage[x]; DACFlag = 1;
                  }
                }
              }
            } break;
            case 2: {
               if (SystemTime == NextPulseTransitionTime[x]) {
                 NextPulseTransitionTime[x] = SystemTime + Phase2Duration[x];
                 PulseStatus[x] = 3;
                 if (CustomTrainID[x] == 0) {
                   dacValue.uint16[x] = Phase2Voltage[x]; DACFlag = 1;
                 } else {
                   if (CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]] < 32768) {
                       dacValue.uint16[x] = 32768 + (32768 - CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]); DACFlag = 1;
                     } else {
                       dacValue.uint16[x] = 32768 - (CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]] - 32768); DACFlag = 1;
                   }
                   if (CustomTrainTarget[x] == 0) {
                       CustomPulseTimeIndex[x] = CustomPulseTimeIndex[x] + 1;
                   }
                 }
               }
            } break;
            case 3: {
              if (SystemTime == NextPulseTransitionTime[x]) {
                  if (CustomTrainID[x] == 0) {
                      NextPulseTransitionTime[x] = SystemTime + InterPulseInterval[x];
                  } else {
                    if (CustomTrainTarget[x] == 0) {
                      NextPulseTransitionTime[x] = PulseTrainTimestamps[x] + CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]];
                      if (CustomPulseTimeIndex[x] == (CustomTrainNpulses[thisTrainIDIndex])){
                          killChannel(x);
                     }
                    } else {
                      NextPulseTransitionTime[x] = SystemTime + InterPulseInterval[x];
                    } 
                  }   
                 if (!((CustomTrainID[x] == 0) && (InterPulseInterval[x] == 0))) { 
                   PulseStatus[x] = 0;
                   dacValue.uint16[x] = RestingVoltage[x]; DACFlag = 1;
                 } else {
                   PulseStatus[x] = 1;
                   NextPulseTransitionTime[x] = (NextPulseTransitionTime[x] - InterPulseInterval[x]) + (Phase1Duration[x]);
                   dacValue.uint16[x] = Phase1Voltage[x]; DACFlag = 1;
                 }
               }
            } break;
            
          }
        }
          // Determine if burst status should go to 0 now
       if (UsesBursts[x] == true) {
        if (SystemTime == NextBurstTransitionTime[x]) {
          if (BurstStatus[x] == 1) {
            if (CustomTrainID[x] == 0) {
                     NextPulseTransitionTime[x] = SystemTime + BurstInterval[x];
                     NextBurstTransitionTime[x] = SystemTime + BurstInterval[x];              
            } else if (CustomTrainTarget[x] == 1) {
              CustomPulseTimeIndex[x] = CustomPulseTimeIndex[x] + 1;
              if (CustomTrainID[x] > 0) {
                 if (CustomPulseTimeIndex[x] == (CustomTrainNpulses[thisTrainIDIndex])){
                     killChannel(x);
                 }
                 NextPulseTransitionTime[x] = PulseTrainTimestamps[x] + CustomPulseTimes[thisTrainIDIndex][CustomPulseTimeIndex[x]];
                 NextBurstTransitionTime[x] = NextPulseTransitionTime[x];
              }
            }
              BurstStatus[x] = 0;
              dacValue.uint16[x] = RestingVoltage[x]; DACFlag = 1;
          } else {
          // Determine if burst status should go to 1 now
            NextBurstTransitionTime[x] = SystemTime + BurstDuration[x];
            NextPulseTransitionTime[x] = SystemTime + Phase1Duration[x];
            PulseStatus[x] = 1;
            if ((CustomTrainID[x] > 0) && (CustomTrainTarget[x] == 1)) {
              if (CustomPulseTimeIndex[x] < CustomTrainNpulses[thisTrainIDIndex]){
                 dacValue.uint16[x] = CustomVoltages[thisTrainIDIndex][CustomPulseTimeIndex[x]]; DACFlag = 1;
              }       
            } else {
                 dacValue.uint16[x] = Phase1Voltage[x]; DACFlag = 1;
            }
            BurstStatus[x] = 1;
         }
        }
       } 
        // Determine if Stimulus Status should go to 0 now
        if ((SystemTime == PulseTrainEndTime[x]) && (StimulusStatus[x] == 1)) {
          if (((CustomTrainID[x] > 0) && (CustomTrainLoop[x] == 1)) || (CustomTrainID[x] == 0)) {
            if (ContinuousLoopMode[x] == false) {
                killChannel(x);
            }
          }
        }
     }
   }
}
// End main loop

byte* Long2Bytes(long LongInt2Break) {
  byte Output[4] = {0};
  return Output;
}


void killChannel(byte outputChannel) {
  CustomPulseTimeIndex[outputChannel] = 0;
  PreStimulusStatus[outputChannel] = 0;
  StimulusStatus[outputChannel] = 0;
  PulseStatus[outputChannel] = 0;
  BurstStatus[outputChannel] = 0;
  dacValue.uint16[outputChannel] = RestingVoltage[outputChannel]; DACFlag = 1;
}


void dacWrite() {
  digitalWrite(LDACPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  dacBuffer[0] = 3;
  dacBuffer[1] = dacValue.byteArray[1];
  dacBuffer[2] = dacValue.byteArray[0];
  SPI.transfer(dacBuffer,3);
  digitalWrite(SyncPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  dacBuffer[0] = 2;
  dacBuffer[1] = dacValue.byteArray[3];
  dacBuffer[2] = dacValue.byteArray[2];
  SPI.transfer(dacBuffer,3);
  digitalWrite(SyncPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  dacBuffer[0] = 0;
  dacBuffer[1] = dacValue.byteArray[5];
  dacBuffer[2] = dacValue.byteArray[4];
  SPI.transfer(dacBuffer,3);
  digitalWrite(SyncPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  dacBuffer[0] = 1;
  dacBuffer[1] = dacValue.byteArray[7];
  dacBuffer[2] = dacValue.byteArray[6];
  SPI.transfer(dacBuffer,3);
  digitalWrite(SyncPin1,HIGH); 
  #if NUM_CHANNELS == 8
    digitalWrite(LDACPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    dacBuffer[0] = 3;
    dacBuffer[1] = dacValue.byteArray[9];
    dacBuffer[2] = dacValue.byteArray[8];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    dacBuffer[0] = 2;
    dacBuffer[1] = dacValue.byteArray[11];
    dacBuffer[2] = dacValue.byteArray[10];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    dacBuffer[0] = 0;
    dacBuffer[1] = dacValue.byteArray[13];
    dacBuffer[2] = dacValue.byteArray[12];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    dacBuffer[0] = 1;
    dacBuffer[1] = dacValue.byteArray[15];
    dacBuffer[2] = dacValue.byteArray[14];
    SPI.transfer(dacBuffer,3);
    digitalWrite(SyncPin2,HIGH); 
    digitalWrite(LDACPin2,HIGH);
    digitalWrite(LDACPin2,LOW);
  #else
    digitalWrite(LDACPin1,HIGH); // In 4ch model, this line adds a required delay, added already in 8ch above by  digitalWrite(LDACPin2,HIGH);
  #endif
  digitalWrite(LDACPin1,LOW);
}

void ProgramDAC(byte Data1, byte Data2, byte Data3) {
  digitalWrite(LDACPin1,HIGH);
  digitalWrite(SyncPin1,LOW);
  SPI.transfer (Data1);
  SPI.transfer (Data2);
  SPI.transfer (Data3);
  digitalWrite(SyncPin1,HIGH);
  digitalWrite(LDACPin1,LOW);
  #if NUM_CHANNELS == 8
    digitalWrite(LDACPin2,HIGH);
    digitalWrite(SyncPin2,LOW);
    SPI.transfer (Data1);
    SPI.transfer (Data2);
    SPI.transfer (Data3);
    digitalWrite(SyncPin2,HIGH);
    digitalWrite(LDACPin2,LOW);
  #endif
}

void LoadDefaultParameters() {
  // This function is called on boot if the EEPROM has an invalid program (or no program).
  for (int x = 0; x < NUM_CHANNELS; x++) {
      Phase1Duration[x] = 10;
      InterPhaseInterval[x] = 10;
      Phase2Duration[x] = 10;
      InterPulseInterval[x] = 90;
      BurstDuration[x] = 0;
      BurstInterval[x] = 0;
      PulseTrainDuration[x] = 10000;
      PulseTrainDelay[x] = 0;
      IsBiphasic[x] = 0;
      Phase1Voltage[x] = 49152;
      Phase2Voltage[x] = 16384;
      RestingVoltage[x] = 32768;
      CustomTrainID[x] = 0;
      CustomTrainTarget[x] = 0;
      CustomTrainLoop[x] = 0;
      UsesBursts[x] = 0;
    }
   TriggerMode[0] = 0; 
}

void AbortAllPulseTrains() {
    for (int x = 0; x < NUM_CHANNELS; x++) {
      killChannel(x);
    }
    dacWrite();
}

unsigned long ComputePulseDuration(byte myBiphasic, unsigned long myPhase1, unsigned long myPhaseInterval, unsigned long myPhase2) {
    unsigned long Duration = 0;
    if (myBiphasic == 0) {
       Duration = myPhase1;
     } else {
       Duration = myPhase1 + myPhaseInterval + myPhase2;
     }
     return Duration;
}

void sendCurrentParams() {
    USBCOM.writeUint32Array(Phase1Duration, NUM_CHANNELS);
    USBCOM.writeUint32Array(InterPhaseInterval, NUM_CHANNELS);
    USBCOM.writeUint32Array(Phase2Duration, NUM_CHANNELS);
    USBCOM.writeUint32Array(InterPulseInterval, NUM_CHANNELS);
    USBCOM.writeUint32Array(BurstDuration, NUM_CHANNELS);
    USBCOM.writeUint32Array(BurstInterval, NUM_CHANNELS);
    USBCOM.writeUint32Array(PulseTrainDuration, NUM_CHANNELS);
    USBCOM.writeUint32Array(PulseTrainDelay, NUM_CHANNELS);
    USBCOM.writeUint16Array(Phase1Voltage, NUM_CHANNELS);
    USBCOM.writeUint16Array(Phase2Voltage, NUM_CHANNELS);
    USBCOM.writeUint16Array(RestingVoltage, NUM_CHANNELS);
    USBCOM.writeByteArray(IsBiphasic, NUM_CHANNELS);
    USBCOM.writeByteArray(CustomTrainID, NUM_CHANNELS);
    USBCOM.writeByteArray(CustomTrainTarget, NUM_CHANNELS);
    USBCOM.writeByteArray(CustomTrainLoop, NUM_CHANNELS);
    USBCOM.writeByteArray(TriggerMode, 1);
}

void zeroDAC() {
  // Set DAC to resting voltage on all channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    dacValue.uint16[i] = DACBits_ZeroVolts;
  }
  dacWrite(); // Update the DAC, to set all channels to mid-range (0V)
}

void loadParams() {
  USBCOM.readUint32Array(Phase1Duration, NUM_CHANNELS);
  USBCOM.readUint32Array(InterPhaseInterval, NUM_CHANNELS);
  USBCOM.readUint32Array(Phase2Duration, NUM_CHANNELS);
  USBCOM.readUint32Array(InterPulseInterval, NUM_CHANNELS);
  USBCOM.readUint32Array(BurstDuration, NUM_CHANNELS);
  USBCOM.readUint32Array(BurstInterval, NUM_CHANNELS);
  USBCOM.readUint32Array(PulseTrainDuration, NUM_CHANNELS);
  USBCOM.readUint32Array(PulseTrainDelay, NUM_CHANNELS);
  USBCOM.readUint16Array(Phase1Voltage, NUM_CHANNELS);
  USBCOM.readUint16Array(Phase2Voltage, NUM_CHANNELS);
  USBCOM.readUint16Array(RestingVoltage, NUM_CHANNELS);
  USBCOM.readByteArray(IsBiphasic, NUM_CHANNELS);
  USBCOM.readByteArray(CustomTrainID, NUM_CHANNELS);
  USBCOM.readByteArray(CustomTrainTarget, NUM_CHANNELS);
  USBCOM.readByteArray(CustomTrainLoop, NUM_CHANNELS);
 USBCOM.readByteArray(TriggerMode, 1);
 USBCOM.writeByte(1); // Send confirm byte
 for (int x = 0; x < NUM_CHANNELS; x++) {
   if ((BurstDuration[x] == 0) || (BurstInterval[x] == 0)) {UsesBursts[x] = false;} else {UsesBursts[x] = true;}
   if (CustomTrainTarget[x] == 1) {UsesBursts[x] = true;}
   if ((CustomTrainID[x] > 0) && (CustomTrainTarget[x] == 0)) {UsesBursts[x] = false;}
   PulseDuration[x] = ComputePulseDuration(IsBiphasic[x], Phase1Duration[x], InterPhaseInterval[x], Phase2Duration[x]);
   if ((CustomTrainID[x] > 0) && (CustomTrainTarget[x] == 1)) {
    IsCustomBurstTrain[x] = 1;
   } else {
    IsCustomBurstTrain[x] = 0;
   }
   dacValue.uint16[x] = RestingVoltage[x];
 }
 dacWrite();
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
