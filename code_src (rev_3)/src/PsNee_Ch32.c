#include "ch32fun.h"
#include <stdio.h>

#define SYSCLK_FREQ_48MHZ_HSI 48000000
// PsNee
// Porting CH32V003 by Carmax91 rev 3.1 heavly based on kalymos PsNee_v9 (https://github.com/kalymos/PsNee)
//
// Beware to use the PSX 3.5V / 3.3V power, *NOT* 5V! The installation pictures include an example.
//
// Only for WCH CH32V003 mcu @ 48Mhz internal clock.
//
// Coded using Ch32fun library! https://github.com/cnlohr/ch32fun
//
// PAL PM-41 consoles BIOS patching is NOT supported.
//
// Changelog:
// Rev1.0: First rev
// Rev2.0: Newer timing implementation made by the great @kalymos, code now is even smaller and no more ISR timing dipendent!
// Rev3.0: Newer board detection function with added 300ms stabilization delay to filter PU-7/20 power-up noise and sync with PU-22+ oscillating signals.
//         On the Injection function updated WFCK modulation for 7.3/14.6 kHz compatibility.
// Rev3.1: Refactor board detection function.
//         Refactor WFCK modulation.
//
// PINOUT for wch ch32v003j4m6:
/*
                   ___ ___
       LED (PD6) -|   U   |- PD1 (SWIO)
             GND -|       |- SUBQ (PC4)
 WFCK_GATE (PA2) -|       |- SQCK (PC2)
             VCC -|       |- DATA (PC1)
                   -------

*/
// DON'T forget to set the region of modchip via #define X_BIT ('e' for PAL, 'a' for USA, 'i' for JAP region) predirectives.
//------REGION SELECT---------
#define X_Bit 'e' // PAL
// #define X_Bit 'a' // USA
// #define X_Bit 'i' // JAP
//----------------------------

/* Global define */
//The definitions below should be never touched!
#define bits_delay 4000     // 250 bits/s (microseconds)
#define injections_delay 90 // 72 in oldcrow. PU-22+ work best with 80 to 100 (milliseconds)
#define HYSTERESIS_MAX 17  // The sweet spot is between 11~19. All models have bad behavior below 11, 
                           // PU-41 can start to have bad behavior beyond 20, for fat models we can go up to 60
                           // On fat models if your reader is really bad you can increase this value in steps of 5

/* Global Variable */
uint8_t wfck_mode = 0; //Flag initializing for automatic console generation selection 0 = old, 1 = pu-22 and above.
uint8_t hysteresis = 0;
uint8_t scbuf[12] = {0}; // SUBQ bit storage
uint16_t timeout_clock_counter = 0;
uint8_t bitbuf = 0;
uint8_t bitpos = 0;
uint8_t scpos = 0; // scbuf position


// *****************************************************************************
// Function: board_detection
// DESCRIPTION: 
// Distinguishes motherboard generations (PU-7 through PU-22+) by analyzing 
// the behavior of the WFCK signal.
//
// SIGNAL CHARACTERISTICS:
// - Legacy Boards (PU-7 to PU-20): WFCK acts as a static GATE signal. 
//   It remains HIGH (continuous) during the region-check window.
// - Modern Boards (PU-22 or newer): WFCK is an oscillating clock signal 
//   (Frequency-based).
//  
// WFCK: __-----------------------  // CONTINUOUS (PU-7 .. PU-20)(GATE)
// 
// WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // FREQUENCY  (PU-22 or newer)
// 
// HISTORICAL CONTEXT:
// Traditionally, WFCK was referred to as the "GATE" signal. On early models, 
// modchips functioned as a synchronized gate, pulling the signal LOW 
// precisely when the region-lock data was being processed.
//  
// FREQUENCY DATA:
// - Initial/Protection Phase: ~7.3 kHz.
// - Standard Data Reading: ~14.6 kHz.
//
// *****************************************************************************

void board_detection()
{
 wfck_mode = 0;           // Default: Legacy (GATE)
 uint8_t pulse_hits = 25; // We need to see 25 oscillations to confirm FREQUENCY mode
 uint16_t detectionWindow = 10000; 
 Delay_Ms(300);          // Wait for WFCK to stabilize (High on Legacy, Oscillation on Modern)

  while (--detectionWindow) {
    // LOGIC BASED ON YOUR ANALYSIS:
    // If WFCK is "CONTINUOUS" (Legacy), it stays HIGH. PIN_WFCK_READ will always be 1.
    // If WFCK is "FREQUENCY" (Modern), it will hit 0 (LOW) periodically.
    
     if (!(GPIOA->INDR >> (2 & 0xf) & 1)) { //Wfck == 0  //Detect a LOW state (only possible in FREQUENCY mode)
      
      pulse_hits--;        // Record one oscillation hit

      if (pulse_hits == 0) {
        wfck_mode = 1;     // Confirmed: FREQUENCY mode (PU-22 or newer)
        return;            // Exit as soon as we are sure
      }

      //SYNC: Wait for the signal to go HIGH again.
      //This ensures we count each pulse of the "FREQUENCY" signal only once.
      
      while (!(GPIOA->INDR >> (2 & 0xf) & 1) && detectionWindow > 0) {
        detectionWindow--;
      }
    }
  }
  // If the window expires without seeing enough LOW pulses, it remains wfck_mode = 0 (GATE)
}


// *****************************************************************************************
// Function: inject_SCEX
// DESCRIPTION:
// Injects SCEX data corresponding to a given region ('e' for Europe, 'a' for America,
// 'i' for Japan). This function is used for modulating the SCEX signal to bypass
// region-locking mechanisms.
//
// PARAMETERS:
// - region: A character ('e', 'a', or 'i') representing the target region.
//
// *****************************************************************************************

void inject_SCEX(const char region)
{

  // SCEX data patterns for different regions (SCEE: Europe, SCEA: America, SCEI: Japan)
  // Each array contains the specific bit sequence required to bypass region locking.
  static const uint8_t SCEEData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11101010,
      0b00000010};

  static const uint8_t SCEAData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11111010,
      0b00000010};

  static const uint8_t SCEIData[] = {
      0b01011001,
      0b11001001,
      0b01001011,
      0b01011101,
      0b11011010,
      0b00000010};

  // Select the appropriate data pointer based on the region character to avoid
  // repetitive conditional checks inside the high-timing-sensitive loop.
  const uint8_t *ByteSet = (region == 'e') ? SCEEData : (region == 'a') ? SCEAData
                                                                        : SCEIData;

  // Iterate through the 44 bits of the SCEX sequence
  for (uint8_t bit_counter = 0; bit_counter < 44; bit_counter++)
  {

    // Extraction of the current bit (Inlined readBit logic)
    uint8_t currentBit = (ByteSet[bit_counter / 8] & (1 << (bit_counter % 8)));

    // -------------------------------------------------------------------------
    // MODE: OLDER BOARDS (PU-7 to PU-20) - Standard Gate Logic
    // -------------------------------------------------------------------------
    if (!wfck_mode)
    {
      if (currentBit == 0)
      {
        // For OLD boards, bit 0 is a forced LOW signal
        // PC1 -> Data Output -------------------------
        GPIOC->CFGLR &= ~(0xf << (4 * 1));
        GPIOC->CFGLR |= (3 | 0) << (4 * 1);
        //---------------------------------------------
        GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
        Delay_Us(bits_delay);
      }
      else
      {
        // For OLD boards, bit 1 is High-Z (Pin set as input)
        // PC1 <- Data Input -----------------------------------------------------------------
        GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
        //------------------------------------------------------------------------------------
        Delay_Us(bits_delay);
      }
    }

    // -------------------------------------------------------------------------
    // MODE: NEWER BOARDS (PU-22 or newer) - WFCK Clock Synchronization
    // -------------------------------------------------------------------------
    else if (wfck_mode)
    {
      if (currentBit == 0)
      {
        // For NEW boards, bit 0 is also a forced LOW signal
        // PC1 -> Data Output ----------------------
        GPIOC->CFGLR &= ~(0xf << (4 * 1));
        GPIOC->CFGLR |= (3 | 0) << (4 * 1);
        //------------------------------------------
        GPIOC->BSHR = (1 << 16 + 1); // PC1 Data Low
        Delay_Us(bits_delay);        // Wait for specified delay between bits
      }
      else
      {
       // For NEW boards, bit 1 must be modulated with the WFCK clock signal
       /* 
       WFCK Modulation Loop: Syncs to 7.3kHz or 14.6kHz.
       Follows hardware edges to stay bit-perfect with the console.
       */
       // PC1 -> Data Output ----------------------
       GPIOC->CFGLR &= ~(0xf << (4 * 1));
       GPIOC->CFGLR |= (3 | 0) << (4 * 1);
       //------------------------------------------
       
       for (uint8_t count = 30; count > 0; count--){
             
        while((GPIOA->INDR >> (2 & 0xf) & 1)); //Wait for Falling Edge  // While WfckRead == 1
        GPIOC->BSHR = (1 << 16 + 1); //PC1 -> Data Low
      
        while(!(GPIOA->INDR >> (2 & 0xf) & 1)); //Wait for Rising Edge // While WfckRead == 0
        GPIOC->BSHR = (1 << 1); //PC1 -> Data High
       }
      }
    }
  }
  // After injecting SCEX data, set DATA pin as output and clear (low)
  // PC1 -> Data Output ----------------------
  GPIOC->CFGLR &= ~(0xf << (4 * 1));
  GPIOC->CFGLR |= (3 | 0) << (4 * 1);
  //------------------------------------------
  GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
  Delay_Ms(injections_delay);
}

int main()
{
  SystemInit();
  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

  // PC1 <- Data Input -----------------------------------------------------------------
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
  //------------------------------------------------------------------------------------
  // PA2 <- Wfck Input -----------------------------------------------------------------
  GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
  //------------------------------------------------------------------------------------
  // PC4 <- Subq Input -----------------------------------------------------------------
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (4 & 0xf))))) | (4 << (4 * (4 & 0xf)));
  //------------------------------------------------------------------------------------
  // PC2 <- Sqck Input -----------------------------------------------------------------
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
  //------------------------------------------------------------------------------------
  // PD6 -> LED Output ---------------
  GPIOD->CFGLR &= ~(0xF << (4 * 6));
  GPIOD->CFGLR |= (3 | 0) << (4 * 6);
  //----------------------------------


  GPIOD->BSHR = (1 << 6); // PD6 -> LED High  //Setup begin!:

  while (!(GPIOC->INDR >> (2 & 0xf) & 1))
    ; // Sqck read==0
  while (!(GPIOA->INDR >> (2 & 0xf) & 1))
    ; // Wfck read==0

  board_detection();

  GPIOD->BSHR = (1 << 16 + 6); // PD6 -> LED Low  //Setup done!

  while (1)
  {
    Delay_Ms(1); /* Start with a small delay, which can be necessary in cases where the MCU loops too quickly and picks up the laster SUBQ trailing end*/

    __disable_irq(); // start critical section

    do
    {
      for (bitpos = 0; bitpos < 8; bitpos++)
      {
        while ((GPIOC->INDR >> (2 & 0xf) & 1) != 0) // Sqck read == 1  // wait for clock to go low
        {
          timeout_clock_counter++;
          // a timeout resets the 12 byte stream in case the PSX sends malformatted clock pulses, as happens on bootup
          if (timeout_clock_counter > 1000)
          {
            scpos = 0;
            timeout_clock_counter = 0;
            bitbuf = 0;
            bitpos = 0;
            continue;
          }
        }

        // Wait for clock to go high
        while ((GPIOC->INDR >> (2 & 0xf) & 1) == 0)
          ; // Sqck read == 0

        if ((GPIOC->INDR >> (4 & 0xf) & 1) == 1) // Subq == 1 // If clock pin high
        {
          bitbuf |= 1 << bitpos; // Set the bit at position bitpos in the bitbuf to 1. Using OR combined with a bit shift
        }

        timeout_clock_counter = 0; // no problem with this bit
      }

      scbuf[scpos] = bitbuf; // One byte done
      scpos++;
      bitbuf = 0;
    }

    while (scpos < 12); // Repeat for all 12 bytes

    __enable_irq(); // End critical section

    // *************************************************************************************************
    // Check if read head is in wobble area
    // We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
    // We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
    // All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
    // Hysteresis is used because older drives exhibit more variation in read head positioning.
    // While the laser lens moves to correct for the error, they can pick up a few TOC sectors.
    // **************************************************************************************************

    // This variable initialization macro is to replace (0x41) with a filter that will check that only the three most significant bits are correct. 0x001xxxxx
    uint8_t isDataSector = (((scbuf[0] & 0x40) == 0x40) && (((scbuf[0] & 0x10) == 0) && ((scbuf[0] & 0x80) == 0)));

    if (
        (isDataSector && scbuf[1] == 0x00 && scbuf[6] == 0x00) &&      // [0] = 41 means psx game disk. the other 2 checks are garbage protection
        (scbuf[2] == 0xA0 || scbuf[2] == 0xA1 || scbuf[2] == 0xA2 ||   // if [2] = A0, A1, A2 ..
         (scbuf[2] == 0x01 && (scbuf[3] >= 0x98 || scbuf[3] <= 0x02))) // .. or = 01 but then [3] is either > 98 or < 02
    )
    {
      hysteresis++;
    }

    // This CD has the wobble into CD-DA space. (started at 0x41, then went into 0x01)
    else if (hysteresis > 0 && ((scbuf[0] == 0x01 || isDataSector) && (scbuf[1] == 0x00 /*|| scbuf[1] == 0x01*/) && scbuf[6] == 0x00))
    {
      hysteresis++;
    }

    // None of the above. Initial detection was noise. Decrease the counter.
    else if (hysteresis > 0)
    {
      hysteresis--;
    }

    // hysteresis value "optimized" using very worn but working drive
    // should be fine on other MCUs and speeds, as the PSX dictates SUBQ rate
    if (hysteresis >= HYSTERESIS_MAX)
    {
      hysteresis = 11;

      //************************************************************************
      // Executes the region code patch injection sequence.
      //************************************************************************
      GPIOD->BSHR = (1 << 6); // PD6 -> LED High //Start Injection

      // PC1 -> Data Output ----------------------
      GPIOC->CFGLR &= ~(0xf << (4 * 1));
      GPIOC->CFGLR |= (3 | 0) << (4 * 1);
      //------------------------------------------
      GPIOC->BSHR = (1 << 16 + 1); // PC1 Data Low

      if (!wfck_mode) // If wfck_mode is 0 (oldmode)
      {
        // PA2 -> Wfck Output ----------------------
        GPIOA->CFGLR &= ~(0xf << (4 * 2));
        GPIOA->CFGLR |= (3 | 0) << (4 * 2);
        //------------------------------------------
        GPIOA->BSHR = (1 << 16 + 2); // PA2 Wfck Low
      }
      Delay_Ms(injections_delay); // HC-05 waits for a bit of silence (pin low) before it begins decoding.

      // inject symbols now. 2 x 3 runs seems optimal to cover all boards
      for (uint8_t scex = 0; scex < 2; scex++)
      {
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
        inject_SCEX(X_Bit); // e = SCEE, a = SCEA, i = SCEI
      }

      if (!wfck_mode)
      {
        // PA2 <- Wfck Input -----------------------------------------------------------------
        GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
        //------------------------------------------------------------------------------------
      }
      // PC1 <- Data Input -----------------------------------------------------------------
      GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
      //------------------------------------------------------------------------------------

      GPIOD->BSHR = (1 << 16 + 6); // PD6 -> LED Low //Injection done!
    }
  }
}

