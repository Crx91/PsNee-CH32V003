#include "ch32fun.h"
#include <stdio.h>

#define SYSCLK_FREQ_48MHZ_HSI 48000000
// PsNee
// Porting CH32V003 by Carmax91 rev3 (no timer) heavly based on kalymos PsNee_v7/v8 (https://github.com/kalymos/PsNee)
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
// Rev1: First rev
// Rev2: Newer timing implementation made by the great @kalymos, code now is even smaller and no more ISR timing dipendent!
// Rev3: Newer board detection function with added 300ms stabilization delay to filter PU-7/20 power-up noise and sync with PU-22+ oscillating signals.
//       On the Injection function updated WFCK modulation for 7.3/14.6 kHz compatibility
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

/* Global define */
//------REGION SELECT-------
#define X_Bit 'e' // PAL
// #define X_Bit 'a' //USA
// #define X_Bit 'i' // JAP
//----------------------------
#define bits_delay 4000     // 250 bits/s (microseconds)
#define injections_delay 90 // 72 in oldcrow. PU-22+ work best with 80 to 100 (milliseconds)
#define HYSTERESIS_MAX 17

/* Global Variable */
// Setup() detects which (of 2) injection methods this PSX board requires, then stores it in wfck_mode.
uint8_t wfck_mode = 0;

/*----------------------------------------------------------------------
  Function: board_detection

  This function distinguishes motherboard generations by detecting
  the nature of the WFCK signal:

  WFCK: __-----------------------  // CONTINUOUS (PU-7 .. PU-20)(GATE)

  WFCK: __-_-_-_-_-_-_-_-_-_-_-_-  // FREQUENCY  (PU-22 or newer)

  Traditionally, the WFCK signal was called GATE. This is because, on early models,
  modchips acted like a gate that would open to pull the signal down
   at the exact moment the region code was being passed (which is still the case today).

  During the initialization and region protection zone reading phases,
  the WFCK clock frequency is approximately 7.3 kHz.
  During normal data reading, the frequency shifts to 14.6 kHz.

-----------------------------------------------------------------------*/
void board_detection()
{
// Default to static signal (PU-7 to PU-20)
  wfck_mode = 0;

  /* 
    INITIAL STABILIZATION DELAY (300ms)
    PU-7 to PU-20: Voltage climbs slowly (up to 54ms) then stays HIGH (static).
    PU-22+: Signal only starts oscillating (~7.3kHz) after approximately 297ms.
    Waiting 300ms bypasses all power-up transients and initial noise.
  */
  Delay_Ms(300); 

  // Sampling window to detect the oscillating signal
  uint16_t detectionWindow = 10000; 
  
  while (--detectionWindow) {
    /* 
       On older boards (PU-7/20), the signal is now a solid HIGH.
       If we detect a LOW state, it's a potential oscillation from a newer board.
    */
    if (!(GPIOA->INDR >> (2 & 0xf) & 1)) { //Wfck == 0
      // Small debounce delay to filter out micro-glitches or remaining noise
      uint8_t debounce = 200;//100;
      while (--debounce);

      /* 
         VERIFICATION: If the signal is STILL low, it confirms a real 
         clock cycle (WFCK). Older boards will never reach this state 
         once stabilized at HIGH.
      */
      if (!(GPIOA->INDR >> (2 & 0xf) & 1)) { //Wfck == 0
        wfck_mode = 1; // Target: PU-22 or newer
        return;
      }
    }
  }
}

// *****************************************************************************************
// Function: inject_SCEX
// Description:
// Injects SCEX data corresponding to a given region ('e' for Europe, 'a' for America,
// 'i' for Japan). This function is used for modulating the SCEX signal to bypass
// region-locking mechanisms.
//
// Parameters:
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
        // PC1 -> Data Output ---------------------------------------
        GPIOC->CFGLR &= ~(0xf << (4 * 1));
        GPIOC->CFGLR |= (3 | 0) << (4 * 1);
        //----------------------------------------------------------
        GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
        Delay_Us(bits_delay);
      }
      else
      {
        // For OLD boards, bit 1 is High-Z (Pin set as input)
        // PC1 <- Data Input -----------------------------------------------
        GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
        //-----------------------------------------------------------------
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
        // PC1 -> Data Output ---------------------------------------
        GPIOC->CFGLR &= ~(0xf << (4 * 1));
        GPIOC->CFGLR |= (3 | 0) << (4 * 1);
        //----------------------------------------------------------
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
       // PC1 -> Data Output ---------------------------------------
       GPIOC->CFGLR &= ~(0xf << (4 * 1));
       GPIOC->CFGLR |= (3 | 0) << (4 * 1);
       //----------------------------------------------------------
       
       uint8_t count = 30; 
       
       while (count--) {
      
        while((GPIOA->INDR >> (2 & 0xf) & 1)); //Wait for Falling Edge  //WfckRead
        GPIOC->BSHR = (1 << 16 + 1); // //PC1 -> Data Low
      
        while(!(GPIOA->INDR >> (2 & 0xf) & 1)); //Wait for Rising Edge //!WfckRead
        GPIOC->BSHR = (1 << 1); // //PC1 -> Data High
       
       }
      }
    }
  }
  // After injecting SCEX data, set DATA pin as output and clear (low)
  // PC1 -> Data Output ---------------------------------------
  GPIOC->CFGLR &= ~(0xf << (4 * 1));
  GPIOC->CFGLR |= (3 | 0) << (4 * 1);
  //----------------------------------------------------------
  GPIOC->BSHR = (1 << 16 + 1); // PC1 -> Data Low
  Delay_Ms(injections_delay);
}

int main()
{
  SystemInit();
  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

  // PC1 <- Data Input -----------------------------------------------
  // funPinMode(PC1, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
  //-----------------------------------------------------------------
  // PA2 <- Wfck Input -----------------------------------------------
  // funPinMode(PA2, GPIO_CNF_IN_FLOATING);
  GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
  //-----------------------------------------------------------------
  // PC4 <- Subq Input-----------------------------------------------
  // funPinMode(PC4, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (4 & 0xf))))) | (4 << (4 * (4 & 0xf)));
  //-----------------------------------------------------------------
  // PC2 <- Sqck Input-------------------------------------------------
  // funPinMode(PC2, GPIO_CNF_IN_FLOATING);
  GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
  //------------------------------------------------------------------
  // PD6 -> LED Output ------------------------------------------------
  GPIOD->CFGLR &= ~(0xF << (4 * 6));
  GPIOD->CFGLR |= (3 | 0) << (4 * 6);
  //------------------------------------------------------------------
  uint8_t hysteresis = 0;
  uint8_t scbuf[12] = {0}; // SUBQ bit storage
  uint16_t timeout_clock_counter = 0;
  uint8_t bitbuf = 0;
  uint8_t bitpos = 0;
  uint8_t scpos = 0; // scbuf position

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

    //************************************************************************
    // Check if read head is in wobble area
    // We only want to unlock game discs (0x41) and only if the read head is in the outer TOC area.
    // We want to see a TOC sector repeatedly before injecting (helps with timing and marginal lasers).
    // All this logic is because we don't know if the HC-05 is actually processing a getSCEX() command.
    // Hysteresis is used because older drives exhibit more variation in read head positioning.
    // While the laser lens moves to correct for the error, they can pick up a few TOC sectors.
    //************************************************************************

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

    // hysteresis value "optimized" using very worn but working drive on ATmega328 @ 16Mhz
    // should be fine on other MCUs and speeds, as the PSX dictates SUBQ rate
    if (hysteresis >= HYSTERESIS_MAX)
    {
      // If the read head is still here after injection, resending should be quick.
      // Hysteresis naturally goes to 0 otherwise (the read head moved).
      hysteresis = 11;

      //************************************************************************
      // Executes the region code patch injection sequence.
      //************************************************************************
      GPIOD->BSHR = (1 << 6); // PD6 -> LED High //Start Injection

      // PC1 -> Data Output ---------------------------------------
      GPIOC->CFGLR &= ~(0xf << (4 * 1));
      GPIOC->CFGLR |= (3 | 0) << (4 * 1);
      //----------------------------------------------------------
      GPIOC->BSHR = (1 << 16 + 1); // PC1 Data Low

      if (!wfck_mode) // If wfck_mode is fals (oldmode)
      {
        // PA2 -> Wfck Output -----------------------------------------------
        GPIOA->CFGLR &= ~(0xf << (4 * 2));
        GPIOA->CFGLR |= (3 | 0) << (4 * 2);
        //-----------------------------------------------------------------
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
        // PA2 <- Wfck Input ---------------------------------------------------
        // funPinMode(PA2, GPIO_CNF_IN_FLOATING);
        GPIOA->CFGLR = (GPIOA->CFGLR & (~(0xf << (4 * (2 & 0xf))))) | (4 << (4 * (2 & 0xf)));
        //----------------------------------------------------------------------
      }
      // PC1 <- Data Input ----------------------------------------------------
      // funPinMode(PC1, GPIO_CNF_IN_FLOATING);
      GPIOC->CFGLR = (GPIOC->CFGLR & (~(0xf << (4 * (1 & 0xf))))) | (4 << (4 * (1 & 0xf)));
      //-----------------------------------------------------------------------

      GPIOD->BSHR = (1 << 16 + 6); // PD6 -> LED Low //Injection done!
    }
  }
}
