# PsNee-CH32V003
Porting of PsNee to CH32V003 rev_3.1

PsNee v7 port to the ch32v003 MCU compatible  and stealth with all Ps1 motherboards!
Plus this code virtually don't introduces any noise or degradation of the laser RF signal level because the it injects the SCEX string only when needed.

I'm just a hobbyist programmer, so please forgive any coding mistakes, syntax errors, or other issues. The code is heavily based on PsNee v7/v8 by kalymos, 
but this time I've chosed to not follow the Arduino portability philosophy. 
Code is written using ch32fun libs avoiding HAL when possible. The result are less portability but a faster, lighter, efficient and overall way better code!

Why I haven't ported "postal" PsNee v8? Simply because JAP bios patching is bugged (with some BIOS menu crashes) and until a fix came out I'm not interested in a porting...

**Warning:**

- On JAP region consoles you can ONLY play japanese backups due to BIOS protection!
- Same story for the PAL Psone SCPH-102 wich can run only backup of PAL games, but here you can use my [OneNee_ch32v003](https://github.com/carmax91/OneNee_Ch32v003) port
  made specifically for this console to "unlock" all region reading...

## Features

- Full Stealth on ALL motherboards!
- Virtually any noise or degradation on the laser RF signal level unlike the old oldcrow\mayumi\multimode mod.
- Led injecting status output on chip PIN 1 (You need a 1K resistor).
- No atmega328p+16mhz cristal or other arduino uno like "big" board, simply a 8-pin ch32v003j4m6 chip to solder that can be easly fitted in your console!
- Bin files should be compatible even on other ch32v003 package (SOP-16, TSSOP-20 and QFN-20) and valutation board provided that you remove the external osc!

## Supported Playstations
- All Playstation models, but with the SCPH-102 and JAP variants import games aren't supported.
- For SCPH-102 models, you can use my [OneNee_ch32v003](https://github.com/carmax91/OneNee_Ch32v003) port wich "unlocks" all regions backup.

## Prerequisites
- [WCH-LinkE](https://github.com/carmax91/PsNee-CH32V003/blob/main/Imgs/WCH-LinkEPrg.jpg) programmer (pay attention to the E).
- WCH LinkUtility software.

## HowTo
- Download this repository.
- Install the WCH LinkUtility software and the releative drivers (all the the needed software/files are zipped in the "tool" directory).
- Connect the LinkE programmer and check if is in RISCV mode (Blue LED should be always off when idle).
- If not, disconect and reconnect the programmer holding down the "Mode-S" button.
- Open LinkUtility and check if the tool sees the programmer.
- Set the core (RISC-V) and the series (CH32V003).
- Click on the folder icon (or press ALT+F1)
- Select the BIN compiled files based on your console region (you can find the bin files in the [BIN](https://github.com/Crx91/PsNee-CH32V003/tree/main/BIN%20(rev_3)) folder).
- Connect the chip to the programmer.
- Program the chip via Target -> Program (or press F10).
![done](https://github.com/carmax91/PsNee-CH32V003/blob/main/Imgs/LinU2.png)
- Now you have your PsNee on WCH CH32V003 chip!
- For the installation, follow the wiring diagrams in the [Install](https://github.com/carmax91/PsNee-CH32V003/tree/main/Install) folder based on your console motherboard.

## Some info about the source code
I've ported the code to the ch32fun platform wich is lighter faster and ages better than the official WCH bugged arduino platform!
All in bare-metal and the difference is huge!!!!

- The older arduino sketch used 10480 bytes (63%) of program storage space and used 588 bytes (28%) of dynamic memory. 
- The rev_1 based on ch32fun uses 1728 bytes (10.3%) of program storage space and only 8 bytes (0.4%) of dynamic memory!
- The rev_2 has newer timing implementation made by the great @kalymos, code now is even smaller and no more ISR timing dipendent!
  The newer ch32fun uses around 1500 bytes (8.4) of program storage space and only 4 bytes (0.2%) of dynamic memory!
- The rev_3 has a newer board detection function with added 300ms stabilization delay to filter PU-7/20 power-up noise and sync with PU-22+ oscillating signals.
  And on the Injection function has updated WFCK modulation for 7.3/14.6 kHz compatibility (Again thanks to @kalymos!).
- The rev_3.1 has some code refactoring on the board detection function and WFCK modulation.

A very huge difference, because we don't have to carry anymore all the bloatware (super bugged) HAL of arduino ported libs! So now the code is way faster and efficient!

~ ~In the src directory you can find the .ino sketch, but at the moment is useless because the official wch arduino libs are a incomplete broken mess! 
They have completely broken functions and at the current state they are only usefull for a simple blink led or very basic GPIO playing (even blink no delay is bugged :P).
Those MCU are ATM are a total mess, minimal or no documentation, mounriver dependency and the arduino core seems totally abbandoned with all its issues :P
To compile the code i have fixed some libs manually and atm it uses HAL that i don't like much :P But anyway it works, so use precompiled HEX and you have no problems!~ ~


## Why this modc. is better and different compared to the older oldcrow\mayumi\multimode\onchip\stealth2.x? (extracted part from PsNeev7 readme)

The original code doesn't have a mechanism to turn the injections off. It bases everything on a timer. After power on, it will start sending injections for some time, then turns off. 
It also doesn't know when it's required to turn on again (except for after a reset), so it gets detected by anti-mod games.
The mechanism to know when to inject and when to turn it off.

This is the 2 wires for SUBQ / SQCK. The PSX transmits the current subchannel Q data on this bus. It tells the console where on the disc the read head is. We know that the protection symbols only exist on the earliest sectors, and that anti-mod games exploit this by looking for the symbols elsewhere on the disk. If they get those symbols, a modchip must be generating them!

So with that information, my code knows when the PSX wants to see the unlock symbols, and when it's "fake" / anti-mod. The chip is continously looking at that subcode bus, so you don't need the reset wire or any other timing hints that other modchips use. That makes it compatible and fully functional with all revisions of the PSX, not just the later ones. Also with this method, the chip knows more about the current CD. This allows it to not send unlock symbols for a music CD, which means the BIOS starts right into the CD player, instead of after a long delay with other modchips.

This has some drawbacks, though:

- It's more logic / code. More things to go wrong. The testing done so far suggests it's working fine though.
- It's not a good example anymore to demonstrate PSX security, and how modchips work in general.


## Thanks to:
ramapcsx2, kalymos, SpenceKonde, oldcrow, mayumi, arduino community, ch32fun community and lots of people that can't remeber now.
