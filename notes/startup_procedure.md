# Start Up Procedure for ADEPT-FC
*Grayson Schaer\
Bretl Research Group\
Aerodynamics and Unsteady Flow Group\
ESAero\
Created: 09/24/2020 13:08 CST\
Updated: 10/28/2020 10:39 CST*

---

## Sub-Procedure 1: Attach Wings

### Required Items

|  Item  |  Quantity  |  Purpose/Description  | Storage Location  |
|:------:|:----------:|-----------------------|-------------------|
|ADEPT-FC Aircraft|X1|Test aircraft|On mobile storage cart: <br> Upper shelf|
|Left Wing|X1|Left wing of the Adept-FC aircraft|On mobile storage cart: <br> Lower shelf|
|Right Wing|X1|Right wing of the Adept-FC aircraft|On mobile storage cart: <br> Upper shelf|
|Carbon Fiber Rod|X1|Loading bearing rod used to attach wings|On mobile storage cart: <br> Upper shelf|
|Wing Nut|X2|Nut used to secure wings to body|On mobile storage cart:<br> In plastic bag on upper shelf|
|Washer|X2|Washer used to spread load of wing nut(s)|On mobile storage cart:<br> In plastic bag on upper shelf|


### Procedure
- Remove *ADEPT-FC Aircraft* from mobile storage cart and place on ground.

- Remove both doors from *ADEPT-FC Aircraft*.
- Slide the *Carbon Fiber Rob* through the center hole of the wing root on the *ADEPT-FC Aircraft*. Avoid touching the cut-off faces of the tube as they can give carbon fiber splinters.
- With two hands distributing the load as evenly and widely as possible, lift the *Left Wing* and slide it onto the left side of the *Carbon Fiber Rod*. Pass through:
  - 4 propulsor power connectors
  - 4 propulsor PWM cables
  - 1 shielded Hall-Effect sensor servo cable
  - 1 left flap servo cable (blue tape, "LF")
  - 1 left aileron servo cable (yellow and red tape, "LA")
- Secure *Left Wing* to *ADEPT-FC Aircraft* with *Wing Nut* and *Washer*. Only finger tighten. Over tightening can damage wing root.
- With two hands distributing the load as evenly and widely as possible, lift the *Right Wing* and slide it onto the right side of the *Carbon Fiber Rod*. Pass through:
  - 4 propulsor power connectors
  - 4 propulsor PWM cables
  - 1 shielded Hall-Effect sensor servo cable
  - 1 right flap servo cable (green tape, "RF")
  - 1 right aileron servo cable (red tape, "RA")
- Secure *Left Wing* to *ADEPT-FC Aircraft* with *Wing Nut* and *Washer*. Only finger tighten. Over tightening can damage wing root.
- Attach all PWM cables from both wings (labeled 1-8) to their associated PWM ports. The ports are fixed just inside of the wing root and also labeled 1-8.
- Attach the "LA", "LF", "RA", and "RF" servo cables to the associated ports. The ports are color coded and labeled.
- Connect the *Left Wing* Hall effect sensor into the ADC riser port 1. The partially exposed face should face the right of the aircraft.
- Connect the *Right Wing* Hall effect sensor into the ADC riser port 0. The partially exposed face should face the right of the aircraft.
- Remove the nut from the ADC board riser grounding bolt.
- Attach both grounding rings from the *Left Wing* and *Right Wing* Hall effect sensors to the grounding bolt on the ADC board riser.
- Replace the nut to the ADC board riser grounding bolt.
---

## Sub-Procedure 2: Batteries

### Required Items

|  Item  |  Quantity  |  Purpose/Description  | Storage Location  |
|:------:|:----------:|-----------------------|-------------------|
|ADEPT-FC Aircraft|X1|Test aircraft|On mobile storage cart: <br> Upper shelf|
|Lipo Battery Voltage Tester|X1|Used to test the voltages of the Lipo batteries|In lipo pelican case in flammables cabinet|
|7.4V Lipo|X5|Powers all avionics systems|In lipo pelican case in flammables cabinet|
|18.5V Lipo|X4|Powers all propulsors|In lipo pelican case in flammables cabinet|
|4-8 Power Cable Splitter|X1|Splits 18.5V connectors to 8 outputs (1 per propulsor)|On mobile storage cart: <br> Upper shelf|

### Procedure

#### Always:
- Test battery voltages: https://www.google.com/search?channel=fs&client=ubuntu&q=using+lipo+battery+voltage+tester#kpvalbx=_rvlsX7O0OZnPtQayx4OwCQ40

- Attach the *7.4V lipo* labeled "SE 1" to the servo power distribution board power header. This is the head nearest the left side of the aircraft on the front of the PDB. **BEFORE PLUGGING IN, CHECK POLARITY**. Secure on velcro strip near front right of aircraft.
- Attach the *7.4V lipo* labeled "ProLite RX" to the servo power distribution board power header. This is the head nearest the left side of the aircraft on the back of the PDB. **BEFORE PLUGGING IN, CHECK POLARITY**. Secure on velcro strip near front right of aircraft.
- Attach the *7.4V lipo* labeled "SE 2" to the ADC board power connector. Secure on velcro strip.
- Attach the *7.4V lipo* labeled "Pi 1" to the the Pi's primary power source. Secure on velcro strip near front left of aircraft.
- Attach the *7.4V lipo* labeled "Pi 2" to the the Pi's backup power source. Secure on velcro strip near front left of aircraft.

#### If flying:
- Attach the *4-8 Power Cable Splitter* to each of the 8 motor power connectors.

- Place 2 18.5V lipos in the nose shield of the *ADEPT-FC Aircraft*. Ensure the selected 18.5V lipos are the ones with velcro on their skinny face. Attach their power connects to the adapters located in the shield. Be careful not to pinch or damage the pressure transducers or their cables.
- Place 1 *18.5V lipo* in the front of the aircraft on the center line (on a small piece of velcro).
- Place 1 *18.5V lipo* in the rear of the aircraft off of the center lines towards the right wing (on a small piece of velcro)
- Attach all 4 *18.5V lipo* power connectors to the *4-8 Power Cable Splitter*. **WARNING: THE PROPULSORS ARE NOW POWERED**.

---

## Sub-Procedure 3: Startup

### Required Items

|  Item  |  Quantity  |  Purpose/Description  | Storage Location  |
|:------:|:----------:|-----------------------|-------------------|
|HDMI Monitor|X1|Allows user to interface with Pi|On table near mobile storage cart|
|HDMI Cable|X1|Connects Pi to Monitor|Stored with monitor|
|USB Keyboard|X1|Allows user to interact with Pi|On table near mobile storage cart|

### Procedure
- Attach HDMI cable to Pi

- Attach keyboard to Pi
- Attach HDMI Monitor to HDMI cable connected to Pi
- Wait for Pi to boot. If it has not booted, leave the keyboard and monitor attached and restart the device by removing and reconnecting its batteries.
---

## Sub-Procedure 4: Interfacing

### Procedure
- Login: pi

- Password: *********
- `cd adept-fc/`
- `git branch`
- `git checkout autopilot_dev`
- `git fetch`
- `git pull`
- `make all`
- Ensure all SC on TX are fully forward
- `sudo ./run.sh`
- `pwm arm`
- Begin autopilot test by moving SC fully backward
- End autopilot test by moving SC fully forward
- `pwm disarm`
- `all exit`
- Shut down system
