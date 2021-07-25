# ADEPT-FC Autopilot Validation Procedure
*Grayson Schaer\
Bretl Research Group\
Aerodynamics and Unsteady Flow Group\
ESAero\
Created: 10/29/2020 13:50 CST\
Updated: 05/18/2021 21:27 CST*

---
## Quasi HITL Tests
_This section will describe the purpose, scope, and procedure relating to the autopilot Quasi HITL tests._

TEST CONDUCTED ON:

TEST CONDUCTED BY:

**Purpose**

Quasi HITL tests are conducted to ensure:
1. All PWM input commands given by the autopilot are within an acceptable range and will not damage the physical servos.
2. Input commands calculated by the autopilot are in the expected direction.
3. Input commands calculated by the autopilot zero state error in a desired time

These tasks are achieved by intercepting the states observed by the 5-hole probe and INS and replacing them with a set of test initial states that are time propogated by FE integration of the linear system dynamics. In total, 16 initial states are tested.
1. Trim condition
2. + Pitch (up)
3. - Pitch (down)
4. + Roll (right)
5. - Roll (left)
6. + Angle of Attack (up)
7. - Angle of Attack (down)
8. + Sideslip Angle (right)
9. - Sideslip Angle (left)
10. + Airspeed
11. - Airspeed
12. + Longitudinal States
13. + Lateral States
14. + All Attitude States
15. + All States
15. - All States

**Scope**

These tests cannot verify that the control surfaces deflect in the proper direction, only that the proper directional command is sent to them based on the standard deflection directions. It is up to the user to confirm proper directional calibration of control surfaces by verifying expected behavior.

**Procedure**

- In `config_files/autopilot_main.config` input name of `gains.dat` to be used for test
- In terminal `make clean`
- In terminal `git clean -fxd`
- In terminal `make test`
- In terminal `sudo ./run.sh`
- When autopilot module is loaded the text "WARNING autopilot started in test mode. DO NOT FLY." will appear
- In terminal `pwm arm`
- On TX, SC3 to AP+AT
- On TX, pull SH to engage AP+AT. Hold down SH until steady state achieved, approx. 10 seconds.
- On TX, release SH to complete first Quasi HITL test.
- Repeat previous two steps until all 16 tests are done. The current test number, AP+AT status, and input trim conditions will be displayed to stdout.

---
## Walkaround Inspection
_This section will describe the purpose, scope, and procedure relating to the ADEPT-FC walkaround inspection._

TEST CONDUCTED ON:

TEST CONDUCTED BY:

**Purpose**

This inspection ensures that there is no visible damage to the airframe, wings, doors, skin, control surfaces, or propulsors of the aircraft. It also ensures that the control surfaces behave as expected.

**Scope**

This inspection does not test the health of the software systems. It also does not inspect the calibrations for any hardware components

**Procedure**

- [ ] At nose of aircraft, remove 5 hole probe cover. Ensure all 5 holes are open and not obstructed. Return cover.
- [ ] Inspect 5 hole probe support beam for any signs of damage to surface or structure. Ensure that is it firmly attached to aircraft.
- [ ] Remove nose shield, inspect pneumatic cables for kinks, tears, or other signs of damage.
- [ ] Inspect battery harnesses for signs of fraying, adapter damage, or insulation damage.
- [ ] Return nose shield.
- [ ] Inspect nose gear for signs of wall damage. Ensure that there are no flat spots on wheel.
- [ ] Inspect front windscreen for signs of cracking or delamination.
- [ ] Ensure the left wing is secured to fuselage by wing nut.
- [ ] Inspect upper surface of left wing for any signs of cracking, tearing, or delamination.
- [ ] Inspect each motor and motor duct. Ensure free of debris and damage.
- [ ] Ensure each left wing propulsor freely spins.
- [ ] Ensure left aileron freely moves.
- [ ] Inspect lower surface of left wing for any signs of cracking, tearing, or delamination.
- [ ] Ensure left flap moves freely.
- [ ] Ensure the left hall effect sensor tape is in tact with no signs of tearing or delamination.
- [ ] Ensure the right wing is secured to fuselage by wing nut.
- [ ] Inspect upper surface of right wing for any signs of cracking, tearing, or delamination.
- [ ] Inspect each motor and motor duct. Ensure free of debris and damage.
- [ ] Ensure each right wing propulsor freely spins.
- [ ] Ensure right aileron freely moves.
- [ ] Inspect lower surface of right wing for any signs of cracking, tearing, or delamination.
- [ ] Ensure right flap moves freely.
- [ ] Ensure the right hall effect sensor tape is in tact with no signs of tearing or delamination.
- [ ] Inspect the upper surface of the fuselage. Ensure there are no visible signs of cracking. Ensure all tape is free of tears and delamination. Ensure all holes in fuselage are covered by tape.
- [ ] Inspect the left surface of the fuselage. Ensure there are no visible signs of cracking. Ensure all tape is free of tears and delamination. Ensure radio probes are covered by tape.
- [ ] Inspect the right surface of the fuselage. Ensure there are no visible signs of cracking. Ensure all tape is free of tears and delamination.
- [ ] Inspect the lower surface of the fuselage. Ensure there are no visible signs of cracking.
- [ ] Inspect both main gear for signs of wall damage. Ensure that there are no flat spots on either wheel.
- [ ] Inspect the upper surface of the elevator. Ensure there are no visible signs of cracking.
- [ ] Ensure the left elevator moves freely.
- [ ] Ensure the right elevator move freely.
- [ ] Inspect the lower surface of the elevator. Ensure that there are no visible signs of cracking.
- [ ] Ensure all 4 screws securing left elevator servo to elevator are secure.
- [ ] Ensure screw securing left elevator is secure.
- [ ] Ensure the left hall effect sensor tape is in tact with no signs of tearing or delamination.
- [ ] Ensure all 4 screws securing right elevator servo to elevator are secure.
- [ ] Ensure screw securing right elevator is secure.
- [ ] Ensure the right hall effect sensor tape is in tact with no signs of tearing or delamination.
- [ ] Inspect rudder surface. Ensure that there are no visible signs of cracking.
- [ ] Ensure rudder moves freely.
- [ ] Ensure the rudder hall effect sensor tape is in tact with no signs of tearing or delamination.
- [ ] Download most recent flight release: https://github.com/tbretl/adept-fc.git
- [ ] In `config_files/autopilot_main.config` input name of `gains.dat` to be used for test
- [ ] In terminal `make clean`
- [ ] In terminal `git clean -fxd`
- [ ] In terminal `make all`
- [ ] In terminal `sudo ./run.sh`
- [ ] In terminal `pwm arm`
- [ ] Deflect each control surface in both directions to the limit using the TX. Ensure each control surface behaves as expected.
- [ ] Fully actuate flaps. Ensure both behave as expected.
- [ ] On TX, SC3 to AP arm. Ensure `AP Armed` message on stdout.
- [ ] On TX, SC3 to AP+AT arm. Ensure `AP+AT Armed` message on stdout.
- [ ] On TX, SH to engage. Ensure `AP Engaged` message on stdout.
- [ ] On TX, SH to disengage, SC3 to disarm. Ensure `AP Disengaged` and `AP+AT disarmed` messages on stdout.
- [ ] In terminal `pwm disarm`
- [ ] In terminal `all exit`
