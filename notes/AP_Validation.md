# ADEPT-FC Autopilot Validation Procedure
*Grayson Schaer\
Bretl Research Group\
Aerodynamics and Unsteady Flow Group\
ESAero\
Created: 10/29/2020 13:50 CST\
Updated: 10/29/2020 13:50 CST*

---
## Unit Tests
_This section will describe the purpose, scope, and procedure relating to the autopilot unit tests._

**Purpose**

Unit tests are conducted to ensure:
1. All PWM input commands given by the autopilot are within an acceptable range and will not damage the physical servos.
2. Autopilot state rejection performs as expected.
3. Input commands calculated by the autopilot are in the expected direction.

These tasks are achieved by intercepting the states observed by the 5-hole probe and INS and replacing them with a set of test states. In total, 22 states are tested. The expected inputs are based on optimal controller simulation. The test points include:
1. Trim condition
  - *Expected behavior:* No state variable rejection.
  - *Expected input:* Trim input on all input axes.
2. Rejection of too negative state
  - *Expected behavior:* All state variables rejected. Replaced by last known good state (trim state)
  - *Expected input:* Trim input on all input axes.
3. Rejection of too large state
  - *Expected behavior:* All state variables rejected. Replaced by last known good state (trim state)
  - *Expected input:* Trim input on all input axes.
4. Positive pitch
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, positive inputs from elevator and throttle. **ELSE**, trim.
5. Negative pitch
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, negative inputs from elevator and throttle. **ELSE**, trim.
6. Positive roll
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, negative inputs from aileron, rudder, and left throttle. Positive inputs from right throttle. **ELSE**, trim.
7. Negative roll
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, positive inputs from aileron, rudder, and left throttle. Negative inputs from right throttle. **ELSE**, trim.
8. Positive yaw
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, negative inputs from aileron and left throttle. Positive inputs from rudder and right throttle. **ELSE**, trim.
9. Negative yaw
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, positive inputs from aileron and left throttle. Negative inputs from rudder and right throttle. **ELSE**, trim.
10. Positive pitch rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, positive inputs from elevator and throttle. **ELSE**, trim.
11. Negative pitch rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, negative inputs from elevator and throttle. **ELSE**, trim.
12. Positive roll rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, negative inputs from aileron, rudder, and left throttle. Positive inputs from right throttle. **ELSE**, trim.
13. Negative roll rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, positive inputs from aileron, rudder, and left throttle. Negative inputs from right throttle. **ELSE**, trim.
14. Positive yaw rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, positive inputs from aileron, rudder, and right throttle. Negative inputs from left throttle. **ELSE**, trim.
15. Negative yaw rate
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, negative inputs from aileron, rudder, and right throttle. Positive inputs from left throttle. **ELSE**, trim.
16. Positive angle of attack
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, negative input from elevator, positive input from throttle. **ELSE**, trim.
17. Negative angle of attack
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, positive input from elevator, negative input from throttle. **ELSE**, trim.    
18. Positive sideslip angle
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, negative input from aileron, rudder, and right throttle. Positive input from left throttle. **ELSE**, trim.
19. Negative sideslip angle
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on elevator. **IF CONTROLLED**, positive input from aileron, rudder, and right throttle. Negative input from left throttle. **ELSE**, trim.   
20. Positive velocity
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, negative inputs from elevator and throttle. **ELSE**, trim.
21. Negative velocity
  - *Expected behavior:* No state variables rejected.
  - *Expected input:* Trim input on aileron and rudder. **IF CONTROLLED**, positive inputs from elevator and throttle. **ELSE**, trim.
22. Trim condition
  - *Expected behavior:* No state variable rejection.
  - *Expected input:* Trim input on all input axes.

**Scope**

**Procedure**

---
## Calibration
_This section will describe the purpose, scope, and procedure relating to the autopilot calibration._

**Purpose**

**Scope**

**Procedure**

---
## HITL
_This section will describe the purpose, scope, and procedure relating to the autopilot HITL tests._

**Purpose**

**Scope**

**Procedure**

---
## Walkaround
_This section will describe the purpose, scope, and procedure relating to the ADEPT-FC walkaround check._

**Purpose**

**Scope**

**Procedure**
