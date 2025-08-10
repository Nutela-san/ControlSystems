# English ControlSystems Change-Log  </br>

List of changes made to the ControlSystems library. </br>

## [v0.2.6] - 09/08/2025
- Bug fix in the anti-winding system for the integral term.
- Added the ChangeLog in spanish (CHANGELOG_ES.md) and english (CHANGELOG_EN.md).
- *reset()* method on SimplePID upgrated. Now you can set the value of past-error (default value is 0).


## [v0.2.1 a v0.2.5] - 07/02/2024 
- Added the "SimpleFilters.h" library, which implements 3 types of digital filters for the signal processing (MA, EMA and RC).
- Added one example for the use of "SimpleFilters.h".
- Changed the class name from "PIDControl.h" to "SimplePID.h".
- Changed the way to include the library for the PID control from "ControlSystems.h" to "SimplePID.h".
- Now SimplePID implements a reset method for the internal varibles of the PID control.


## [v0.2.0] - 21/04/2023
- First functional version of the library.
- Added use examples.
- Registed on the "PlatformIO Registry" package manager.
- Added a proyect brach for the Arduino IDE library on .zip format.
