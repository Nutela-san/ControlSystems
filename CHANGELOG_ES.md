# Change-Log en español para ControlSystems </br>
Lista de los cambios realizados en la librería de ControlSystems. </br>

## [v0.2.6] - 09/08/2025
- Correción de bug en sistema anti-acomulación del ternino integral.
- Se añadio el ChangeLog en español (CHANGELOG_ES.md) he inglés (CHANGELOG_EN.md).
- Método *reset()* en SimplePID mejorado. Ahora se puede definir el valor del error anterior (valor predeterminado se configura a 0).


## [v0.2.1 a v0.2.5] - 07/02/2024 
- Se añadío la libreria "SimpleFilters.h" que implementa 3 tipos de filtros digitales para el procesamiento de señales (MA, EMA y RC).
- Se añadío un ejemplo para uso de "SimpleFilters.h".
- Cambio de nombre de la clase "PIDControl" a "SimplePID".
- Cambio en la forma de incluir la librería para el control PID de "ControlSystems.h" "SimplePID.h"
- Ahora SimplePID implementa un método de reset para las varibles internas del control PID.


## [v0.2.0] - 21/04/2023
- Primera versión funcional de librería.
- Se añadieron ejemplos de uso.
- Se registró en el administrador de paquetes de "PlatformIO Registry".
- Se añadio una rama de proyecto para la libreria para Arduino IDE en formato .zip
