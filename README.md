# symax_sri_app
SyMAX-SRi Application Board Firmware

## Status

Status Last Updated - 03-11-2021 (OG): 
For the SyMAX-SRi Alpha build, this project is in design freeze. Minimal implemented features are limited to sending Speed commands with Modbus as the source.

### Flex-Mouse Core: 
> **Status:** Stable, In-Development,

    - Has not undergone extensive loaded testing yet.

    - Notes: Interrupt Systems was just updated. 'Memory' systems appear stable but are still being testing in individual modules. 

### module_analog_0_10v: Working, Debugging in process
> **Status:** Disabled

    - Calculates Demand properly from voltage input

    - Detailed Features Have not Completed Testing (Various Enable/ Disable Flags), slope off by one sample in one direction

### module_app: 
> **Status:** Unused

    - This app does not currently provide any vital function, and is currently not inserted in to the 'scheduler, so it does not run.

### module_modbus:
> **Status:** Functional

    - Some basic framework has been set in place, but has not been debugged exhaustively.

    - Access to Structured/Sequential memory may have pointer errors, if implemented. More in depth details needed here.

### module_motor_com:
> **Status:** Functional

    - UART Ports are configured and data can be seen as sent.

    - Access to Structured/Sequential memory likely has pointer errors and/or errors with initialization calls
