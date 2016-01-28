SET PATH=C:\Keil\ARM\ARMCC\bin\;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Altium Designer Summer 09\System;C:\Program Files (x86)\Microsoft SQL Server\100\Tools\Binn\;C:\Program Files\Microsoft SQL Server\100\Tools\Binn\;C:\Program Files\Microsoft SQL Server\100\DTS\Binn\;D:\Program Files\TortoiseGit\bin;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Altium Designer Summer 09\System;C:\Program Files (x86)\Microsoft SQL Server\100\Tools\Binn\;C:\Program Files\Microsoft SQL Server\100\Tools\Binn\;C:\Program Files\Microsoft SQL Server\100\DTS\Binn\
SET CPU_TYPE=STM32F103C8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=ADC-DEMO
SET CPU_CLOCK=0x007A1200
"C:\Keil\ARM\ARMCC\bin\ArmAsm" --Via "..\output\startup_stm32f10x_hd._ia"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\main.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_it.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\usart1.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\adc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\led.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\agv_pwm.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\rc522.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\systick.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\time_test.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_gpio.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_rcc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_usart.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_adc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_dma.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_flash.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_tim.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_bkp.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\misc.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\stm32f10x_spi.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\core_cm3.__i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "..\output\system_stm32f10x.__i"
"C:\Keil\ARM\ARMCC\bin\ArmLink" --Via "..\Output\STM32-DEMO.lnp"
C:\Keil\ARM\ARMCC\bin\fromelf.exe "..\Output\STM32-DEMO.axf" --i32combined --output "..\Output\STM32-DEMO.hex"
