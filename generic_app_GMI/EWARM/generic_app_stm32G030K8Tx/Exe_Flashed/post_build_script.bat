REM Merge Application and Bootloader into One Hex File for Flashing 
"%~dp0"srec_cat.exe "%~dpn2.hex" -Intel "%~dp0STM32G030_BootLoader_With_GPIO.hex" -Intel -o "%~dpn2_withBootloader_forStLink.hex" -intel -obs=16
