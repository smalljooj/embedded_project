@echo off
cd /D C:\Users\anali\projeto_embarcados\embedded_project\build\esp-idf\esp_system || (set FAIL_LINE=2& goto :ABORT)
C:\Users\anali\.espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe C:/Users/anali/esp/v5.3.2/esp-idf/tools/ldgen/ldgen.py --config C:/Users/anali/projeto_embarcados/embedded_project/sdkconfig --fragments-list C:/Users/anali/esp/v5.3.2/esp-idf/components/riscv/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_gpio/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_pm/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_mm/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/spi_flash/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_system/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_system/app.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_common/common.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_common/soc.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_rom/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/hal/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/log/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/heap/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/soc/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_hw_support/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_hw_support/dma/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_hw_support/ldo/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/freertos/linker_common.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/freertos/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/newlib/newlib.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/newlib/system_libs.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_gptimer/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_ringbuf/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_uart/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/app_trace/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_event/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_pcnt/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_spi/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_mcpwm/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_ana_cmpr/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_dac/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_rmt/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_sdm/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_i2c/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_ledc/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_driver_parlio/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/driver/twai/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_phy/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/vfs/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/lwip/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_netif/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/wpa_supplicant/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_coex/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_wifi/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_adc/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_eth/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_gdbstub/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_psram/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/esp_lcd/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/espcoredump/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/ieee802154/linker.lf;C:/Users/anali/esp/v5.3.2/esp-idf/components/openthread/linker.lf --input C:/Users/anali/projeto_embarcados/embedded_project/build/esp-idf/esp_system/ld/sections.ld.in --output C:/Users/anali/projeto_embarcados/embedded_project/build/esp-idf/esp_system/ld/sections.ld --kconfig C:/Users/anali/esp/v5.3.2/esp-idf/Kconfig --env-file C:/Users/anali/projeto_embarcados/embedded_project/build/config.env --libraries-file C:/Users/anali/projeto_embarcados/embedded_project/build/ldgen_libraries --objdump C:/Users/anali/.espressif/tools/riscv32-esp-elf/esp-13.2.0_20240530/riscv32-esp-elf/bin/riscv32-esp-elf-objdump.exe || (set FAIL_LINE=3& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%