ESP-OPEN-RTOS Library for the electrical energy measurement chip ADE7953

The ADE7953 is a high accuracy electrical energy measurement IC intended for single phase applications.
It measures line voltage and current and calculates active, reactive, and apparent energy, as well as instantaneous rms voltage and current.

The library currently supports measuring of RMS Voltage and RMS Current for channel 1 and 2.
It is planned to deliver also the Energy values, however, the values are still not calculated correctly.
Also there is a function to collect all values at once, which is not yet verified for correct working state.

It is work in progress, however the develop was stopped, because of my transition to ESP8266_RTOS_SDK and ESP-IDF. It is possible to finish it as a version for the new SDKs.
