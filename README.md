# OMPM v2

**Open Multi Power Meter V2**

OMPM v2 is an open hardware and firmware instrument for electrical load monitoring. It combines an ESP32 controller with six PZEM004T v3.0 measurement modules, a color TFT display, physical navigation buttons, microSD logging, environmental sensing, and a web interface. The design supports a physically measured aggregate channel alongside five individual loads, or six individual channels with an aggregate calculated from their readings.

## Repository contents

This repository provides the materials needed to inspect and reproduce the instrument:

- Schematics and PCB design files for the controller and PZEM module boards.
- Manufacturing files for PCB fabrication.
- A bill of materials with component quantities and approximate prices.
- ESP32 firmware for acquisition, display, configuration, calibration, logging, and web access.
- Example configuration files for network, aggregate-channel, MQTT, and calibration settings.

The six PZEM channels use the addresses `0x10`, `0x60`, `0x50`, `0x40`, `0x30`, and `0x20` for M0–M5, respectively. Jumpers on the measurement board allow the modules to be isolated individually while their addresses are assigned. During normal operation, `config.json` holds the operating settings and `calib.json` holds the per-channel correction factors.

OMPM v2 is presented as a research instrument whose design can be built, examined, and adapted for other monitoring experiments. The repository should be used together with the assembly instructions, the component documentation, and the measurement scope reported in the accompanying paper.

## Publications

1. Rodríguez-Navarro, C.; Alcayde, A.; Isanbaev, V.; Castro-Santos, L.; Filgueira-Vizoso, A.; Montoya, F. G. “DSUALMH—A New High-Resolution Dataset for NILM.” *Renewable Energy and Power Quality Journal*, **21**(1), 238–243, 2023. DOI: [10.24084/repqj21.286](https://doi.org/10.24084/repqj21.286).

2. Rodríguez-Navarro, C.; Portillo, F.; Martínez, F.; Manzano-Agugliaro, F.; Alcayde, A. “Development and Application of an Open Power Meter Suitable for NILM.” *Inventions*, **9**(1), 2, 2024. DOI: [10.3390/inventions9010002](https://doi.org/10.3390/inventions9010002).

3. Rodriguez-Navarro, C.; Portillo, F.; Martínez, F.; Manzano-Agugliaro, F.; Alcayde, A. “The Design, Creation, Implementation, and Study of a New Dataset Suitable for Non-Intrusive Load Monitoring.” *Applied Sciences*, **15**(13), 7200, 2025. DOI: [10.3390/app15137200](https://doi.org/10.3390/app15137200).

4. Rodríguez-Navarro, C.; Portillo, F.; Castro-Santos, L.; Filgueira-Vizoso, A.; Montoya, F. G.; Alcayde, A. “Optimising energy efficiency enhancing NILM through high-resolution data analytics.” *Renewable Energy and Power Quality Journal*, **22**(4), 85–91, 2024. DOI: [10.52152/4013](https://doi.org/10.52152/4013).

5. Rodriguez-Navarro, C.; Portillo, F.; Robalo, I.; Alcayde, A. “Evaluation of Traditional and Data-Driven Algorithms for Energy Disaggregation Under Sampling and Filtering Conditions.” *Inventions*, **10**(3), 43, 2025. DOI: [10.3390/inventions10030043](https://doi.org/10.3390/inventions10030043).

6. Rodriguez-Navarro, C.; Portillo, F.; Martínez-Gil, F.; Gil, C.; Manzano-Agugliaro, F.; Alcayde, A. “Advances in NILM dataset evaluation: a comparative review within the NILMTK framework.” *Electrical Engineering*, **108**, 163, 2026. DOI: [10.1007/s00202-025-03476-y](https://doi.org/10.1007/s00202-025-03476-y).

7. Rodriguez-Navarro, C.; Portillo, F.; Soler Ortiz, M.; Alcayde, A. “A comparative assessment of open-hardware and commercial energy meters for non-intrusive load monitoring.” *Measurement*, **284**, 122275, 2026. DOI: [10.1016/j.measurement.2026.122275](https://doi.org/10.1016/j.measurement.2026.122275).

8. Rodriguez-Navarro, C.; Portillo, F.; Robalo Cabrera, I.; Alcayde, A. “Integration of High-Frequency and Harmonic Measurements from Open-Source Meters into NILMTK Datasets.” *Renewable Energy and Power Quality Journal*, **26**(3), 385–390, 2026. DOI: [10.24084/reepqj26-35](https://doi.org/10.24084/reepqj26-35).

9. Rodriguez-Navarro, C.; Portillo, F.; Soler-Ortiz, M.; Alcayde, A. “Office plug-load dataset for non-intrusive load monitoring in buildings using an open metering platform.” *Energy and Buildings*, 2026. DOI: [10.1016/j.enbuild.2026.118300](https://doi.org/10.1016/j.enbuild.2026.118300).
