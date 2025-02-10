# nRF24L01+ Raspberry PI Pico C/C++ SDK library
This repository contains a driver library for the nRF24L01+ written in the Raspberry Pi Pico C/C++ SDK.
The library only abstracts key functionality, this is done to avoid overhead due to abstraction layers.
This requires the user to have a litte knowledge of the nRF24L01+. The datasheet in this repository
provides all needed information.

## Notes
This library has not implemented all, but most, features of the nRF24L01+.
At this moment the implementation for flushing the RX/TX FIFO,
payloads with no acknowledgement and dynamic payload widths is missing.
Example MACROs for these features are included so they can be implemented with relative ease.
The datasheet is included in this repositiory which can help with implementing the missing features.
There is no guarantee the missing features will be updated in the future.
