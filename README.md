# lis2mdl-hal

embedded-hal driver for the LIS2MDL magnetometer

## Support
* [ ] synchronous API
  * not planned to be supported, as I have no use for it
  * feel free to make a PR for it
* [x] asynchronous API
  * [x] interrupt support
  * [x] full register access
    * only missing updating hard-iron offset registers
