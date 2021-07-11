.cpu cortex-m0
.thumb

none: b .

.section vectors
.word _estack               /* stack address */
.word Reset_Handler         /* Reset */
.word none                  /* NMI */
.word Hard_Fault            /* HardFault */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* SVCall */
.word none                  /* Reserved */
.word none                  /* Reserved */
.word none                  /* PendSV */
.word none                  /* SysTick */

.word none                  /* 0 */
.word none                  /* 1 */
.word none                  /* 2 */
.word none                  /* 3 */
.word none                  /* 4 */
.word none                  /* 5 */
.word none                  /* 6 */
.word none                  /* 7 */
.word none                  /* 8 */
.word none                  /* 9 */
.word dac_irq               /* 10 */
.word none                  /* 11 */
.word none                  /* 12 */
.word none                  /* 13 */
.word none                  /* 14 */
.word none                  /* 15 */
.word none                  /* 16 */
.word none                  /* 17 */
.word none                  /* 18 */
.word none                  /* 19 */
.word none                  /* 20 */
.word none                  /* 21 */
.word none                  /* 22 */
.word none                  /* 23 */
.word none                  /* 24 */
.word none                  /* 25 */
.word none                  /* 26 */
.word none                  /* 27 */
.word none                  /* 28 */
.word none                  /* 29 */
.word none                  /* 30 */
.word usb_irq               /* 31 */
