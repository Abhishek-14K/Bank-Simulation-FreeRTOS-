# Bank-Simulation-FreeRTOS-

Using STM32L476RG, simulate a virtual bank with three tellers. Using FreeRTOS, ensure the operation of each teller runs on an individual thread, any extra threads can also be used as required. Customers will enter the bank at 9 AM forming queue until 4 PM when the bank closes. The customers will enter at random intervals and the tellers will take breaks in a random fashion as long as they are not actively serving a customer, which also is a random interval. The MFS is to be used for its buttons and 7 segment display. Each of the buttons corresponds to a break a teller will take which is priority and the queue depth is to be displayed on the display. At the end of the operation, the final stats are to be printed on the console window. The simulated time of 1 minute should be equal to 100 ms of real time.

Refer to the src and inc folders for the source code.
