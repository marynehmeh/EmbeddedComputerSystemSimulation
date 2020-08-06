# Embedded Computer System Simulation

Project completed as part of the Concurrent Systems module completed during my 3rd Year at the University of Manchester. 

## Project Details 
As part of the project I modelled  a system that is responsible for gathering environmental data from a set of three sensors, each of a different type. In the model a set of threads are used to gather the data, and then transmit it to a Receiver.

Access to the sensors is via a Bus Controller (BC), such that only one thread is granted access at a time. Thus, in order to acquire data, each thread must gain sole access to the BC, configure it to sample from a specific sensor, and sample the data. In order to transmit the data, a thread must gain access to one of two communications links via a Link Access Controller. Having gained access, it sends its data, and then releases the link. Each thread may use either communications link, but once it has gained access to a particular link, no other thread should be allowed to use that link until it is released.

The solution to include access to the communications links. In particular, the threads should request a Link from a LinkAccessController object. When one is available, the sample values that the thread has acquired can be transmitted. The Link object is used to copy this data to the Receiver object. To facilitate this, each thread stores sensor data in objects of a class called SensorData, which stores the samples internally in a vector.
