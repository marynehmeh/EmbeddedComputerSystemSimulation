//Name: Mary Nehmeh ID: 9933977 Date: 10/10/2018

//pre-processor directives:
#include <iostream>
#include <thread>
#include <mutex>
#include <random>
#include <chrono>
#include <map>
#include <condition_variable>

using namespace std;

//global constants:
int const MAX_NUM_OF_THREADS = 6;
int const NUM_OF_SAMPLES = 50;
int const NUM_OF_LINKS = 2;

//global variables:
std::mt19937 gen(time(0));
std::uniform_int_distribution<>delay_random(1,10);
std::uniform_int_distribution<>temp_random(10,30);
std::uniform_int_distribution<>pres_random(95,105);
std::uniform_int_distribution<>cap_random(1,5);
std::uniform_int_distribution<>select_random(0,2);
std::mutex mu;
std::condition_variable cond;
std::map<thread::id, int> threadIDs;

int delay_rand = 0;
double temp_rand = 0;
double pres_rand = 0;
double cap_rand = 0;
int select_rand = 0;
int temp_counter = 0;
int pres_counter = 0;
int cap_counter =  0;
int id = 0;
double value = 0;
string type;
//function prototypes: (as required)

int search(void) {
    std::unique_lock<std::mutex> map_locker(mu);
    std::map <std::thread::id, int>::iterator it = threadIDs.find(std::this_thread::get_id());
    int i=0;
    if(it ==threadIDs.end())
       return -1;
    else
        return it->second;
} // used to search map of threads ids to give a number from 0 to 5 for the threads

class Sensor { //abstract base class that models a sensor
 public:
    Sensor(string& type) //constructor
        : sensorType(type) {}
 //Virtual method overridden by derived classes to return a random value dependant upon the different sensors
    virtual double getValue() = 0;
 //non-virtual method returning the type of the sensor
    string getType() {
        return sensorType;
    }
    string sensorType; //instance variables
}; //end abstract class Sensor

class TempSensor : public Sensor { //derived class from Sensor
 public:
    TempSensor (string& s) //constructor initialising the type of the sensor
        : Sensor(s) {}
    virtual double getValue() {
        temp_rand = temp_random(gen);//return a random value of ambient temperature between 10 and 30
        return temp_rand;
    }
}; //end class TempSensor

class PressureSensor : public Sensor { //derived class from Sensor
 public:
    PressureSensor (string& s) //constructor initializing the type of the sensor
    : Sensor(s) {}
    virtual double getValue() {
        pres_rand = pres_random(gen);//return a random value of ambient temperature between 95 to 105
        return pres_rand;
    }
}; //end class PressureSensor

class CapacitiveSensor : public Sensor { //derived class from Sensor
 public:
    CapacitiveSensor (string& s) //constructor initialising the type of the sensor
        : Sensor(s) {}
    virtual double getValue() {
        cap_rand = cap_random(gen);//return a random value of ambient temperature between 1 to 5
        return cap_rand;
    }
}; //end class CapacitiveSensor

class BC {
 public:
//constructor: initialises a vector of Sensor pointers that are
//passed in by reference:
    BC(std::vector<Sensor*>& sensors)
        : theSensors(sensors) {}
    void requestBC() {
        std::unique_lock<std::mutex> locker(BC_mu);
        while (lock) {
            id = search();
            cond.wait(locker);
            cout<<"BusController is locked, thread "<<id<<" is about to suspend.."<<endl;
        }
        lock = true;
        id = search();
        cout<<"BusController locked by thread "<<id<<endl;


    }
    double getSensorValue(int selector) {
        std::unique_lock<std::mutex> locker(BC_mu);
        return (*theSensors[selector]).getValue();
    }
    string getSensorType(int selector) {
        std::unique_lock<std::mutex> locker(BC_mu);
        return (*theSensors[selector]).getType();
    }
    void releaseBC() {
        std::unique_lock<std::mutex> locker(BC_mu);
        id = search();
        cout<<"BusController unlocked by thread "<<id<<endl;
        lock = false;
        cond.notify_all();
    }

 private:
    bool lock = false; //'false' means that the BC is not locked
    std::vector<Sensor*>& theSensors; //reference to vector of Sensor pointers
    std::mutex BC_mu; //mutex
}; //end class BC
//run function –executed by each thread:

class SensorData { //Utility class to store sensor data
 public:
    SensorData(string type) //Constructor
        : sensor_type(type) {}
    string getSensorType() {
        return sensor_type;
        }
    std::vector<double> getSensorData() {
            return sensor_data;
            }
    void addData(double newData) {
        sensor_data.push_back(newData);
    }

 private:
    string sensor_type;
    std::vector<double> sensor_data;
}; //end class SensorData

/*class LinkAccessController {
 public:
    LinkAccessController(Receiver& r) : myReceiver(r), numOfAvailableLinks(NUM_OF_LINKS){
        for (int i = 0; i < NUM_OF_LINKS; i++) {
            commsLinks.push_back(Link(myReceiver, i));
        }
    }
    //Request a comm's link: returns a reference to an available Link.
    //If none are available, the calling thread is suspended.
    Link& requestLink() {

        ....
        return std::ref(commsLinks[linkNum]);
    }
 //Release a comms link:
    void releaseLink(Link& releasedLink) {
    ....
    }
 private:
    Receiver& myReceiver; //Receiver reference
    int numOfAvailableLinks;
    std::vector<Link> commsLinks;
    std::mutex LAC_mu; //mutex
    ....
}; //end class LinkAccessController

class Link {
 public:
    Link (Receiver& r, int linkNum) //Constructor
        : inUse(false), myReceiver(r), linkId(linkNum){}
 //check if the link is currently in use
    bool isInUse() {
        return inUse;
    }
 //set the link status to busy
    void setInUse() {
        inUse = true;
    }
//set the link status to idle
    void setIdle() {
        inUse = false;
    }
 //write data to the receiver
    void writeToDataLink(SensorData sd) {
        ....
    }
 //returns the link Id
    int getLinkId() {
        return linkId;
    }
 private:
    bool inUse;
    Receiver& myReceiver; //Receiver reference
    int linkId;
}; //end class Link

class Receiver {
 public:
    Receiver () { } //constructor
 //Receives a SensorData object:
    void receiveData(SensorData sd) {
    ....
    }
 // print out all data for each sensor:
    void printSensorData () {
    ....
    }
 private:
//mutex:
....
 //vectors to store sensor numeric data received from threads:
    std::vector<double> temp_data;
    ....

}; //end class Receiver */
void run(BC& theBC, int id) {
     std::unique_lock<std::mutex> map_locker(mu);
     threadIDs.insert(std::make_pair(std::this_thread::get_id(),id));
     map_locker.unlock(); //mqpping thread ids to values from 0 to 6

     int i;
     for (i=0; i< NUM_OF_SAMPLES; i++) { // NUM_OF_SAMPLES = 50 (initially)

        // request use of the BC:
         theBC.requestBC();
        // generate a random value between 0 and 2, and use it to
        // select a sensor and obtain a value and the sensor's type:
        select_rand = select_random(gen);
        type = theBC.getSensorType(select_rand);
        value = theBC.getSensorValue(select_rand);
        id = search ();
        cout<<"sample value from thread "<<id<<" from "<<type<<" sensor "<<"= "<<value<<endl;
        // increment counter for sensor chosen (to keep count of
        // how many times it was used)
        if (select_rand == 0)
            temp_counter = temp_counter + 1;
        else if (select_rand == 1)
            pres_counter = pres_counter + 1;
        else if (select_rand == 2)
            cap_counter = cap_counter + 1;

        // release the BC:
        theBC.releaseBC();
        //thread sleeps for a period between 1-10 ms
        delay_rand = delay_random(gen);
        std::this_thread::sleep_for (std::chrono::milliseconds(delay_rand));

        if (select_rand == 0)
            theTemperatureData.addData(type);
        else if (select_rand == 1)
            thePressureData.addData(type);
        else if (select_rand == 2)
            theCapacitiveData.addData(type);

    }
}

int main(void) {
 //declare a vector of Sensor pointers:
 std::vector<Sensor*> sensors;
 //initialise each sensor and insert into the vector:
 string s = "temperature sensor";
 sensors.push_back(new TempSensor(s)); //push_back is a vector method.
 s = "pressure sensor";
 sensors.push_back(new PressureSensor(s));
 s = "capacitive sensor";
 sensors.push_back(new CapacitiveSensor(s));

 // Instantiate the BC:
 BC theBC(std::ref(sensors));

 //instantiate SensorData for temperature, pressure and capacitive sensors
 SensorData theTemperatureData("temperature");
 SensorData thePressureData("pressure");
 SensorData theCapacitiveData("capacitive");

 //instantiate the LinkAccessController

 //LinkAccessController lac () ** add to it the reference from the receiver
 //instantiate and start the threads:

 std::thread the_threads[MAX_NUM_OF_THREADS]; //array of threads
 for (int i = 0; i < MAX_NUM_OF_THREADS; i++) {
    the_threads[i] = std::thread(run,std::ref(theBC),i);
 }

 for (int i = 0; i < MAX_NUM_OF_THREADS; i++) {
    the_threads[i].join();
 }

 cout << "All threads terminated" << endl;
//print out the number of times each sensor was accessed:
 cout << "temperature sensor accessed: "<< temp_counter << " times" <<endl;
 cout << "pressure sensor accessed: "<< pres_counter << " times" <<endl;
 cout << "capacitive sensor accessed: "<< cap_counter << " times" <<endl;

 return 0;
}

