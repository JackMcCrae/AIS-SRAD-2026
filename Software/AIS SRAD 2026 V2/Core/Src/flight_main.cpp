#include <iostream>
#include "stm32f2xx_hal.h"
#include "main.h"
#include "pinset.h"
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;
// Define STM32 pins used in Actuators below
pinset Buzzer(GPIOE, GPIO_PIN_4);
pinset Drogue(GPIOE, GPIO_PIN_2);
pinset MainChute(GPIOE, GPIO_PIN_3);
// Define five stages in flight
enum flight_status{
    pre_launch,
    burn,
    coast,
    descent,
    standby};
flight_status currentState = flight_status::pre_launch;
//-------------------------------
//Insert division and  subtractions here
//leave potential to change to doubles
int safeSubtract(int augend, int addend){
    return augend + (~addend + 1);
}
float safeSubtractFloat(float augend, float addend) {
    return augend + (addend * -1.0f);
}
float safeDivide(float numerator, float denominator) {
    float x = denominator;
    int i;
    std::memcpy(&i, &x, sizeof(int));
    i = safeSubtract(0x7EEEEEEE,i);
    float y;
    std::memcpy(&y, &i, sizeof(float));
    y = y * (safeSubtractFloat(2.0f, x * y));
    return numerator * y;
}
//-------------------------------
//Kalman filter here
vector<float> runKalmanFilter(){
    vector<float> measured;
	ifstream file("Simulated and Real Value Comparisson.csv");
	string line;
	getline(file, line);
	stringstream header(line);
	string column;
	vector<string> headers;
	vector<float> realdata;
	
	while (getline(header, column, ','))
		headers.push_back(column);
		
	int index = 0;
	
	for (size_t i = 0; i < headers.size(); i++)
	{
		if (headers[i].find("real altitude") != string::npos)
			index = i;
	} 
	
	while (getline(file, line))
	{
		stringstream ss(line);
		string value;
		vector<string> row;

		while (getline(ss, value, ','))
			row.push_back(value);

		if (row.size() > index)
		{
			measured.push_back(stod(row[index]));
		}
	} 
	if (measured.empty()) {
		return vector<float>(); 
	}

	float x = measured[0]; //estimated state
	float q = 0.01; // noise from the system
	float r = 0.1;  // measurement noise
	float p = 1;    // estimation uncertainty
	float k;        // kalman gain
	
	vector<float> filtered;

	for (size_t i = 0; i < measured.size(); i++) {
		float z = measured[i];
		p = p + q;
		k = p / (p+r);
		x = x+k * (z-x);
		p = (1-k) * p;
		filtered.push_back(x);
	}
	return filtered;
}
//-------------------------------
//-------------------------------
//Sensors
//-------------------------------
float readAccelerometer(){
    //insert accelerometer drive here
    return 0;
}
float readAltitude(){
    //insert altitude drive here
    return 0;
}
//-------------------------------
//Actuators
extern TIM_HandleTypeDef htim6; 
void activateBuzzer() {
    HAL_TIM_Base_Start_IT(&htim6); 
}

void deactivateBuzzer() {
    HAL_TIM_Base_Stop_IT(&htim6); 

    Buzzer.reset(); 
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4); 
    }
}
void deployDrogue() {
    Drogue.set_high();
}

void deployMainChute() {
    MainChute.set_high();
}
//Dummy Actions Below

//void activateBuzzer() {
//    std::cout << "Buzzer ON" << std::endl;
//}

//void deployDrogue() {
//    std::cout << "Drogue Deployed" << std::endl;
//}

//void deployMainChute() {
//   std::cout << "Main Chute Deployed" << std::endl;
//}
void transmitData(){
    std::cout << "Data Transmitting" << std::endl;
}
//-------------------------------
//-------------------------------
//Prelaunch 
void handlePreLaunch() {
    float accel = readAccelerometer();
    std::cout << "PRE_LAUNCH";
    if (safeSubtractFloat(accel, 10.0f) > 0) {
        std::cout << "SWITCH TO BURN" << std::endl;
        currentState = flight_status::burn;
    }
}
//-------------------------------
//Burn
void handleBurn() {
    float accel = readAccelerometer();

    if (safeSubtractFloat(accel, 3.0f) < 0) {
        std::cout << "SWITCH TO COAST" << std::endl;
        currentState = flight_status::coast;
    }
}
//-------------------------------
//Coast
void handleCoast() {
    static float prevAltitude = 0;
    static bool firstRun = true;
    
    if (firstRun) {
        prevAltitude = readAltitude();
        firstRun = false;
        return; 
    }
    float currentAltitude = readAltitude();

    float rawVelocity = safeDivide(safeSubtractFloat(currentAltitude, prevAltitude),10);
    prevAltitude = currentAltitude;

    float filteredVelocity = runKalmanFilter(rawVelocity);
    if (filteredVelocity <= 0) {
        std::cout << "SWITCH TO DESCENT" << std::endl;
        currentState = flight_status::descent;
    }
}
//------------------------------
//Descent
void handleDescent() {
    static bool drogueDeployed = false;
    static bool mainchuteDeployed = false;
    if (!drogueDeployed) {
        deployDrogue();
        drogueDeployed = true;
        std::cout << "DROGUE DEPLOYED" << std::endl;
    }

    float altitude = readAltitude();

    if (safeSubtractFloat(altitude, 450.0f) <= 0 && !mainchuteDeployed) {
        deployMainChute();
        mainchuteDeployed = true;
        std::cout << "MAIN CHUTE DEPLOYED" << std::endl;
        currentState = flight_status::standby;
    }
}
//------------------------------
//Standby
void handleStandby(){
    transmitData();
    std::cout << "SYNCING DATA" << std::endl;
}
//------------------------------
//Main
extern "C" void flight_main() {
    //Change these two to somewhere else, leave main only with switch currentState
    std::cout << "SYS POWER ON..." << std::endl;
    activateBuzzer();
    //
    while (true) {
        switch (currentState) {
            case flight_status::pre_launch:
                handlePreLaunch();
                break;
            case flight_status::burn:
                handleBurn();
                break;
            case flight_status::coast:
                handleCoast();
                break;
            case flight_status::descent:
                handleDescent();
                break;
            case flight_status::standby:
                handleStandby();
                break;
        }
        
        HAL_Delay(10); 
    }
}
