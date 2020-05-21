/*
 * 
 * This is a CO2 sensor from the Candle project.
 * 
 * You can attach both a carbon monoxide and a carbon dioxide sensor.
 * 
 * Carbon monoxide is a dangerous, poisonous gas which is completely odourless. It is often formed when something is burning, but doesn't burn with enough oxygen.
 * 
 * Carbon dioxide is what we breathe out. Plants consume carbon dioxide to grow. High levels of carbon dioxide can influence how you feel.
 * 
 * Do not use this device as your sole carbon monoxide sensor! Use it as a support to your main carbon monoxide sensor only.
 * 
 * SETTINGS */ 



#define HAS_CO_SENSOR                               // Have you attached a CO sensor?

#define HAS_CO2_SENSOR                              // Have you attached a CO2 sensor?

#define MEASUREMENT_INTERVAL 90                     // How many seconds do you want between each measurement? For CO sensor, we need 90 sec interval!

#define HAS_DISPLAY                                 // Does the sensor have a little OLED display attached?

#define HAS_PASSIVE_BUZZER                          // Does the sensor have a passive buzzer?

#define ALLOW_CONNECTING_TO_NETWORK                 // Connect wirelessly. Is this device allowed to connect to the local Candle network? For privacy or security reasons you may prefer a stand-alone device.

//#define ALLOW_FAKE_DATA                             // Allow fake data? This feature is designed to make the sensor less intrusive in some social situations. If enabled, you can toggle this ability by pressing a button that you will need to connect as well. When fake data is being generated, a small F icon is visible on the display.

#define MY_REPEATER_FEATURE                       // Act as a repeater? The devices can pass along messages to each other to increase the range of your network.

#define RF_NANO                                     // RF-Nano. Check this box if you are using the RF-Nano Arduino, which has a built in radio. The Candle project uses the RF-Nano.

/* END OF SETTINGS
 *
 */

#define DEBUG
//#define MY_DEBUG // MySensors debugging. Enable MySensors debug output to the serial monitor, so you can check if the radio is working ok.

// Enable and select the attached radio type
#define MY_RADIO_RF24                               // This is a common and simple radio used with MySensors. Downside is that it uses the same frequency space as WiFi.
//#define MY_RADIO_NRF5_ESB                         // This is a new type of device that is arduino and radio all in one. Currently not suitable for beginners yet.
//#define MY_RADIO_RFM69                            // This is an open source radio on the 433mhz frequency. Great range and built-in encryption, but more expensive and little more difficult to connect.
//#define MY_RADIO_RFM95                            // This is a LoRaWan radio, which can have a range of 10km.

// MySensors: Choose your desired radio power level. High power can cause issues on cheap Chinese NRF24 radio's.
//#define MY_RF24_PA_LEVEL RF24_PA_MIN
//#define MY_RF24_PA_LEVEL RF24_PA_LOW
//#define MY_RF24_PA_LEVEL RF24_PA_HIGH
#define MY_RF24_PA_LEVEL RF24_PA_MAX

// Mysensors security
#define MY_ENCRYPTION_SIMPLE_PASSWD "A2Ed8vCV"      // Be aware, the length of the password has an effect on memory use.
//#define MY_SECURITY_SIMPLE_PASSWD "A2Ed8vCV"      // Be aware, the length of the password has an effect on memory use.
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN A7         // Setting a pin to pickup random electromagnetic noise helps make encryption more secure.

// Mysensors advanced settings
#define MY_TRANSPORT_WAIT_READY_MS 10000            // Try connecting for 10 seconds. Otherwise just continue.
//#define MY_RF24_CHANNEL 100                       // In EU the default channel 76 overlaps with wifi, so you could try using channel 100. But you will have to set this up on every device, and also on the controller.
#define MY_RF24_DATARATE RF24_1MBPS                 // Slower datarate makes the network more stable?
#define MY_SPLASH_SCREEN_DISABLED                   // Saves a little memory.
//#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE      // Saves a little memory.



// PINS
// Be aware, on the RF-Nano pins 9 through 13 are used by the radio.
#define analogMQ7CO 3                               // CO sensor analogue signal pin 
#define ledPinMQ7CO 2                               // Pin for LEds on CO sensor
#define CO2_RX_PIN 5                                // The RX (receive) pin for the CO2 sensor. This should be connected to the TX (transmit) pin on the sensor module.
#define CO2_TX_PIN 6                                // The TX (transmit) pin for the CO2 sensor. This should be connected to the RX (receive) pin on the sensor module.
#define BUZPIN 8

#define HORIZONTAL_START_POSITION 0                // Pixels from the left of the screen
#define DATA_TRANSMISSION_BUTTON_PIN 12
#define TOGGLE_FAKE_DATA_PIN 10


#ifdef RF_NANO
// If you are using an RF-Nano, you have to switch CE and CS pins.
#define MY_RF24_CS_PIN 9                            // Used by the MySensors library.
#define MY_RF24_CE_PIN 10                           // Used by the MySensors library.
#endif

// LIBRARIES (in the Arduino IDE go to Sketch -> Include Library -> Manage Libraries to add these if you don't have them installed yet.)
#include <MySensors.h>                              // MySensors library                  
#include <SoftwareSerial.h>                         // Serial data connection to the sensor

#ifdef HAS_DISPLAY
  #define OLED_I2C_ADDRESS 0x3C
  #include <SSD1306Ascii.h>                         // Simple drivers for the screen.
  #include <SSD1306AsciiAvrI2c.h>                   // "SSD1306Ascii".
  SSD1306AsciiAvrI2c oled;
  byte screen_vertical_position = 3;                // Used to always show both CO and CO2 levels at the top of the screen.
  #define F_POSITION 66                             // Horizontal position of the "F" icon, indicating it is allowed to generate fake data.
  #define T_POSITION 72                             // Horizontal position of the "T" icon, indicating it is allowed to transmit data.
  #define W_POSITION 80                             // Horizontal position of the "W" icon, indicating a wireless connection.
#endif


#ifdef HAS_CO_SENSOR
// CO sensor variables
int COValue = 0;
int Heatstatus = 1;
#endif


#ifdef HAS_CO2_SENSOR
// CO2 sensor variables
SoftwareSerial co2_sensor(CO2_TX_PIN, CO2_RX_PIN);  // Receive (RX) and transmit (TX) pins. RX should always connect to TX on the opposite device.
int co2_value = 0;
float average_co2_value = 400;                            
#endif


// Mysensors settings
#define CO_CHILD_ID  1                              // The child ID of the sensor that will be presented to the controller.
#define CO2_CHILD_ID 2                              // The child ID of the sensor that will be presented to the controller.
#define DATA_TRANSMISSION_CHILD_ID 3                // The child ID of the data transmission switch.
#define CO_OPINION_CHILD_ID 4                       // The child ID of the human readable opinion about the carbon monoxide level.
#define CO2_OPINION_CHILD_ID 5                      // The child ID of the human readable opinion about the carbon dioxide level.

const byte RADIO_DELAY = 100;                       // A few milliseconds delay between sending makes the radio happy.

MyMessage relaymsg(DATA_TRANSMISSION_CHILD_ID, V_STATUS); // To toggle data transmission on or off remotely.       

MyMessage prefix_message(CO_CHILD_ID, V_UNIT_PREFIX); // Tell the controller what to display along with the value.


#ifdef HAS_CO_SENSOR
MyMessage CO_message(CO_CHILD_ID, V_LEVEL);         // Sets up the message format that we'll be sending to the MySensors gateway later.
#endif

#ifdef HAS_CO2_SENSOR
MyMessage CO2_message(CO2_CHILD_ID, V_LEVEL);        // Sets up the message format that we'll be sending to the controller.
MyMessage info_message(CO2_OPINION_CHILD_ID,V_TEXT); // Sets up the message format that we'll be sending to the controller. The first part is the ID of the specific sensor module on this node. The second part describes what kind of data to expect.

#endif



#ifdef ALLOW_FAKE_DATA
// Fake data feature
#define AMOUNT_OF_MEASUREMENTS_TO_AVERAGE 5         // How many old measurements to remember. This is used to determine a good fake data range.
boolean sending_fake_data = false;                  // Experimental. Will allow a user to send fake data for a while. Useful in some social situations.
boolean desired_sending_fake_data = false;          // If the user wants to change the state of sending fake data.
float fake_co2_value = 0;                           // Holds the meandering fake value
float co2_minimum_found = 400;
float co2_maximum_found = 410;
float last_co2_minimum_found = 400;
float last_co2_maximum_found = 410;


float co2_fakeness_range = 0;                       // How far of the last average the value can meander.   
float fake_co2_jitter = 0;                          // Holds how much will actually be deviated from the average when generating a fake value.
float co2_fake_data_movement_factor = -0.5;         // The odds that a new fake value will be above or below the current fake value.
float past_measurements[AMOUNT_OF_MEASUREMENTS_TO_AVERAGE]; // An array that holds previous measurements. Used to generate fake data.
byte measurements_fakeness_range_counter = AMOUNT_OF_MEASUREMENTS_TO_AVERAGE; // Current position in the array that holds a few of the last real measurements.
#endif

// Connection toggle feature
boolean connected_to_network = false;
boolean transmission_state = true;
boolean previous_transmission_state = true;
// Other
#define LOOPDURATION 1000                           // The main loop runs every x milliseconds.  Normally this sensor has a 'heartbeat' of once every second.
boolean send_all_values = true;                     // If the controller asks the devive to re-present itself, then this is used to also resend all the current sensor values.

// Here I will start inserting my own stuff concerning CO2. Sorry if it is unclear shit.
#ifdef HAS_CO2_SENSOR
  int co2_value_PWM = 0;
  int co2_value_UART = 0;
  byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
  char response[9];
  #define pwmPin 7 // PWM pin
  int prevVal = LOW;
  long th, tl, h, l, ppm, CO2PWM = 0.0;
#endif

#ifdef HAS_CO_SENSOR
  int dangercount = 0;
#endif

void presentation()
{
#ifdef ALLOW_CONNECTING_TO_NETWORK
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(F("Carbon sensor"), F("1.0")); wait(RADIO_DELAY);
  
  // Register all sensors to gateway:
  present(DATA_TRANSMISSION_CHILD_ID, S_BINARY, F("Data transmission")); wait(RADIO_DELAY);
  
#ifdef HAS_CO_SENSOR
  present(CO_CHILD_ID, S_AIR_QUALITY, F("Carbon monoxide")); wait(RADIO_DELAY);
#endif

#ifdef HAS_CO2_SENSOR
  present(CO2_CHILD_ID, S_AIR_QUALITY, F("Carbon dioxide")); wait(RADIO_DELAY);
#ifdef HAS_CO_SENSOR  
  present(CO2_OPINION_CHILD_ID, S_INFO, F("Carbon dioxide opinion")); wait(RADIO_DELAY);  // The opinion about the CO2 level. If a CO sensor is also present, the name of this property helps distinguish the two opinions.
#else
  present(CO2_OPINION_CHILD_ID, S_INFO, F("Opinion")); wait(RADIO_DELAY); // The opinion about the CO2 level.
#endif
#endif

  send_all_values = true;
#endif
}


void setup() 
{
  Serial.begin(115200);
  wait(200);
  
  Serial.println(F("Hello, I am a carbon sensor"));
  transmission_state = loadState(DATA_TRANSMISSION_CHILD_ID);
  
  //This was in the original script, but uncommenting this will crash the Arduino?:
  //pinMode(CO2_RX_PIN, INPUT);
  //pinMode(CO2_TX_PIN, OUTPUT);

  co2_sensor.begin(9600);
  pinMode(pwmPin, INPUT);
  
  pinMode(DATA_TRANSMISSION_BUTTON_PIN, INPUT_PULLUP); // Attach a push button to this pin to toggle whether or not the device is allowed to transmit data to the controller.

#ifdef ALLOW_FAKE_DATA
  pinMode(TOGGLE_FAKE_DATA_PIN, INPUT_PULLUP);
  //Serial.print(F("Toggle fake-data-mode using a button on pin ")); Serial.println(TOGGLE_FAKE_DATA_PIN);
#endif


#ifdef HAS_DISPLAY
  // Initiate the display
  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
  oled.setScroll(false);
  oled.setCursor(HORIZONTAL_START_POSITION,0);
  oled.print(F("KOOLSTOF")); // ORIGINAL: CARBON
#endif


#ifdef ALLOW_CONNECTING_TO_NETWORK
  if( isTransportReady() ){                         // Check if a network connection has been established
    Serial.println(F("Connected to gateway!"));
    connected_to_network = true;
  }
  else {
     Serial.println(F("! NO CONNECTION")); 
  }
#endif 

#ifdef HAS_CO_SENSOR
  pinMode(ledPinMQ7CO,OUTPUT);
  fullheatCOreading();
  Heatstatus = 0;
#ifdef DEBUG
  Serial.print(F("Will test led in 3..."));
  wait(1000);
  Serial.print(F("2..."));
  wait(1000);
  Serial.println(F("1..."));
  wait(1000);
  Serial.println(F("BLINK"));
#endif
  digitalWrite(ledPinMQ7CO, HIGH); // test LED
  delay(1000);
  digitalWrite(ledPinMQ7CO, LOW);
  
#ifdef HAS_DISPLAY
  oled.setCursor(0,screen_vertical_position - 1);   // The labels are shown slightly above the values.
  oled.print(F("CO PPB:")); 
  screen_vertical_position = screen_vertical_position + 3;
#endif
#endif


#ifdef HAS_CO2_SENSOR
  wait(2000);                                       // Give the sensor some time to boot up
  co2_sensor.begin(9600);
  //wait(4000);                                     // Give the sensor some time to boot up
#ifdef HAS_DISPLAY
  oled.setCursor(0,screen_vertical_position - 1);   // The labels are shown slightly above the values.
  oled.print(F("CO2 PPM:")); 
#endif
#endif  

#if not defined(HAS_CO_SENSOR) && not defined(HAS_CO2_SENSOR)
  Serial.println(F("Please enable at least one sensor!"));
#ifdef HAS_DISPLAY
  oled.setCursor(0,3);
  oled.print(F("NO SENSORS ENABLED")); 
#endif
  while(1);
#endif

#ifdef HAS_PASSIVE_BUZZER
#ifdef DEBUG
  Serial.print(F("PREPARE FOR AUDIO TEST, in 3..."));
  wait(1000);
  Serial.print(F("2..."));
  wait(1000);
  Serial.println(F("1..."));
  wait(1000);
  Serial.println(F("BEEEEP"));
#endif
  beep();
  wait(1000);
  beep();
#endif

  wdt_enable(WDTO_8S);                              // Starts the watchdog timer. If it is not reset once every few seconds, then the entire device will automatically restart.
}


#ifdef ALLOW_CONNECTING_TO_NETWORK
void send_values()
{
  send(relaymsg.setSensor(DATA_TRANSMISSION_CHILD_ID).set(transmission_state)); wait(RADIO_DELAY);
#ifdef HAS_CO_SENSOR
  send(prefix_message.setSensor(CO_CHILD_ID).set( "ppb" )); delay(RADIO_DELAY); // Carbon values are always transmitted. They are not a large privacy risk, while being very important to safety.
  if( COValue != 0 ){
    send(CO_message.setSensor(CO_CHILD_ID).set(COValue),1);
  }
#endif
//  if(transmission_state){
#ifdef HAS_CO2_SENSOR
    send(prefix_message.setSensor(CO2_CHILD_ID).set( "ppm" )); delay(RADIO_DELAY);
    if(co2_value > 25){
      send(CO2_message.setSensor(CO2_CHILD_ID).set(co2_value_UART),1);
    }
#endif
//  }
}
#endif


void loop() 
{
#ifdef ALLOW_CONNECTING_TO_NETWORK
  if( send_all_values ){
#ifdef DEBUG
    Serial.println(F("RESENDING VALUES"));
#endif
    send_all_values = 0;
    send_values();
  }
#endif


#ifdef ALLOW_FAKE_DATA
  boolean fake_data_pin_state = digitalRead(TOGGLE_FAKE_DATA_PIN);
  if( fake_data_pin_state == 0 ){                   // If the button is being pressed
    desired_sending_fake_data = !desired_sending_fake_data; // Switch the setting to its opposive value.
    Serial.print(F("FAKE DATA TOGGLED TO ")); Serial.println(desired_sending_fake_data);
    wait(500); // Wait a while to allow the button to be released
  }
#endif


  boolean transmission_button_state = digitalRead(DATA_TRANSMISSION_BUTTON_PIN);
  if( transmission_button_state == 0 ){             // If the button is being pressed
    transmission_state = !transmission_state;       // Switch the setting to its opposive value.
    saveState(DATA_TRANSMISSION_CHILD_ID, transmission_state); // Store the new preference, so that is the device is rebooted, it will still be correct.
    Serial.println(F("Data transmission button pressed "));
    wait(500); // Wait a while to allow the button to be released
#ifdef ALLOW_CONNECTING_TO_NETWORK
      
      send(relaymsg.setSensor(DATA_TRANSMISSION_CHILD_ID).set(transmission_state));
#endif
  }

if ( transmission_state != previous_transmission_state ){
    previous_transmission_state = transmission_state;
    saveState(DATA_TRANSMISSION_CHILD_ID, transmission_state);
    Serial.print(F("Sending new data transmission state: ")); Serial.println(transmission_state);
    send(relaymsg.setSensor(DATA_TRANSMISSION_CHILD_ID).set(transmission_state));
}

  
  //
  // HEARTBEAT LOOP
  // runs every second (or as long as you want). By counting how often this loop has run (and resetting that counter back to zero after a number of loops), it becomes possible to schedule all kinds of things without using a lot of memory.
  // The maximum time that can be scheduled is 255 * the time that one loop takes. So usually 255 seconds.
  //

  static unsigned long lastLoopTime = 0;            // Holds the last time the main loop ran.
  static int loopCounter = 0;                       // Count how many loops have passed (reset to 0 after at most 254 loops).
  unsigned long currentMillis = millis();

  if( currentMillis - lastLoopTime > LOOPDURATION ){
    lastLoopTime = currentMillis;
    loopCounter++;
#ifdef DEBUG
    Serial.print("loopcounter:"); Serial.println(loopCounter);
#endif
    if(loopCounter >= MEASUREMENT_INTERVAL){
      Serial.println(); Serial.println(F("__starting__"));  
      loopCounter = 0;
    }

    wdt_reset();                                    // Reset the watchdog timer

  /*
  // Used during development
  if(measurements_counter == 10 && desired_sending_fake_data == false){
    Serial.println(); Serial.println(F("INITIATING FAKENESS----------------------------------------"));
    desired_sending_fake_data = true;
  }
  */
  
#ifdef HAS_DISPLAY
    // Show second counter
    oled.set1X();
    oled.setCursor(100,0);
    oled.print(MEASUREMENT_INTERVAL - loopCounter);
    oled.clearToEOL();

    screen_vertical_position = 3;                   // If there is one sensor attached, then new data should be shown at line 3 of the screen. If there are two, then data is shown on line 3 and line 6.

#endif
      

    // schedule
    if( loopCounter == 1 ){
        
        // CARBON MONIXODE
#ifdef HAS_CO_SENSOR
        //find out if last cycle was heating or measuring.
        
         if (Heatstatus==0) {
          //if low heat --> measure and heat.
          //First measurement is UNRELIABLE!
          COValue = readCOValue();
          Heatstatus = 1;
         } else{
          //if high heating --> go to low heat
          medheatCOreading();
          Heatstatus = 0;
         }

        if (COValue > 3000){
          //Dangerous CO levels, if I measure this twice (?) activate emergency powers
          dangercount++;
          if (dangercount > 1){
          activate_emergency();
          }
        } else {
          dangercount = 0;
        }

#ifdef HAS_DISPLAY
        // Show CO level on the screen
        //oled.set1X();
        oled.setCursor(HORIZONTAL_START_POSITION,screen_vertical_position);
        
        if (COValue == -1){                         // -1 value means sensor probably not connected
          oled.print(F("CHECK WIRE"));
          oled.clearToEOL();                       
          //break;
        }
        else if (COValue == -2){
          oled.print(F("DATA ERROR"));              // -2 value means we got data form the sensor module was was not a CO2 level. For example, a response to some other command.
          oled.clearToEOL();
        }
        else{
          // Display CO value.
          oled.print(COValue);
          oled.clearToEOL();
  
          // Show quality opinion the screen. MAX 5 LETTERS.
          oled.setCursor(60,screen_vertical_position);
          if (COValue > 0 && COValue < 450){       oled.print(F("TOP"));}
          else if (COValue < 700){  oled.print(F("GOED"));oled.clearToEOL();}
          else if (COValue < 1000){ oled.print(F("OK"));  oled.clearToEOL();}
          else if (COValue < 2000){ oled.print(F("MATIG"));oled.clearToEOL();}
          else if (COValue < 4500){ oled.print(F("NAAR")); oled.clearToEOL();}
          else {
            oled.print(F("Wait.."));
            oled.clearToEOL();
          }
        }
        screen_vertical_position = screen_vertical_position + 3; // If a CO sensor is attached, it's value will be displayed on top. The Co2 value will then be shown 3 lines below it.
#endif
#endif

        // CARBON DIOXIDE
#ifdef HAS_CO2_SENSOR
        //int new_co2_value = readco2_value();      // Get carbon dioxide level from sensor module - old way of doing this
        co2_value_UART = readCO2UART();
        Serial.print(F("fresh UART co2 value: ")); Serial.println(co2_value_UART);
        #ifdef DEBUG //During normal use, PWM value is unnessecarily complicated.
          co2_value_PWM = getCO2PWM();               // Get carbon dioxide level from sensor module - PWM
          Serial.print(F("fresh PWM co2 value: ")); Serial.println(co2_value_PWM);
        #endif
        
#ifdef DEBUG
        if( co2_value_PWM == -1 || co2_value_PWM == -2 ){
          Serial.println(F("PWM SENSOR ERROR")); // Used during development to test even though no actual sensor is attached.
        //  co2_value = random(500,600);
        }
        if( co2_value_UART == -1 || co2_value_UART == -2 ){
          Serial.println(F("PWM SENSOR ERROR")); // Used during development to test even though no actual sensor is attached.
        //  co2_value = random(500,600);
        }
#endif

#ifdef ALLOW_FAKE_DATA
        if( co2_value_UART > 350 && sending_fake_data == false){ // While fake data is not being created, we analyse the real data to look for the range it displays.
          measurements_fakeness_range_counter++;
          Serial.print(F("measurements_fakeness_range_counter = ")); Serial.println(measurements_fakeness_range_counter);
          if( measurements_fakeness_range_counter >= AMOUNT_OF_MEASUREMENTS_TO_AVERAGE){
            Serial.print(F("Restarting min-max analysis around co2 value of ")); Serial.println(co2_value_UART);
            measurements_fakeness_range_counter = 0;
            if( co2_maximum_found - co2_minimum_found != 0 && co2_maximum_found - co2_minimum_found < 30 ){
              co2_fakeness_range = co2_maximum_found - co2_minimum_found; // What is the difference between the highest and lowest co2 value we spotted recently.
              last_co2_minimum_found = co2_minimum_found;
              last_co2_maximum_found = co2_maximum_found;
            }
            co2_minimum_found = co2_value_UART;
            co2_maximum_found = co2_value_UART;
          }
          
          if(co2_value_UART < co2_minimum_found){
            co2_minimum_found = co2_value_UART;
            Serial.println(F("new co2 value was smaller than minimum found."));
          }
          else if(co2_value_UART > co2_maximum_found){
            co2_maximum_found = co2_value_UART;
            Serial.println(F("new co2 value was bigger than maximum found."));
          }
          else{
            Serial.println(F("new co2 values was not bigger or smaller than recent measurements."));
          }
          Serial.print(F("potential min-max range: ")); Serial.println(co2_maximum_found - co2_minimum_found);
          Serial.print(F("actual min-max range: ")); Serial.println(co2_fakeness_range);
        }


        if( desired_sending_fake_data == true ){
          Serial.println(F("User wants to generate fake data."));
          if( sending_fake_data == false ){         // On the first run of fake data, we try and figure out what the range to fake in is.
            Serial.println(F("initiating fake data"));
            sending_fake_data = true;
            if(co2_fakeness_range == 0){
              co2_fakeness_range = 1;               // The minimum to actually make some fake jitter
            }
            co2_fake_data_movement_factor = -0.5;
            fake_co2_value = co2_value_UART;             // The initial fake value;
          }
        }
        else{
          // The user no longer wants to generate fake data.
          if( sending_fake_data == true ){ // If we were generating fake data, we should slowly move the fake value towards the real value. Only then can we stop generating the fake value.
            last_co2_minimum_found = co2_value_UART - co2_fakeness_range;
            last_co2_maximum_found = co2_value_UART + co2_fakeness_range;
            
            if(co2_value_UART > fake_co2_value){
              co2_fake_data_movement_factor = -0.1;  // By modifiying this factor to favour one direction, the fake data will move towards the real co2 value.
              Serial.println(F("stopping faking, movement factor set to 0,9: "));
            }
            else if(co2_value_UART < fake_co2_value){
              Serial.println(F("stopping faking, movement factor set to -0,9: "));
              co2_fake_data_movement_factor = -0.9;
            }
            if( abs(fake_co2_value - co2_value_UART) < co2_fakeness_range ){ // When the fake value is very close to the real value, the real value can take over again.
              Serial.println(F("Faking has ended"));
              sending_fake_data = false;
            }
          }
        }

        if( sending_fake_data == true ){
          // We are now sending fake data.
          fake_co2_jitter = (float)random( (co2_fakeness_range) * 10000) / 10000;
          Serial.print(F("fake CO2 addition: ")); Serial.println(fake_co2_jitter);

          float flipped_coin = random(2);           // This will be 0 or 1.
          Serial.print(F("flipped coin: ")); Serial.println(flipped_coin);
          Serial.print(F("co2_fake_data_movement_factor: ")); Serial.println(co2_fake_data_movement_factor);
          float factor = flipped_coin + co2_fake_data_movement_factor;
          Serial.print(F("actual movement factor: ")); Serial.println(factor);
          fake_co2_jitter = fake_co2_jitter * factor; // The addition is now multiplied by -0,5 or +0,5.
          Serial.print(F("fake CO2 jitter after movement factor: ")); Serial.println(fake_co2_jitter);

          Serial.print(F("last_min: ")); Serial.println(last_co2_minimum_found);
          Serial.print(F("last_max: ")); Serial.println(last_co2_maximum_found);
          if( fake_co2_jitter > 0 && fake_co2_value + fake_co2_jitter > last_co2_maximum_found){ // If the new addition (which can be negative) moves the fake data value ourside of the allowed range, then adjust it.
            fake_co2_jitter = -fake_co2_jitter;
            Serial.println("A");
          }
          else if( fake_co2_jitter < 0 && fake_co2_value + fake_co2_jitter < last_co2_minimum_found){ // If the new addition (which can be negative) moves the fake data value ourside of the allowed range, then adjust it.
            fake_co2_jitter = -fake_co2_jitter;
            Serial.println("B");
          }
          else{
            Serial.println("CC");
          }
          Serial.print(F("fake CO2 addition after bounds check: ")); Serial.println(fake_co2_jitter);

          fake_co2_value = fake_co2_value + fake_co2_jitter;
          co2_value_UART = int(fake_co2_value);
          /*
          // Create meandering data effect
          if( flipped_coin == 0 && fake_co2_value + fake_co2_jitter > average_co2_value + co2_fakeness_range ){ // Check if there is head room to make the fake data value change in the random direction.
            co2_value = fake_co2_value - fake_co2_jitter; // There is no room to go up, the fake value should go down.
          }
          else if( flipped_coin == 1 && fake_co2_value - fake_co2_jitter < average_co2_value - co2_fakeness_range ){
            co2_value = fake_co2_value + fake_co2_jitter; // There is no room to go down, the fake value should go up.
          }
          else{
            if( flipped_coin ){ fake_co2_jitter = -fake_co2_jitter; } // If we have not reached the maximum high or low fake data value, then randomly add or subtract the addition.
            fake_co2_value = fake_co2_value + fake_co2_jitter;
          }
          */
          Serial.print(F("  {}{}{} Fake CO2 value: ")); Serial.println(co2_value_UART);

        }
#endif // End of allow fake data



#ifdef HAS_DISPLAY
        // Show CO2 level on the screen 
        oled.set2X();
        oled.setCursor(HORIZONTAL_START_POSITION,screen_vertical_position);
        
        if (co2_value_UART == -1){                       // -1 value means sensor probably not connected
          oled.print(F("CHECK WIRE"));
          oled.clearToEOL();
        }
        else if (co2_value_UART == -2){
          oled.print(F("DATA ERROR"));              // -2 value means we got data form the sensor module was was not a CO2 level. For example, a response to some other command.
          oled.clearToEOL();
        }
        else if( co2_value_UART > 350 &&  co2_value_UART < 5001){
          // Display CO2 value.
          oled.print(co2_value_UART);
          oled.clearToEOL();
  
          // Show quality opinion the screen. Max 5 symbols.
          oled.setCursor(60,screen_vertical_position);

          if (co2_value_UART < 500){       oled.print(F("TOP"));}
          else if (co2_value_UART < 700){  oled.print(F("GOED"));}
          else if (co2_value_UART < 1000){ oled.print(F("OK"));}
          else if (co2_value_UART < 2000){ oled.print(F("MATIG"));}
          else if (co2_value_UART < 5001){ oled.print(F("NAAR"));}
          else {
            oled.print(F("Wait.."));
          }
          oled.clearToEOL();
        }
#ifdef DEBUG
        else{
          Serial.println(F("CO2 value was out of bounds"));    
        }
#endif
        
        
#endif // End of has_display
#endif // End of hasCO2senor
    }

    
    else if( loopCounter == 2 ){                    // Send the data                                       

#ifdef HAS_CO_SENSOR
      if( COValue > 0 && COValue < 4500 ){          // Avoid sending erroneous values
#ifdef ALLOW_CONNECTING_TO_NETWORK
          connected_to_network = false;
          Serial.println(F("Sending CO to controller"));
          send(CO_message.setSensor(CO_CHILD_ID).set(COValue),1); // We ask the controller to acknowledge that it has received the data.  
#endif // end of allow connecting to network
      }
#endif // end of has CO sensor


#ifdef HAS_CO2_SENSOR
      if( co2_value_UART > 0 && co2_value_UART < 4500 ){    // Avoid sending erroneous values
#ifdef ALLOW_CONNECTING_TO_NETWORK
        if( transmission_state ){
          connected_to_network = false;             // If the network connection is ok, then this will be immediately set back to true.
          Serial.print(F("Sending CO2 value: ")); Serial.println(co2_value_UART); 
          send(CO2_message.setSensor(CO2_CHILD_ID).set(co2_value_UART),1); // We send the data, and ask the controller to acknowledge that it has received the data.
          wait(RADIO_DELAY);

          // Also send the human readable opinion
          if (co2_value_UART < 450){       send(info_message.setSensor(CO2_OPINION_CHILD_ID).set( F("Erg Goed") )); }
          else if (co2_value_UART < 700){  send(info_message.setSensor(CO2_OPINION_CHILD_ID).set( F("Goed") )); }
          else if (co2_value_UART < 1000){ send(info_message.setSensor(CO2_OPINION_CHILD_ID).set( F("Prima") )); }
          else if (co2_value_UART < 2000){ send(info_message.setSensor(CO2_OPINION_CHILD_ID).set( F("Matig") )); }
          else if (co2_value_UART < 5000){ send(info_message.setSensor(CO2_OPINION_CHILD_ID).set( F("Slecht") )); }
          
        }
        else{
          Serial.println(F("Not allowed to send the CO2 data"));
        }
#endif // end of allow connecting to network
      }
#endif // end of has CO2 sensor
    }


#ifdef HAS_DISPLAY
    else if( loopCounter == 3 ){                    // Show the various states on the display                                     
      oled.set1X();
      oled.setCursor(W_POSITION,0);
      if( connected_to_network ){                   // Add W icon to the top right corner of the screen, indicating a wireless connection.
        oled.print(F("W"));
      }else {
        oled.print(F("."));                         // Remove W icon
      }
    }


    // The following two are updated every second.
    oled.set1X();

 #ifdef ALLOW_FAKE_DATA
    oled.setCursor(F_POSITION,0);
    if( desired_sending_fake_data && sending_fake_data){ // We are sending fake data
      oled.print(F("F"));
    }
    else if(desired_sending_fake_data != sending_fake_data){ // In the transition between real and fake data
      oled.print(F("f"));
    }
    else{                                           // No fake data is being generated
      oled.print(F("."));
    }
#endif

#ifdef ALLOW_CONNECTING_TO_NETWORK
    oled.setCursor(T_POSITION,0);
    if( transmission_state ){
      oled.print(F("T"));
    }
    else{
      oled.print(F("."));
    }
#endif

#endif // end of has display
  }
}

#ifdef HAS_CO2_SENSOR
long getCO2PWM(){ // Function to get CO2 PWM value
  int j; 
  j = 0;
  do {
    #ifdef DEBUG
      Serial.print("Listening on CO2 PWM...  ");
    #endif
    th = pulseIn(pwmPin, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    CO2PWM = 5000 * (th-2)/(th+tl-4);
    j++;
    #ifdef DEBUG
      Serial.println("Response was: ");
      Serial.print(th);
      Serial.print(". Corresponding ppm value: ");
      Serial.print(CO2PWM);
    #endif
    delay(1000); // be nice to the sensor.
    //Serial.println("Start next try in 1s");
    } while ( (CO2PWM < 0.0) && (j<3) ); // Beware CO2PWM<0 can be max out or error.
    if(th == 0){
      Serial.println("Error in measurement, better luck next time...");
    };
  return CO2PWM;
}

float readCO2UART(){
  byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
  byte response[9]; // for answer
  #ifdef DEBUG
    Serial.println("Sending CO2 UART request...");
  #endif
  co2_sensor.write(cmd, 9); //request PPM CO2
  #ifdef DEBUG
    Serial.println("Request send.");
  #endif
  // clear the buffer
  memset(response, 0, 9);
  int i = 0;
  #ifdef DEBUG
    Serial.println("Into the loop.");
  #endif
  while ( (co2_sensor.available() == 0) && (i<5) ) {
    Serial.print("Waiting for response ");
    Serial.print(i);
    Serial.println(" s");
    delay(1000);
    i++;
  }
  #ifdef DEBUG
    Serial.println("Out of the loop");
  #endif
  if (co2_sensor.available() > 0) {
      co2_sensor.readBytes(response, 9);
  } else{
    Serial.println("Some error occured during reading, just wait for next loop");
    return -1.0;
  }
  
  // print out the response in hexa
  #ifdef DEBUG
    for (int i = 0; i < 9; i++) {
      Serial.print(String(response[i], HEX));
      Serial.print("   ");
    }
    Serial.println("");
  #endif
  
  // checksum
  byte check = getCheckSum(response);
  if (response[8] != check) {
    Serial.println("Checksum not OK!");
    Serial.print("Received: ");
    Serial.println(response[8]);
    Serial.print("Should be: ");
    Serial.println(check);
  }
 
  // ppm
  int ppm_uart = 256 * (int)response[2] + response[3];
  #ifdef DEBUG
    Serial.print("PPM UART: ");
    Serial.println(ppm_uart);
  #endif
  
  // temp
  byte temp = response[4] - 40;
  #ifdef DEBUG
    Serial.print("Temperature? ");
    Serial.println(temp);
  #endif
  
  // status
  byte status = response[5];
  #ifdef DEBUG
    Serial.print("Status? ");
    Serial.println(status);
    if (status == 0x40) {
      Serial.println("Status OK");
    }
  #endif
  return ppm_uart;
}
#endif

#ifdef HAS_CO_SENSOR
// see https://www.hackster.io/ingo-lohs/gas-sensor-carbon-monoxide-mq-7-aka-flying-fish-e58457, https://www.instructables.com/id/Arduino-CO-Monitor-Using-MQ-7-Sensor/
//BAsically: START --> fullheatCOreading--> wait 60 sec --> medheatCOreading --> wait 90 sec --> readCOValue --> wait 60 seconds (or more) --> medheatCOreading --> wait 90 sec --> readCOValue --> etc.
void fullheatCOreading(){
  // turn the heater fully on, burn of CO on plate. HEAT FOR AT LEAST 1 minute. 
  Serial.println("Turn the CO heater to full");
  analogWrite(analogMQ7CO, HIGH);
}

void medheatCOreading(){
  // turn the heater lower, let CO accumulate on plate and measure. TIMING IS IMPORTANT! Heat for 90 seconds!!
    Serial.println("Turn the CO heater to medium");
  analogWrite(analogMQ7CO, 71.4);
}

int readCOValue(){
  // CO2 via MQ7: we need to read the sensor at 5V, but must not let it heat up (CO will detatch). So hurry!
  Serial.println("Turn the CO heater to high and measure");
  analogWrite(analogMQ7CO, HIGH); 
  delay(50); // Getting an analog read apparently takes 100uSec
  COValue = analogRead(analogMQ7CO);     
  Serial.println("CO value measured:");
  Serial.println(COValue);
  return COValue;
}

void activate_emergency(){
  // In case of high CO levels, make sure to sound the alarms
  while (true){
    // give auditory warning
    beep();
    // give a heartbeat so arduino stays alive
    wdt_reset();
    // LED ON
    digitalWrite(ledPinMQ7CO, HIGH);     
    delay(1000);
    //LED OFF
    digitalWrite(ledPinMQ7CO, LOW);
    delay(1000);
  }
}

#endif

#ifdef HAS_PASSIVE_BUZZER
void beep(){
  tone(BUZPIN, 4978, 250);
  wait(300);
  noTone(BUZPIN);
  tone(BUZPIN,41,250);
  wait(300);
  noTone(BUZPIN);
  digitalWrite(BUZPIN, HIGH);
}
#endif

#ifdef ALLOW_CONNECTING_TO_NETWORK
void receive(const MyMessage &message)
{
  Serial.println(F(">> receiving message"));
  connected_to_network = true;
  
  if( message.isAck() ){
    Serial.println(F("-Got echo"));
    return;
  }

  if (message.type == V_STATUS && message.sensor == DATA_TRANSMISSION_CHILD_ID ){
    transmission_state = message.getBool(); //?RELAY_ON:RELAY_OFF;
    Serial.print(F("-New desired transmission state: ")); Serial.println(transmission_state);
  }  
}
#endif

// A helper function to check the integrity of a received sensor message.
byte getCheckSum(byte* packet)
{
  byte i;
  unsigned char checksum = 0;
  for (i = 1; i < 8; i++) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}



/* 
 *  
 * This code makes use of the MySensors library:
 * 
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013b-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 */
