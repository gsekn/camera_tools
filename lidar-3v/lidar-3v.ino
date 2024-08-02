// guy eakin 2024-07-21
// adapted from https://www.diyengineers.com/2022/06/02/lidar-how-to-use-with-arduino/
// display adapted from https://github.com/adafruit/Adafruit_SSD1306/blob/master/examples/ssd1306_128x32_i2c/ssd1306_128x32_i2c.ino
//
// Lidar device that allows user to define a range between two points.  If the LIDAR detects
// an object between these two points, a 3.3v pulse is delivered as output through a 2.5 mm stereo jack
//  currently set up to run on an Arduino micro
    #include <Wire.h>        // Instantiate the Wire library
    #include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1
    #include <Adafruit_GFX.h>  // for display
    #include <Adafruit_SSD1306.h> // for display

//display
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 32 // OLED display height, in pixels
    // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
    #define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//lidar functions
    TFLI2C tflI2C;
    int16_t  tfDist;    // distance in centimeters
    int16_t  tfAddr = TFL_DEF_ADR;  // Use this default I2C address
    int16_t  tfFlux;  //flux or strength of lidar signal
    int16_t  tfTemp;    // temperature in Celsius

//button functions
    const int buttonPin_Near = 5;   // the number of the pushbutton pin
    const int buttonPin_Far = 7;   // the number of the pushbutton pin
    const int ledPin = 13;    // the number of the LED pin
    int buttonState_Near = 0;
    int buttonState_Far = 0;
    int triggerDistNear = 50;  //set a threshold distance in 850 cm 
    int triggerDistFar = 100;  //set a threshold distance in 900 cm 
    char message = "";  //general alert message

//output function - delivers 5v to pin 3
    int armingPin = A0; // if pin is low, then this will prevent triggering function
    int isArmed = 0;
    int outputPin = 6;   //consider changing to 5 so project can be used on Arduino Micro as micro reserves pin  for SCL

// radio functions  D0-D4 correspond to receiver pin ID
  int signalPinD0 = 8;       
  int signalPinD1 = 9;
  int signalPinD2 = 10;
  int signalPinD3 = 11;
  int isRadio = 0; // variable to flag to display if a radio signal is being received
  
// targeting laser output pin
  int laserPin = 12;


// I2C Pins are SCL/SDA on Arduino Uno, but SCL=3 and SDA=2 on Micro


void setup(){
    Serial.begin(115200);  // Initalize serial port
    Wire.begin();           // Initalize Wire library
    // initialize the LED pin as an output:
    pinMode(ledPin, OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(buttonPin_Near, INPUT);
    pinMode(buttonPin_Far, INPUT);

    // set armed Pin as input
      pinMode(armingPin, INPUT_PULLUP);
      // initialize output pin to deliver 5v
      pinMode(outputPin, OUTPUT);

    //display
      // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
      if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
      }
      // Show initial display buffer contents on the screen --
      // the library initializes this with an Adafruit splash screen.
      display.display();
      delay(250); // Pause 

      // Clear the buffer
      display.clearDisplay();

      // Draw a single pixel in white
      display.drawPixel(10, 10, SSD1306_WHITE);

      // Show the display buffer on the screen. You MUST call display() after
      // drawing commands to make them visible on screen!
      display.display();
      delay(250);

      // radio setup
      pinMode(signalPinD0, INPUT);
      pinMode(signalPinD1, INPUT);
      pinMode(signalPinD2, INPUT);
      pinMode(signalPinD3, INPUT);
      //pinMode(outputPinD3_A, OUTPUT);      

      //targetting laser output
      pinMode(laserPin, OUTPUT);
}

 
void loop(){
    char *message = "";
    // get LIDAR button state
    buttonState_Near = digitalRead(buttonPin_Near);
    buttonState_Far = digitalRead(buttonPin_Far);

    // this is counterintuitive but input_pullup is declared on armingPin to reduce noise
    // so the low state is actually the armed state.
    if (digitalRead(armingPin) == 1){
      isArmed = 0;
    }
    else {isArmed = 1;}
    //Serial.println(isArmed);

    // get radio button state
    int PinD0State_D = digitalRead(signalPinD0); // unassigned function to this radio button
    int PinD1State_C = digitalRead(signalPinD1);  // sets far distance for lidar
    int PinD2State_B = digitalRead(signalPinD2);  // sets near distance for lidar
    int PinD3State_A = digitalRead(signalPinD3);  // triggers camera shutter
    isRadio = 0; // variable to flag to display if a radio signal is being received

    //if(tflI2C.getData(tfDist, tfAddr)){   
    if(tflI2C.getData(tfDist, tfFlux, tfTemp, tfAddr)){ 
        // trigger output in the case of radio signal "A" (PinD3State_A = High)
        if (PinD3State_A == 1){
            char *message = "Radio trigger";
            isRadio = 1;
            displayConstructor(tfDist, triggerDistNear, triggerDistFar, true, message, isArmed, isRadio);
            triggerOutput(200);
        }
        // turn on laser in case of Radio Signal D
        if (PinD0State_D == 1){
            char *message = "Laser on";
            isRadio = 1;
            displayConstructor(tfDist, triggerDistNear, triggerDistFar, true, message, isArmed, isRadio);
            digitalWrite(laserPin, HIGH);
            delay(50);
            digitalWrite(laserPin, LOW);
        }

        // retrieve LIDAR button data
        // check if the pushbutton is pressed. If it is, the buttonState_Near is HIGH:
        // as long as signal strength is greater than a value
        if ((((buttonState_Near == HIGH) && (tfFlux >= 200)) || ((PinD2State_B == HIGH) && (tfFlux >= 200)) )) {
          // turn LED on:
          digitalWrite(ledPin, HIGH);
          triggerDistNear = tfDist;
          if (PinD2State_B == HIGH){
            char *message = "Radio near";
            isRadio = 1; 
            }
          else {char *message = "set near ";} 
          displayConstructor(tfDist, triggerDistNear, triggerDistFar, false, message, isArmed, isRadio);
          delay(50);
        } 
        else {
          // turn LED off:
          digitalWrite(ledPin, LOW);
          char *message = "";
          displayConstructor(tfDist, triggerDistNear, triggerDistFar, false, message, isArmed, isRadio);
        }

        // check if the pushbutton is pressed. If it is, the buttonState_Far is HIGH:
        // as long as signal strength is greater than a value PinD1State_C
        if ((( ( (buttonState_Far == HIGH) && (tfFlux >= 200)) || (PinD1State_C == HIGH) && (tfFlux >= 200)) )) {
          // turn LED on:
          digitalWrite(ledPin, HIGH);
          triggerDistFar = tfDist;
          if (PinD1State_C == HIGH){
            char *message = "Radio far";
            isRadio = 1; 
            }
          else {
            char *message = "set far  ";
          }
          displayConstructor(tfDist, triggerDistNear, triggerDistFar, false, message, isArmed, isRadio);
        } 
        else {
          // turn LED off:
          digitalWrite(ledPin, LOW);
          char *message = "";
          displayConstructor(tfDist, triggerDistNear, triggerDistFar, false, message, isArmed, isRadio);
        }

        // if far is closer than near, swap values
        if (triggerDistFar < triggerDistNear) {
          char *message = "swap near/far";
          displayConstructor(tfDist, triggerDistNear, triggerDistFar, false, message, isArmed, isRadio);
          int var = triggerDistFar;
          triggerDistFar = triggerDistNear;
          triggerDistNear = var;
        }


        // check if object is between trigger distances
        if (((tfDist >= triggerDistNear)) && (tfDist <= (triggerDistFar))){
            //Serial.println("trigger distance acheived");
            // push relevant data to serial monitor/plotter
            char *message = "in range ";
            displayConstructor(tfDist, triggerDistNear, triggerDistFar, true, message, isArmed, isRadio);

            // take output pin High (trigger output)
            if (isArmed){
              
              char *message = "triggered";
              displayConstructor(tfDist, triggerDistNear, triggerDistFar, true, message, isArmed, isRadio);
              triggerOutput(200);

              //flash LED excitedly
              flashExcitedly(20,6);
            }

        }          
    }
    delay(10);

}

void triggerOutput(int outputDuration){
    digitalWrite(outputPin, HIGH);
    delay(outputDuration);
    digitalWrite(outputPin, LOW);
}

void displayConstructor(int tfDist, int triggerDistNear, int triggerDistFar, bool isTriggered, char *message, int isArmed, int isRadio){
  myserialplotter(tfDist, triggerDistNear, triggerDistFar, true, message);

  int point = vizData(tfDist, triggerDistNear,triggerDistFar);
  Serial.print("Armed: ");Serial.println(isArmed);
  
  oledDisplay(tfDist,triggerDistNear, triggerDistFar, isTriggered, message, point, isArmed, isRadio);
}

void myserialplotter(int tfDist, int triggerDistNear, int triggerDistFar, bool isTriggered, char *message ){
    //Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches / trigger: " + String(triggerDistNear) + "/" + String(triggerDistFar) + " cm");
    
    //when triggered condition is true, provide a indicator in the monitor
    //using a variable set to the average of the near and far distances 
    int var = 0;
    if (isTriggered){
      var = 0.5 * (triggerDistNear + triggerDistFar);
    }
    Serial.print("Triggered:");Serial.print(var);
    Serial.print(",");    
    Serial.print("Dist:");Serial.print(tfDist);
    Serial.print(",");
    Serial.print("Near:");Serial.print(triggerDistNear);
    Serial.print(",");
    Serial.print("Far:");Serial.print(triggerDistFar);
    Serial.print(",");
    Serial.print("Msg:");Serial.print(message);
    Serial.print(",");
}



char vizData(int tfDist, int triggerDistNear, int triggerDistFar){
    // represent lidar distances as ascii image where 
    //  Near "=-==|==========|==-=" Far
    //          |-- zone --|
    //      |-- zone +/- 50% --|
    //  this function returns the index number of where a marker should be placed in this string
    


    int lengthZone = triggerDistFar - triggerDistNear;
    int far = triggerDistFar + (.5 * lengthZone);
    int near = triggerDistNear - (.5 * lengthZone);
    
    int point = 0;
    if (tfDist > far){point = 19;}
    else if (tfDist < near){point = 0;}
    else if ((tfDist <= far) && (tfDist >= near)){
      float unitlength = (far-near)/16; //calculate length in cm of each of the 16 ascii chars in the "zone + 50%"
      point = float(round( (tfDist-near)/(unitlength)) + 1);  // place marker at representaive distance in zone + 50%
    }
    
    Serial.print("vizA:"); Serial.print(point);Serial.print("");   //this works
    return point;
}  

void oledDisplay(int tfDist, int triggerDistNear, int triggerDistFar, bool isTriggered, char *message, int point, int Isarmed, int isRadio ){
    
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    
    // display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text

    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);
    if (isArmed != 0){display.write(0x2A);} // draw an asterix in top left if system is armed 
    else {display.write(0x2D);} // draw an hyphen for unarmed
    display.setCursor(8, 0);     
    display.print(message); 
    display.setCursor(90, 0);
    display.print(tfDist); display.print(" cm");

    display.setCursor(0, 11);
    char vizArray[21] = "=-==|==========|==-="; //20 chars + null
    vizArray[point] = byte(42);  // replace a point in the visualization string with an * to show target position
    display.println(vizArray);
    
    display.setCursor(20, 22);  
    display.print(triggerDistNear);
    display.setCursor(80, 22); 
    display.print(triggerDistFar); 

    display.drawRect(25,10,68,9,SSD1306_WHITE );

    if (isRadio == 1){
      // draw a little radio if a radio signal is received
      display.drawRect(107,23,6,8,SSD1306_WHITE );
      display.drawRect(111,18,2,5, SSD1306_WHITE);
      display.drawPixel(114,25, SSD1306_WHITE);
      display.drawLine(108,24,110,24, SSD1306_WHITE);
    }

    if (message == "in range " || message == "triggered" || message =="Radio trigger"){
    // why not draw a camera when the trigger is fired.
    display.drawRect(50,24,18,8,SSD1306_WHITE ); //camera body
    display.drawCircle(58,28,3,SSD1306_WHITE ); // lens
    display.drawRect(58,22,4,2,SSD1306_WHITE ); // flash
    display.drawRect(63,25,4,2,SSD1306_WHITE ); //viewfinder
    }

    if (message == "triggered"){
    //flash 
    display.drawLine(56,22,53,21, SSD1306_WHITE);
    display.drawLine(63,22,66,21, SSD1306_WHITE);
    }
    display.display();   
}   


void flashExcitedly(int delayTime, int flashTimes){
    int var = 0;
    while (var < flashTimes){    
        digitalWrite(ledPin, HIGH);
        delay(delayTime);
        digitalWrite(ledPin, LOW);
        var++;
    }
}