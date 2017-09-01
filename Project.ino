/*
 * Author: George Garyfallou
 * Arduino gets data from bend sensor and sends to Base Station.
 */
#include <JeeLib.h> // Low power functions library
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include <avr/power.h>


// XBee's DOUT (TX) is connected to pin 0 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 1 (Arduino's Software TX)
SoftwareSerial XBee(0, 1); // RX, TX

#define ledPin 13
#define MAX     5

int wakePin = 2;              // Digital pin No2 used for interrupt(wake up the arduino when the charging starts)
int chargedPin = 3;           // Digital pin No3 used for interrupt(inform the arduino when the charging stops)
unsigned int PP;              // Power Priority
unsigned int MP = 0;          // Message Priority[1,5], 0->Don't Care
float volt;                   // voltage applied to microcontroller
float battery;                // Percent of energy left
int sampling_frequency;       //
boolean charging_mode = 0;    // charging_mode:   1 while the node is charging, 0 when charging stops(full battery)
boolean standalone_mode = 0;  // standalone_mode: 1 while the node depends on it's battery reservs , 0 when charging starts(full battery)




/***************************************************
 *  Name:        readVcc
 *
 *  Returns:     Vcc in millivolts.
 *
 *  Parameters:  None.
 *
 *  Description: Measures the battery voltage applied
 *               to the arduino.
 *
 ***************************************************/
long readVcc()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


/***************************************************
 *  Name:        wakeUpInterrupt
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Service routine for pin2 interrupt
 *               that wakes up the arduino.
 *
 ***************************************************/
void wakeUpInterrupt(void)
{
  /* This will bring us back from sleep. */
}



/***************************************************
 *  Name:        energy_manager
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Energy Manager
 *                  -Decides Transmittion
 *                  -Assign Power Priority(PP) accordingly to battery state
 *                  -PP5 corresponds to the highest energy state and PP1 to
 *                   the lowest energy state respectively.
 *
 ***************************************************/
void energy_manager()
{
  volt = (float)readVcc() / 1000;
  battery = ((volt - 3) / 1) * 100;           //formula = ((volt - min)/(max-min)) * 100

  unsigned long time;                         //count time for battery differencial "guess"

  if (battery <= 100.0 && battery >= 80.0) {
    PP = 5;
  }
  if (battery < 80.0 && battery >= 60.0) {
    PP = 4;
  }
  if (battery < 60.0 && battery >= 40.0) {
    PP = 3;
  }
  if (battery < 40.0 && battery >= 20.0) {
    PP = 2;
  }
  if (battery < 20.0 && battery >= 1.0) {
    PP = 1;
  }




  /* Fall asleep when battery falls below 10%. */
  if (battery <= 10.0) {
    /* Setup pin2 as an interrupt and attach handler. */
    attachInterrupt(0, wakeUpInterrupt, LOW );
    delay(100);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    sleep_enable();

    sleep_mode();

    /* The program will continue from here. */

    /* First thing to do is disable sleep. */
    sleep_disable();

    /*
    * We detach the interrupt to stop it from
    * continuously firing while the interrupt pin
    * is low.
    */
    detachInterrupt(0);
  }


  /*
   * Compare PP with MP to
   * decide transmission.
   * Iff PP>=MP, we can send the sample
   * information to the BS. In any other case we can:
   * 1)Throw away this sample
   * 2)Keep it in a buffer so we can send it in an upcoming round
   */
  /////////////////////////////////////////////////////////////////
  //////                           1                         //////
  /////////////////////////////////////////////////////////////////
  if (PP >= MP) {
    if (XBee.available()) {
      Serial.write("volt=");
      Serial.print( volt, DEC ); //in volts

      Serial.write("\n--->");
      Serial.print( percent, DEC ); // 0% maps to 3V remaining battery
      Serial.write("% \n");

      Serial.write("analog input");
      Serial.print( sensor, DEC );
      Serial.write(" \n");

      Serial.write("Sensor value in degrees:");
      Serial.print( degrees, DEC );
      Serial.write(" \n\n");

      Serial.flush();

      delay(2000);
    }
  }


  /////////////////////////////////////////////////////////////////
  //////                          2                          //////
  /////////////////////////////////////////////////////////////////


}



/***************************************************
*  Name:        information_manager
*
*  Returns:     Nothing.
*
*  Parameters:  None.
*
*  Description: Information Manager
*                  -Database with rules about data samples significance
*                  -Assign priorities(MP) to messages in accordance with the rules
*                  -MP1 indicates the most significant samples and MP5 the least
*                   significant samples respectively.
*               In our Case, we map the sensor values between [0, 180] degrees, with zero(MP0)
*               being a don't care value(unflexed sensor).
*
***************************************************/
void information_manager(int sample)
{

  //Rules
  if (sample > 0 && sample <= 36) {
    MP = 5;
  }
  else if (sample > 36 && sample <= 72) {
    MP = 4;
  }
  else if (sample > 72 && sample <= 108) {
    MP = 3;
  }
  else if (sample > 108 && sample <= 144) {
    MP = 2;
  }
  else if (sample > 144 && sample <= 180) {
    MP = 1;
  }
  else {                                    //(sample <= 0)--> Don't care values
    //handle this case for throwing out the samples
    MP = 0;
  }

}



/***************************************************
 *  Name:        sampling_manager
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Sampling Manager
 *                  -Asks for remaining battery
 *                  -Decides sampling frequency
 *                    Assigns sampling_frequency variable (in milliseconds)
 *                  -If application has predefined rules, define here
 *                    In our case, we will assign sampling frequency values according
 *                    to remaining battery
 *
 ***************************************************/
void sampling_manager()
{

  if (battery <= 100.0 && battery >= 90.0) {
    sampling_frequency = 5 * 1000;
  }
  if (battery < 90.0 && battery >= 80.0) {
    sampling_frequency = 8 * 1000;
  }
  if (battery < 80.0 && battery >= 70.0) {
    sampling_frequency = 11 * 1000;
  }
  if (battery < 70.0 && battery >= 60.0) {
    sampling_frequency = 14 * 1000;
  }
  if (battery < 60.0 && battery >= 50.0) {
    sampling_frequency = 17 * 1000;
  }
  if (battery < 50.0 && battery >= 40.0) {
    sampling_frequency = 20 * 1000;
  }
  if (battery < 40.0 && battery >= 30.0) {
    sampling_frequency = 23 * 1000;
  }
  if (battery < 30.0 && battery >= 20.0) {
    sampling_frequency = 26 * 1000;
  }
  if (battery < 20.0 && battery >= 10.0) {
    sampling_frequency = 29 * 1000;
  }
  if (battery < 10.0 && battery >= 0.0) {
    sampling_frequency = 32 * 1000;
  }

}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{

  /* Setup pin2 as an interrupt and attach handler. */
  //noInterrupts ();
  //  attachInterrupt(0, pin2Interrupt, LOW );
  //interrupts();
  delay(100);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  sleep_enable();

  sleep_mode();

  /* The program will continue from here. */

  /* First thing to do is disable sleep. */
  sleep_disable();
  detachInterrupt(0);
}



/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: The setup function runs once when
 *               you press reset or power the board.
 *
 ***************************************************/
void setup()
{
  // initialize digital pin 13 as an output.
  pinMode(ledPin, OUTPUT);
  pinMode(wakePin, INPUT);

  XBee.begin(9600);
  Serial.begin(9600);
}


/***************************************************
 *  Name:        loop
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Main application loop.
 *
 ***************************************************/
void loop() {

  int sensor, degrees;
  /*
   // Turn the LED on and delay for 1 second
   digitalWrite(ledPin, HIGH);
   delay(1000);
   //Sleepy::loseSomeTime(1000);

   // Turn the LED off and until data has been sent through XBee communication
   digitalWrite(ledPin, LOW);
   //Sleepy::loseSomeTime(1000);
  */


  /*
   * Take a sample from the
   * sensor and map it to degrees
   */
  sensor = analogRead(0);
  degrees = map(sensor, 312, 415, 0, 90);

  /* 
   * Call the Sampling Manager to
   * assign the sampling frequency
   * and wait for that given time.
   * This is the last thing we do!!!
   */ 
  sampling_manager();
  delay(sampling_frequency);


  /*
   * Calculate remaining battery statistics
   * This one will BE DONE IN THE enrgy_manager_function
   */
  //float volt = (float)readVcc()/1000;
  //float percent = ((volt-3)/1)*100;     //formula = ((volt - min)/(max-min)) * 100



  //THE SENDING PROCCESS WILL BE MOVED INTO ENERGY_MANAGER
  /*
   * Send sensor value and battery statistics throught
   * Xbee communication. Send a dummy character from XBee's
   * serial monitor(from the PC) to notify arduino
   * about the other XBee and start the communication.
   */
  /*
  if (XBee.available()) {
   Serial.write("volt=");
   Serial.print( volt, DEC ); //in volts

   Serial.write("\n--->");
   Serial.print( percent, DEC ); // 0% maps to 3V remaining battery
   Serial.write("% \n");

   Serial.write("analog input");
   Serial.print( sensor, DEC );
   Serial.write(" \n");

   Serial.write("Sensor value in degrees:");
   Serial.print( degrees, DEC );
   Serial.write(" \n\n");

   Serial.flush();

   delay(2000);
  }
  */

}





