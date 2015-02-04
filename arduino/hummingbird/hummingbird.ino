/*
implement constant butterfly/hummingbird feedback
 rohan dixit 2014
 
 ECG
 // baseline is the moving average of the signal - the middle of the waveform
 // the idea here is to keep track of a high frequency signal, HFoutput and a 
 // low frequency signal, LFoutput
 // The HF signal is shifted downward slightly (heartbeats are negative peaks)
 // The high freq signal has some hysterisis added. When the HF signal is less than the 
 // shifted LF signal, we have found a heartbeat.
 
 VIBROMOTORS
 assume there are 5 vibromotors.
 
 all off
 o o o o o 
 
 full
 x x x x x
 
 middle
 o x x x o
 
 small
 o o x o o 
 
 
 use last 10 (say) BPM measurements to calculate an average BPM. 
 
 if you're above average, activate "full" setting.
 if you're right at average, activate "middle" setting. 
 if you're below average, activate "small" setting.
 
 */


const int SAMPLES_TO_AVERAGE = 3;             // samples for BPM array, 3-5 seems useful
int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;
int BPM_memory[3] = {}; //we'll store the previous x number of heartbeats here, so we can find when new heartbeats are "slower"
int starter_bpm = 60; //fill BPM array with dummy data on boot.
int current_pos = 0; //pointer to current location in the BPM_array

int total;     // all three LED reads added together
unsigned long time = millis();
unsigned long looptime, motortimeon;

int signalSize;          // the heartbeat signal minus the offset
int max = 0;
int min = 1000;

int max_bpm = 120;
int min_bpm = 30;  
bool beat_on = false;
bool motoron = false;



void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);  
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  
  //fill BPM_memory array with dummy data.
  for (int i = 0; i < SAMPLES_TO_AVERAGE; ++i){
   BPM_memory[i] = starter_bpm;
  }

}



void loop() {
  static  int valley=0, peak=0, smoothPeak, smoothValley, binOut, lastBinOut, BPM;
  static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
  static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;

  
  unsigned long start;
  int i=0;
  int signalSize;
  start = millis();

  int total = analogRead(A0);
//  Serial.println(total);

  baseline = smooth(total, 0.99, baseline);   // 
  HFoutput = smooth((total - baseline), 0.2, HFoutput);    // recycling output - filter to slow down response
  HFoutput2 = HFoutput + hysterisis;     
  LFoutput = smooth((total - baseline), 0.95, LFoutput);
  // heartbeat signal is inverted - we are looking for negative peaks
  shiftedOutput = LFoutput - (signalSize * .05);
  // We need to be able to keep track of peaks and valleys to scale the output for 
  // user convenience. Hysterisis is also scaled.
  if (HFoutput  > peak) peak = HFoutput; 
  if (peak > 1500) peak = 1500; 

  if (millis() - lastPeakTime > 1800){  // reset peak detector slower than lowest human HB
    smoothPeak =  smooth((float)peak, 0.6, (float)smoothPeak);  // smooth peaks
    peak = 0;
    lastPeakTime = millis();
  }

  if (HFoutput  < valley)   valley = HFoutput;
  if (valley < -1500) valley = -1500;

  if (millis() - lastValleyTime > 1800){  // reset valleys detector slower than lowest human HB
    smoothValley =  smooth((float)valley, 0.6, (float)smoothValley);  // smooth valleys
    valley = 0;
    lastValleyTime = millis();           
  }

  signalSize = smoothPeak - smoothValley;  // this the size of the smoothed HF heartbeat signal

  if(HFoutput2 < shiftedOutput){
    lastBinOut = binOut;
    binOut = 1;
    //        Serial.println("\ty");
    hysterisis = - constrain((signalSize / 15), 35, 120) ;   // you might want to divide by smaller number
    // if you start getting "double bumps"
  } 
  else {
    //        Serial.println("\tn");
    lastBinOut = binOut;
    binOut = 0;
    hysterisis = constrain((signalSize / 15), 35, 120);    // ditto above

  } 

  //RIGHT AFTER A HEARTBEAT
  //    if (lastBinOut == 1 && binOut == 0){
  //        Serial.println(binOut);
  //        digitalWrite(9, LOW);  //pin 9 is the buzzer
  //    }


  //IF HEARTBEAT
  if (lastBinOut == 0 && binOut == 1){
    lastBeat = beat;
    beat = millis();
    BPM = 60000 / (beat - lastBeat);

    if (BPM > min_bpm && BPM < max_bpm){
      Serial.println(BPM);  
      
      //add to array
      if (current_pos + 1 == SAMPLES_TO_AVERAGE){
        current_pos = 0;
      }
      else{
        current_pos++;
      }
      BPM_memory[current_pos] = BPM;
      
      //calc avg
      float avg = average(BPM_memory);
      
      //turn motor on if below average BPM, else turn it off
       float diff = avg - BPM;
       if (diff > 0 ){
        float analog_pump_up = scale_num(diff, 0, 11); //guesstimate, play with parameters. this means 11 BPM below average is max drop we'd reward with increased buzz.
        Serial.println(diff);
        Serial.println(analog_pump_up);
        
        analogWrite(A1, analog_pump_up);
//        digitalWrite(A1, HIGH);
//        digitalWrite(A2, HIGH);

      }else{
        digitalWrite(A1, LOW);      
        digitalWrite(A2, LOW);  
      }

    }
  }

  // wait for Analog-Digital converter stabilization
  delay(2);
}


float scale_num(float input, float min_val, float max_val){
   //scales an input, say heartbeat BPM, to 0-255 for analogWrite, returns scaled value
   return input * 255.0 / (max_val - min_val);
}

// simple smoothing function for  heartbeat detection and processing
float smooth(float data, float filterVal, float smoothedVal){
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0.0){
    filterVal = 0.01;
  }
  smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}

float average(int array[]){
   float sum = 0.0;  // double precision.
   for (int i = 0; i < SAMPLES_TO_AVERAGE; ++i)
   {
       sum += array[i];
   }
   return sum/SAMPLES_TO_AVERAGE;
}
 
/*
//takes BPM and, using the average BPM of some number of previous BPM values, determines which vibromotors go on.
 void vibrate_control(float bpm){
 
 // subtract the last reading:
 total= total - readings[index]; 
 
 // place new BPM reading in the array:  
 readings[index] = bpm; 
 
 //do something with the vibromotor
 hummingbird_always_on(bpm, average);
 
 // add the BPM to the total:
 total= total + readings[index];  
 
 // advance to the next position in the BPM array:  
 index = index + 1;                    
 
 // if we're at the end of the array...
 if (index >= numReadings)              
 // ...wrap around to the beginning: 
 index = 0;                           
 
 // calculate the average:
 average = total / numReadings;
 
 
 // println BPM
 Serial.println("vibromotor");
 Serial.println(bpm);  
 // println average
 Serial.println(average);   
 
 }
 
 
 
 void hummingbird(float bpm, float average){
 float fudge = 2; //BPM to fudge things
 int delayms = 100; //time to vibrate after each beat
 
 if (bpm < average - fudge){
 //narrow setting
 analogWrite(vibe[2], 200); 
 delay(delayms);
 analogWrite(vibe[2], 0); 
 }
 else if (bpm > average + fudge){
 //wide setting
 analogWrite(vibe[0], 200); 
 analogWrite(vibe[1], 200); 
 analogWrite(vibe[2], 200); 
 analogWrite(vibe[3], 200);
 analogWrite(vibe[4], 200);
 delay(delayms);
 analogWrite(vibe[0], 0); 
 analogWrite(vibe[1], 0); 
 analogWrite(vibe[2], 0); 
 analogWrite(vibe[3], 0); 
 analogWrite(vibe[4], 0); 
 }
 else {
 //average or medium setting
 analogWrite(vibe[1], 200); 
 analogWrite(vibe[2], 200); 
 analogWrite(vibe[3], 200); 
 delay(delayms);
 analogWrite(vibe[1], 0); 
 analogWrite(vibe[2], 0); 
 analogWrite(vibe[3], 0); 
 }
 }
 
 
 void hummingbird_always_on(float bpm, float average){
 float fudge = 2; //BPM to fudge things
 
 //turn em all off.
 analogWrite(vibe[0], 0); 
 analogWrite(vibe[1], 0); 
 analogWrite(vibe[2], 0); 
 analogWrite(vibe[3], 0); 
 analogWrite(vibe[4], 0); 
 
 if (bpm < average - fudge){
 //narrow setting
 analogWrite(vibe[2], 200); 
 }
 else if (bpm > average + fudge){
 //wide setting
 analogWrite(vibe[0], 200); 
 analogWrite(vibe[1], 200); 
 analogWrite(vibe[2], 200); 
 analogWrite(vibe[3], 200);
 analogWrite(vibe[4], 200);
 }
 else {
 //average or medium setting
 analogWrite(vibe[1], 200); 
 analogWrite(vibe[2], 200); 
 analogWrite(vibe[3], 200); 
 }
 }
 
 */


