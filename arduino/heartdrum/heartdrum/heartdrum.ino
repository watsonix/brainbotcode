/*
motor on every heartbeat
 rohan dixit 2014
 
 ECG
 // baseline is the moving average of the signal - the middle of the waveform
 // the idea here is to keep track of a high frequency signal, HFoutput and a 
 // low frequency signal, LFoutput
 // The HF signal is shifted downward slightly (heartbeats are negative peaks)
 // The high freq signal has some hysterisis added. When the HF signal is less than the 
 // shifted LF signal, we have found a heartbeat.
 

 */


int binOut;     // 1 or 0 depending on state of heartbeat
int BPM;

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

}



void loop() {
  static  int valley=0, peak=0, smoothPeak, smoothValley, binOut, lastBinOut, BPM;
  static unsigned long lastTotal, lastMillis,  valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime=millis(), lastBeat, beat;
  static float baseline, HFoutput, HFoutput2, shiftedOutput, LFoutput, hysterisis;

  
  unsigned long start;
  int i=0;
  int signalSize;
  start = millis();
  
  
  //100 milliseconds on motor
  unsigned long diff = millis() - motortimeon;
//  Serial.println(millis());
//  Serial.println(motoron);
//  Serial.println(diff);
//  Serial.println('n');
  if ( diff < 100){
        digitalWrite(A1, HIGH);
        digitalWrite(A2, HIGH);
        Serial.println('on');

      }else{
        digitalWrite(A1, LOW);      
        digitalWrite(A2, LOW);  
      }
      

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

  //IF HEARTBEAT
  if (lastBinOut == 0 && binOut == 1){
    lastBeat = beat;
    beat = millis();
    BPM = 60000 / (beat - lastBeat);

    if (BPM > min_bpm && BPM < max_bpm){
//      Serial.println(BPM);  
      motortimeon = millis();      

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

