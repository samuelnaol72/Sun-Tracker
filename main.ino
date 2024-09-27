#include <main_board_v0x5.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoLowPower.h>
double DEG_2_RAD = M_PI / 180;
double RAD_2_DEG = 180 / M_PI;
// For RTC
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char daysOfTheWeek_short[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

void calc_sun_vector(double return_vector[3], double sensor_data[4] );
Cubesat_Board myboard = Cubesat_Board();



void calc_sun_vector(double return_vector[3], double sensor_data[4] ){

  // The following operator is obtained from the given formula on the lecture. For my case, I oriented @50 deg and used the angle to calculate the Normal and then the operator.
    double Operator[3][4]={
     {0.7472 ,  -0.7472, 0,        0},
     {0,    0,    0.7472,    -0.7472},
     {0.3364,    0.3364 , 0.3364, 0.3364}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++){
                return_vector[i] += Operator[i][j] * sensor_data[j];
        }
    }
    double norm=sqrt(return_vector[0]*return_vector[0]+return_vector[2]*return_vector[2]+return_vector[1]*return_vector[1]);
    if(norm!=0){
      for(int i; i<3;i++){
        return_vector[i]=return_vector[i]/norm;
      }
    }
}


void azi_and_alt_of_sun(double sun_azi_and_alt[2], double z, double t, double _WV_Tel_RF[3]){
    _WV_Tel_RF[0]=- _WV_Tel_RF[0];
    _WV_Tel_RF[1]=- _WV_Tel_RF[1];
    z = z * DEG_2_RAD; t = t * DEG_2_RAD;
    double _sin_z = sin(z); double _cos_z = cos(z); double _sin_t = sin(t); double _cos_t = cos(t);
    
    //the following matrix is the inverse=transpose of the transformation matrix we learned in the class
    double _TEL_2_GIMB[3][3];
    _TEL_2_GIMB[0][0] = _cos_z;     _TEL_2_GIMB[0][1] = _sin_t * _sin_z; _TEL_2_GIMB[0][2] = _cos_t * _sin_z;
    _TEL_2_GIMB[1][0] = 0;          _TEL_2_GIMB[1][1] = _cos_t;          _TEL_2_GIMB[1][2] = -1*_sin_t;
    _TEL_2_GIMB[2][0] = -1* _sin_z;    _TEL_2_GIMB[2][1] = _sin_t * _cos_z; _TEL_2_GIMB[2][2] = _cos_z * _cos_t;

    // This is matrix multiplication to obtain star's directonal vector in local or Gimbal reference frame.
    double _Vector_in_GMRF[3]={0,0,0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
                _Vector_in_GMRF[i] += _TEL_2_GIMB[i][j] * _WV_Tel_RF[j];

    }

    double norm=sqrt(_Vector_in_GMRF[0]*_Vector_in_GMRF[0]+_Vector_in_GMRF[2]*_Vector_in_GMRF[2]+_Vector_in_GMRF[1]*_Vector_in_GMRF[1]);
    if(norm!=0){
      for(int i; i<3;i++){
        _Vector_in_GMRF[i]=_Vector_in_GMRF[i]/norm;
      }
    }

    
  // The following is to obtain the two z values we can use to make our star at (0,0,1) in Telescop's frame
    double z_new1 = atan(_Vector_in_GMRF[0] / _Vector_in_GMRF[2]);
    double z_new2 = atan(_Vector_in_GMRF[0] / _Vector_in_GMRF[2]) +M_PI;


   // Using the second equations to find the possible t values for z_new1
    double t_new1_for_z_new1 = atan(-1*( _Vector_in_GMRF[1] / ( _Vector_in_GMRF[0] * sin(z_new1) + _Vector_in_GMRF[2] * cos(z_new1) ) )) ;
    double t_new2_for_z_new1 = atan(-1 * (_Vector_in_GMRF[1] / (_Vector_in_GMRF[0] * sin(z_new1) + _Vector_in_GMRF[2] * cos(z_new1))))+M_PI;
    
    // using third equation in ppt to estabilish the correct t value for z_new1
    double t_new_for_z_new1 = t_new1_for_z_new1;
    double resul3 = _Vector_in_GMRF[0] * cos(t_new2_for_z_new1) * sin(z_new1) - _Vector_in_GMRF[1] * sin(t_new2_for_z_new1) + _Vector_in_GMRF[2] * cos(z_new1) * cos(t_new2_for_z_new1);
    if (resul3>=0.95 && resul3<+1.05) {
        t_new_for_z_new1 = t_new2_for_z_new1;
    }

    
    //for z_new2
    // Using the second equations to find the possible t values for z_new2
    double t_new1_for_z_new2 = atan(-1 * (_Vector_in_GMRF[1] / (_Vector_in_GMRF[0] * sin(z_new2) + _Vector_in_GMRF[2] * cos(z_new2)) ) ) ;
    double t_new2_for_z_new2 = atan(-1 * (_Vector_in_GMRF[1] / (_Vector_in_GMRF[0] * sin(z_new2) + _Vector_in_GMRF[2] * cos(z_new2))))+ M_PI;
    
    // using third equation in ppt to estabilish the correct t value for z_new2
    double t_new_for_z_new2 = t_new1_for_z_new2;
    double resul2 = _Vector_in_GMRF[0] * cos(t_new2_for_z_new2) * sin(z_new2) - _Vector_in_GMRF[1] * sin(t_new2_for_z_new2) + _Vector_in_GMRF[2] * cos(z_new2) * cos(t_new2_for_z_new2);
    if (resul2>=0.95&& resul2<=1.05) {
        t_new_for_z_new2 = t_new2_for_z_new2;
    }

    // the following code is to estabilish the set with minimum mount rotation based on z_new values
    double z_new = 0;
    double t_new = 0;
   if (z_new1<-0.001){
      z_new1+=2*M_PI;
    }
    if (z_new2<-0.001){
      z_new2+=2*M_PI;
    }
    
    if (fabs(t_new_for_z_new1)<fabs(t_new_for_z_new2)){
        z_new = z_new1;
        t_new = t_new_for_z_new1;
    }
    else {
         z_new = z_new2;
         t_new = t_new_for_z_new2;
    }
   
   sun_azi_and_alt[0]=z_new*RAD_2_DEG;
   sun_azi_and_alt[1]=t_new*RAD_2_DEG;
   return;
}


// This is used to obtain the angle of rotation of base from the original orientation
double angle_diff(double A[], double B[]) {
    double scalar_prod = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    double norm_A = sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
    double norm_B = sqrt(B[0] * B[0] + B[1] * B[1] + B[2] * B[2]);
    double angle = acos(scalar_prod / (norm_A * norm_B))*RAD_2_DEG;
    return angle;
}

void print_with_leading_zero(int value){
  if(value < 10){
    myboard.display->print(0);
  }
  myboard.display->print(value, DEC);
}


void displaytime_and_batterylevel(){
   if(myboard.read_button_c()){
    
        myboard.begin_display(true);
        myboard.display->clearDisplay();
        myboard.display->setCursor(0,0);
        double voltage = analogRead(A7)*(3.3/1023.0)*2;
        float percentage = (voltage- 3.3)/(0.9);
        percentage = percentage * 100;
        myboard.display->print("Battery :");
        myboard.display->print(percentage);
        myboard.display->println("%");

        DateTime now = rtc.now();
        myboard.display->print(now.year(), DEC);
        myboard.display->print('/');
        print_with_leading_zero(now.month());
        myboard.display->print('/');
        print_with_leading_zero(now.day());
        myboard.display->print(" (");
        myboard.display->print(daysOfTheWeek_short[now.dayOfTheWeek()]);
        myboard.display->println(") ");
        print_with_leading_zero(now.hour());
        myboard.display->print(':');
        print_with_leading_zero(now.minute());
        myboard.display->print(':');
        print_with_leading_zero(now.second());
        myboard.display->println();
        myboard.display->display();
        delay(3000);
        myboard.display->clearDisplay();
        myboard.display->setCursor(0,0);
        myboard.display->display();
      }
}

//To upload the data
File data;
const float conversion= (3.3/1023.0)*2;
int yr, mon, d;
int days[1];
int coun=0;
char fileName[12];

void _put_DATA_TO_THE_SD(int sun_lux, int sun_azi, int sun_alt, int base_azi, int base_alt ){
  String head = "Unix_Time, Bat_level_Percent, Sun_Intensity_Lux, Sun_Azi_Deg, Sun_Alt_Deg, Base_Azi_Deg, Base_Alt_Deg, Azi_Servo_Deg, Alt_Servo_Deg\n";
  if (coun==0) {
    DateTime now = rtc.now();
    yr= now.year();
    mon= now.month();
    d=now.day();
    days[0]= now.day();
    sprintf(fileName, "%d%d%d.csv",yr,mon,d);
    data = SD.open(fileName, FILE_WRITE);
    data.print(head);
    data.close();
    myboard.display->println("New file has been initialized");
    myboard.display->display();
    coun++;
      }
  DateTime now = rtc.now();
  if(now.day()!=days[0]){
    days[0]=now.day();
    DateTime now = rtc.now();
    yr= now.year();
    mon= now.month();
    d=now.day();
    sprintf(fileName, "%d%d%d.csv", yr,mon,d);
    data = SD.open(fileName, FILE_WRITE);
    data.print(head);
    data.close();
    myboard.display->println("New file has been initialized!");
    myboard.display->display();
  }
  Serial.println(fileName);
  data = SD.open(fileName, FILE_WRITE);
  String data_a="";
  data_a=data_a+ String(now.unixtime())+","+String( ((analogRead(A7)*conversion-3.3)/ 0.9) *100)+ ","+ String(sun_lux) +","+String(sun_azi)+","+String(sun_alt)+","+String(base_azi)+","+String(base_alt)+","+String(my_azi_servo.read())+","+String(my_alt_servo.read())+"\n"; 
 
  if (data) {
        data.print(data_a);
        data.close();
        myboard.display->println(fileName);
        myboard.display->display();
        delay(1000);
        myboard.display->clearDisplay();
        myboard.display->setCursor(0,0);
        myboard.display->display();
    }
  else {
    myboard.display->println("Error while uploading the data");
    myboard.display->display();
  }}



void scan_and_put_at_correct_alt (){
      int int_ligh=0;
      int pos=0;
      for (int i=0;i<19;i++){
      my_azi_servo.write(180-10*i);
      delay(1000);
      if (myboard.get_lux_from_sensor_id(0)+myboard.get_lux_from_sensor_id(1)+myboard.get_lux_from_sensor_id(2)+myboard.get_lux_from_sensor_id(3)> int_ligh){
        pos=180-10*i;
        Serial.println(pos);
        int_ligh=myboard.get_lux_from_sensor_id(0)+myboard.get_lux_from_sensor_id(1)+myboard.get_lux_from_sensor_id(2)+myboard.get_lux_from_sensor_id(3);
        Serial.println("Azi");
      }
      delay(10);
      }
      my_azi_servo.write(pos);
      delay(1000);

      int int_light=0;
      int posi=0;
      for (int i=0;i<12;i++){
      my_alt_servo.write(120-10*i);
      delay(1000);
      if (myboard.get_lux_from_sensor_id(0)+myboard.get_lux_from_sensor_id(1)+myboard.get_lux_from_sensor_id(2)+myboard.get_lux_from_sensor_id(3)> int_light){
        posi=180-10*i;
        Serial.println(posi);
        int_ligh=myboard.get_lux_from_sensor_id(0)+myboard.get_lux_from_sensor_id(1)+myboard.get_lux_from_sensor_id(2)+myboard.get_lux_from_sensor_id(3);
      }
      delay(10);
      }
      my_alt_servo.write(posi);
  }
 
  




int calcul_base_azi(double base_angles[1]){
    sensors_event_t gravityvec, orientationData, magD;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&magD, Adafruit_BNO055::VECTOR_MAGNETOMETER);
   
    double norm_m = sqrt(pow(magD.orientation.x,2)+pow(magD.orientation.z,2));

    if (magD.orientation.x <= 0){
        base_angles[0] = 360-acos(magD.orientation.z/norm_m)*180/PI ;

    }
    else{
        base_angles[0] = acos(magD.orientation.z/norm_m)*180/PI;
    }
   return  0;
}




void setup(){
  myboard.payload_power(1);
  myboard.servo_power(  1);
  myboard.begin(115200);

  //IMU sensor initialization
  myboard.display->println("IMU Sensor test");
  myboard.display->display(); // actually display all of the above
  if (!bno.begin())
  {
    myboard.display->println("No BNO055 detected");
    myboard.display->display();
    while(1);
  }
  else{
    myboard.display->println("BNO055 OK");
  }

  // servo initialization
  my_azi_servo.attach(10);
  my_alt_servo.attach(11);
  myboard.display->clearDisplay();
  myboard.display->setCursor(0,0);
  myboard.display->println("SERVO OK");
  myboard.display->display();
  delay(2000);

  // RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    myboard.display->println("Couldn't find RTC");
    myboard.display->display();
    while (1) delay(10);
  }
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    myboard.display->println("RTC not init. Setting time");
    myboard.display->display();

  }
  rtc.start();
  float drift = 5; 
  float period_sec = (60000);  
  float deviation_ppm = (drift / period_sec * 1000000); 
  float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  rtc.calibrate(PCF8523_OneMinute, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  myboard.display->clearDisplay();
  myboard.display->setCursor(0,0);
  myboard.display->print("Offset is ");
  myboard.display->println(offset);
  myboard.display->display();
  delay(2000);
  // SD
  if (!SD.begin(12)) {
    myboard.display->println("SD Initialization failed!");
    myboard.display->display();
    while(SD.begin(12)){};
  }
  myboard.display->println("SD Initialization done.");
  delay(1000);
  myboard.display->display();
}

int nw_base_azi=180;
int prev_alt=0;
int prev_azi=0;
double sensor_read[4];
int status_led = 0;
int count=0;

double Old_accel[3]={0,0,9.8};
double old_b=0;
double Old_mag[3]={0,0,0};
int ddday[1]={0};
int dday;

void loop() {
  myboard.display->clearDisplay();
  myboard.display->setCursor(0,0);
  myboard.display->display();
  displaytime_and_batterylevel();
  double base_alt=0;
  if(count==0){
       my_azi_servo.write(90);
       delay(1000);
       my_alt_servo.write(90);
       delay(1000);
       count++; 
  }
 
  
  DateTime now = rtc.now();
  dday= now.day();
  if (now.hour() > 6 && now.hour() < 19 ){
          if (dday!=ddday[0]){
            scan_and_put_at_correct_alt ();
            ddday[0]=dday;
            }
          
          DateTime now = rtc.now();
          int tim= now.unixtime();
          displaytime_and_batterylevel();
          if (tim %300==0){
                displaytime_and_batterylevel();
                myboard.servo_power(1);
                sensors_event_t eulerdata;
                sensors_event_t eulerdata1;
                uint8_t system, gyro, accel, magn;
                system = gyro = accel = magn = 0;
                bno.getCalibration(&system, &gyro, &accel, &magn);
                bno.getEvent(&eulerdata, Adafruit_BNO055::VECTOR_EULER);
                int x=my_alt_servo.read();
                int y=my_azi_servo.read();
                
                while(system=0){
                  // The following is just to move the sensor board in abnormal manner just for the sole reason of callibration, in a sense, to wait till system level is 3
                  Serial.println("Callibrating IMU~");
                  sensors_event_t eulerdata;
                  sensors_event_t eulerdata1;
                  uint8_t system, gyro, accel, magn;
                  system = gyro = accel = magn = 0;
                  bno.getCalibration(&system, &gyro, &accel, &magn);
                  bno.getEvent(&eulerdata, Adafruit_BNO055::VECTOR_EULER);
                
                  my_alt_servo.write(0);
                  delay(1000);
                  my_alt_servo.write(90);
                  delay(1000);
                  my_alt_servo.write(120);
                  delay(1000);
                  
              
                  my_azi_servo.write(0);
                  delay(1000);
                  my_azi_servo.write(90);
                  delay(1000);
                  my_azi_servo.write(170);
                  delay(1000);
                  
                }
                 my_alt_servo.write(x);
                 delay(1000);
                 my_azi_servo.write(y);
                 delay(1000);
                 
                // store current positions before checking changes in base orientations
                prev_alt=x;
                prev_azi=y;
                // The following is to check amount of rotations of base from the original position.
                my_alt_servo.write(90);
                delay(1000);
                Serial.println("Check alt");
                my_azi_servo.write(90);
                delay(1000);
                Serial.println("check azi");
                
                double base_angl[1]={0};
                calcul_base_azi(base_angl);
                Serial.println(fabs(base_angl[0]- nw_base_azi));
                if(fabs(base_angl[0]- nw_base_azi)> 15){
                   scan_and_put_at_correct_alt ();
                }
                nw_base_azi= base_angl[0];
                my_alt_servo.write(0);
                delay(1000);
                
                int B_AZ_diff=nw_base_azi-180;
                Serial.println(nw_base_azi);
                double  B_Alt_diff=0;
                
                if (my_alt_servo.read()==0 &&  my_azi_servo.read()==90){
                          // To check whether base alt is still at original position or not
                          imu:: Vector<3> acc= bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
                          
                          double accX=0;   double accY=0;   double accZ=0;
                          for (int i=0; i<100; i++){
                               accX+=acc.x(); accY+=acc.y(); accZ+=acc.z();
                               delay(8);
                          }
                          accX=accX/100; accY=accY/100; accZ=accZ/100;
                          if (accZ>9.7) B_Alt_diff=0;
                          else {
                                B_Alt_diff=fabs(acos(accZ/9.8)* RAD_2_DEG);
                                if (accY>0 && accX<0 ){
                                  B_Alt_diff=-B_Alt_diff;
                                }
                          }
                          base_alt= B_Alt_diff;
                }
                Serial.println(base_alt);
                delay(3000);
        
                // This is to put back the sensor board to its original position
                my_alt_servo.write(prev_alt);
                delay(1000);
                my_azi_servo.write(prev_azi);
                delay(1000);
                Serial.println("Returned to original place");
                double sun_vector[3]={0,0,0};
                sensor_read[0]=myboard.get_lux_from_sensor_id(0);
                sensor_read[1]=myboard.get_lux_from_sensor_id(2);
                sensor_read[2]=myboard.get_lux_from_sensor_id(1);
                sensor_read[3]=myboard.get_lux_from_sensor_id(3);
                calc_sun_vector(sun_vector,sensor_read);                                                                               
                
              
               
                double sun_azi_and_alt[2]={0,0};
                azi_and_alt_of_sun(sun_azi_and_alt, 270-my_azi_servo.read(),90-my_alt_servo.read(),sun_vector);
                
                if (270- sun_azi_and_alt[0]< 180){
                  my_azi_servo.write(270- sun_azi_and_alt[0]); delay(200);
                }
                else my_azi_servo.write(180);delay(2000);
                
        
                if (90- sun_azi_and_alt[1]< 110){
                  my_alt_servo.write(90- sun_azi_and_alt[1]); delay(200);
                }
                else my_azi_servo.write(110);delay(2000);
                delay(200);
                
                sun_azi_and_alt[0]+=B_AZ_diff;
                sun_azi_and_alt[1]+=B_Alt_diff;
                
                _put_DATA_TO_THE_SD(myboard.get_lux_from_sensor_id(1)/0.74,sun_azi_and_alt[0],sun_azi_and_alt[1], nw_base_azi,base_alt);
                myboard.servo_power(status_led);
               
               for (int i=0; i<20; i++){
                    myboard.hmi_power(0);
                    LowPower.sleep(10000);
                    myboard.hmi_power(1);
                    delay(50);
                    displaytime_and_batterylevel();
                  }
          }
          
         else{
             displaytime_and_batterylevel();
             delay(200);
          }           
  }
  else{
     LowPower.sleep(5000);
     displaytime_and_batterylevel();
    _put_DATA_TO_THE_SD(0,0,0, nw_base_azi,base_alt);
    LowPower.sleep(5000);
  }
}
