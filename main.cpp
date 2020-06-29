#include "mbed.h"
#include "mbed_rpc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "bbcar.h"
#include "bbcar_rpc.h"

// For OpenMV
Serial uart(D1, D0); //tx,rx

// For distance decoder
Ticker encoder_ticker_r;
Ticker encoder_ticker_l;
DigitalIn pin7(D7);
DigitalIn pin6(D6);
parallax_encoder encoder_right(pin7, encoder_ticker_r);
parallax_encoder encoder_left(pin6, encoder_ticker_l);

// For serv control
Ticker servo_ticker;
PwmOut pin9(D9), pin8(D8);
BBCar car(pin8, pin9, servo_ticker);

// For getting ping data
DigitalOut redLED(LED1);
DigitalInOut pin10(D10);
parallax_ping ping(pin10);

// Xbee transfer
RawSerial xbee(D12, D11);
Thread t;
EventQueue queue(16 * EVENTS_EVENT_SIZE);
int mission_gap = 0;
int count = 0;
char behavior = 'S';


// Xbee keeping communication
void Xbee_Communication(void)
{
  while (1)
  {
    if (mission_gap == 1)
    {
      wait(1);
    }
    if (mission_gap == 0)
    {
      // behavior logging: S for straight, L for left, R for right, B for back
      xbee.printf("%d %c\r\n", count, behavior);
      count++;
      wait(1);
    }
  }
}

// ping guide
void ping_guide(int close, int speed)
{
  car.goStraight(speed);
  while (1)
  {
    if ((float)ping < close)
    {
      car.stop();
      break;
    }
    wait_ms(10);
  }
}

// UART command for matrix
void get_matrix(void)
{
  mission_gap = 1;
  redLED = 0;
  // send request
  char s[10];
  sprintf(s, "matrix");
  uart.puts(s);
  wait(5);
  // receive information
  if (uart.readable())
  {
    char temp;
    char rev[20];
    int counter = 1;
    xbee.printf("80662\r\n");
    while (1)
    {
      temp = uart.getc();

      if (temp == '\0')
      {
        temp = uart.getc();
      }
      else if (temp != '\r')
      {
        rev[counter] = temp;
        counter++;
      }
      else
      {
        break;
      }
    }
    for (int i = 1; i < counter; i++)
    {
      xbee.putc(rev[i]);
    }
    counter = 1;
    xbee.printf("\r\n");
  }
  mission_gap = 0;
  redLED = 1;
}

// turn round: 1 for left ; 0 for right
void turn_round(int turn_round_lr)
{
  // left
  if (turn_round_lr == 1)
  {
    encoder_right.reset();
    car.turn(100, 0.3);
    
    while (encoder_right.get_cm() < 21.4)
      wait_ms(1);
    car.stop();
  }
  // right
  if (turn_round_lr == 0)
  {
    encoder_left.reset();
    car.turn(100, -0.3);
    
    while (encoder_left.get_cm() < 28.4)
      wait_ms(1);
    car.stop();
  }
}

// UART command for identification
void get_identification(void)
{
  mission_gap = 1;
  redLED = 0;
  // send request
  char s[10];
  sprintf(s, "identification");
  uart.puts(s);
  wait(5);
  // receive information
  if (uart.readable())
  {
    char recv = uart.getc();
    xbee.printf("80661\r\n");
    xbee.putc(recv);
    xbee.printf("\r\n");
  }
  mission_gap = 0;
  redLED = 1;
}

// get ping data
void identify_ping_data(void)
{
  float obj_data[3];
  int result = 1;
  float dif1, dif2;
  redLED = 0;
  mission_gap = 1;
  xbee.printf("80663\r\n");
  // grab data
  obj_data[0] = (float)ping;
  car.turn(30, 0.01);
  wait(0.5);
  car.stop();
  wait(2);
  obj_data[1] = (float)ping;
  car.turn(-30, 0.01);
  wait(1);
  car.stop();
  wait(2);
  obj_data[2] = (float)ping;
  car.turn(30, 0.01);
  wait(0.5);
  car.stop();
  // calculate difference
  dif1 = obj_data[1] - obj_data[0];
  dif2 = obj_data[2] - obj_data[0];
  if (dif1 < -0.8)
  {
    if (dif2 > 0.7)
      if (dif1 > -5)
        if (dif1 < -1.9)
          if (dif2 > 2)
            result = 1;
        else
          result = 0;
  }
  if (dif1 < -4)
    if(dif2 > 0.8)
      if(dif2 < 4)
        result = 3;
  else
    result = 2;
  // output result to xbee
  if (result == 0)
    xbee.printf("square\r\n");
  if (result == 1)
    xbee.printf("slope\r\n");
  if (result == 2)
    xbee.printf("sharp\r\n");
  if (result == 3)
    xbee.printf("depression\r\n");
  mission_gap = 0;
  redLED = 1;
}


int main()
{
  // initialization setting
  xbee.baud(9600);
  uart.baud(9600);
  redLED = 1;
  
  // start xbee sending thread
  t.start(callback(&queue, &EventQueue::dispatch_forever));
  queue.call(&Xbee_Communication);
  
  // ****** go to mission 1 ****** //
  // go straight 
  wait(2);
  behavior = 'S';
  ping_guide(30, 100);
  // get matrix 
  get_matrix();
  // turn left
  wait(2);
  behavior = 'L';
  turn_round(1);
  // go straight 
  wait(1);
  behavior = 'S';
  ping_guide(18, 100);
  // ****** arrive mission 1 ****** //
  
  // ******  mission 1 start ****** //
  // turn back left
  behavior = 'L';
  car.turn(-100,0.3);
  wait(2.5);
  car.stop();
  // backforward
  wait(1);
  behavior = 'B';
  car.goStraight(-100);
  while (1)
  {
    if ((float)ping > 40)
    {
      car.stop();
      break;
    }
    wait_ms(10);
  }
  // frontforward 
  wait(1);
  behavior = 'S';
  ping_guide(30, 100);
  // get identification 
  get_identification();
  // turn right
  wait(1.5);
  behavior = 'R';
  car.turn(100, -0.3);
  wait(2.5);
  car.stop();
  // go straight
  wait(1);
  behavior = 'S';
  car.goStraight(100);
  wait(2.1);
  car.stop();
  // ****** finish mission 1 ****** //
  
  // ****** go to mission 2 ****** //
  // turn right
  wait(1);
  behavior = 'R';
  turn_round(0);
  // go straight 
  wait(1);
  behavior = 'S';
  car.goStraight(100);
  wait(6);
  car.stop();
  // turn right
  behavior = 'R';
  wait(1);
  turn_round(0);
  // ****** arrive mission 2 ****** //
  
  // ****** mission 2 start ****** //
  // go straight 
  wait(2);
  behavior = 'S';
  car.goStraight(100);
  wait(2.1);
  car.stop();
  // turn right
  wait(1);
  behavior = 'R';
  turn_round(0);
  // check ping data
  wait(3);
  identify_ping_data();
  // turn back right
  behavior = 'B';
  wait(1);
  car.turn(-100,-0.3);
  wait(2.3);
  car.stop();
  // go straight
  wait(1);
  behavior = 'S';
  ping_guide(30, 100);
  // ****** finish mission 2 ****** //
  
  // ****** go to exit ****** //
  // turn right
  wait(1);
  behavior = 'R';
  turn_round(0);
  // go straight to the End
  wait(1);
  behavior = 'S';
  car.goStraight(100);
  // ****** finish all missions and exit the maze ****** //
  
}
