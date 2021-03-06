#include <libmaple/timer.h>

#include "module_common.h"
#include "Arduino_Interface.h"
#include <avr/dtostrf.h>
#include "tracker.h"

#include "Arduino.h"

//settings structure
struct settings {

  char apn[20];
  char user[20];
  char pwd[20];
  long interval;          //how often to collect data (milli sec, 600000 - 10 mins)
  int interval_send;      //how many times to collect data before sending (times), sending interval interval*interval_send
  byte powersave;
  char key[12];           //key for connection, will be sent with every data transmission
  char sim_pin[5];        //PIN for SIM card
  char sms_key[12];       //password for SMS commands
  char imei[20];          //IMEI number


};

settings config;

// Variables will change:
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long watchdogMillis = 0;        // will store last time modem watchdog was reset

long time_start, time_stop, time_diff;             //count execution time to trigger interval
int interval_count = 0;         //current interval count (increased on each data collection and reset after sending)


char data_current[DATA_LIMIT];   //data collected in one go, max 2500 chars
int data_index = 0;        //current data index (where last data record stopped)
char time_char[20];             //time attached to every data line
char modem_reply[200];     //data received from modem, max 200 chars
long logindex = STORAGE_DATA_START;
byte save_config = 0;      //flag to save config to flash
byte power_reboot = 0;           //flag to reboot everything (used after new settings have been saved)


unsigned long last_time_gps, last_date_gps;

//  TinyGPS gps;
//  DueFlashStorage dueFlashStorage;

#define PIN_BOLT  D20
#define PIN_CAM   D19
#define PIN_MOTO  D38

#define DELAY_AFTER_CAM_PULLED 10

#define debug_print(x) SerialUSB.println(x)

#define OPENLOCK_STAT_IDLE	0
#define OPENLOCK_STAT_MOTO_STARTED 1
#define OPENLOCK_STAT_CAM_PULLED 2

#define RESP_ERROR		-1
#define RESP_PONG		2
#define RESP_OPENLOCK	3

int moto_timeout_in_10ms = 0;
int open_lock_step = 0;

int time_delay_before_stop_moto_in_10ms = 0;

int need_reconnect = 1;
int wait_for_response = 0;
int data_sent = 0;

int gsm_timer = 0;
int ping_timer = 0;
int last_pong = 0;

int bolt;
int previous_bolt = -1;

int cam;
int preview_cam = -1;

int status_changed = 1;
int moto_initialized = 0;

WioTracker wio = WioTracker();

void setup() {
  stop_moto();
    
  // Enable Module Power
  pinMode(wio.MODULE_PWR_PIN, OUTPUT);
  digitalWrite(wio.MODULE_PWR_PIN , HIGH);
  // Enable VCCB
  pinMode(wio.ENABLE_VCCB_PIN, OUTPUT);
  digitalWrite(wio.ENABLE_VCCB_PIN, HIGH);

  SerialUSB.println("Begin...");

  stop_moto();
  
  wio.Power_On();
  while (false == wio.Check_If_Power_On()) {
    SerialUSB.println("Waitting for module to alive...");
    delay(1000);
  }
  SerialUSB.println("Power On O.K!");
  
  SerialUSB.println("Init Timer2");
  init_timer2();

  pinMode(PIN_BOLT, INPUT_PULLUP);
  pinMode(PIN_CAM, INPUT_PULLUP);
  pinMode(PIN_MOTO, OUTPUT);
  digitalWrite(PIN_MOTO, LOW);

  check_gsm();
}

char cmd;

void loop() {
  int ret_tmp;

  //timer_resume(TIMER2);

  /* Debug */
  if (SerialUSB.available()) {
    // .write(SerialUSB.read());
    cmd = SerialUSB.read();
    if (cmd == '1') {
      stop_moto();
    } else if (cmd == '0') {
      start_moto();
    }
  }

//  if (SerialUSB.available()) {
//    MODULE_PORT.write(SerialUSB.read());
//  }
//    if(MODULE_PORT.available()){
//      SerialUSB.write(MODULE_PORT.read());
//    }

  // if the moto started, don't check the network anymore
  // the network related functions are slow, they will stuck the timer
  if (open_lock_step <= OPENLOCK_STAT_IDLE) {
    if (data_sent) {
      // check recv data via at command every 1s
      if (gsm_timer >= 100) {
        gsm_timer = 0;

        check_gsm();
      }

//	  debug_print("ping_timer");
//	  debug_print(ping_timer);
      // send ping every 1 minutes
      if (ping_timer >= 100 * 60) {
        debug_print("send ping");
        send_ping();
      }

//	  debug_print("last_pong");
//	  debug_print(last_pong);
      // if we didn't get pong after more than 1 minute
      if (last_pong > 100 * 65) {
        debug_print("didn't got pong after 1 minute. reset gsm");
        last_pong = 0;
        need_reconnect = 1;
      }
    } else {
      check_gsm();
    }


    if (status_changed == 1) {
      status_changed = 0;

      send_ping();
    }
  }

//  //  delay(500);

  if (moto_initialized == 0 && ping_timer >= 100 * 1) {
    moto_initialized = 1;
    start_moto();
  }

  timer_resume(TIMER2);
}

void send_ping() {
  ping_timer = 0;
  data_current[0] = '#';
  data_current[1] = 'p';
  data_current[2] = cam == 0 ? '0' : '1';
  data_current[3] = bolt == 0 ? '0' : '1';
  data_current[4] = 0;
  gsm_send_http_current();
}

void check_gsm() {
  int ret_tmp;

  debug_print("check_gsm");

  if (open_lock_step <= OPENLOCK_STAT_IDLE) {
    if (need_reconnect) {
      //send 2 ATs
      gsm_send_at();

      gsm_reconnect();
      gsm_get_imei();
      wait_for_response = 0;
      data_sent = 0;
    } else {
      if (data_sent == 0) {
        gsm_send_http_current();
        data_sent = 1;
      } else {
        ret_tmp = parse_receive_reply();
        if (ret_tmp == -1) {
          need_reconnect = 1;
        } else if (ret_tmp == RESP_OPENLOCK) {
          start_moto();
        } else if (ret_tmp == RESP_PONG) {
          last_pong = 0;
        }
      }
    }
  }
}

void gsm_reconnect() {
  int ret_tmp;

  //disconnect GSM
  ret_tmp = gsm_disconnect();
  if (ret_tmp == 1)
  {
    debug_print(F("GPRS deactivated."));
  }
  else
  {
    debug_print(F("Error deactivating GPRS."));
    gsm_send_at();
  }

  //opening connection
  ret_tmp = gsm_connect();
  if (ret_tmp == 1)
  {
    //connection opened, just send data
    //          gsm_send_http_current();  //send all current data
    //          delay(4000);

    //get reply and parse
    //          ret_tmp = parse_receive_reply();
    need_reconnect = 0;

  }
  else
  {
    debug_print(F("Error, can not send data, no connection."));
    gsm_disconnect();

    delay(1000);
    gsm_get_reply();

    need_reconnect = 1;
  }
}

void start_moto() {
  digitalWrite(PIN_MOTO, HIGH);
  open_lock_step = OPENLOCK_STAT_MOTO_STARTED;
  moto_timeout_in_10ms = 0;
  time_delay_before_stop_moto_in_10ms = 0;
}

void stop_moto() {
  open_lock_step = OPENLOCK_STAT_IDLE;
  moto_timeout_in_10ms = 0;
  time_delay_before_stop_moto_in_10ms = 0;
  digitalWrite(PIN_MOTO, LOW);
}

void yunba_gsm_send_data() {
  MODULE_PORT.write("AT\r\n");
  delay(100);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }

  MODULE_PORT.write("AT+QIOPEN=1,0,\"TCP\",\"yunba.io\",80\r\n");
  delay(3000);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }
  MODULE_PORT.write("AT+QISEND=0,5\r\n");
  delay(500);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }
  MODULE_PORT.write("abcde");
  delay(5000);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }
  MODULE_PORT.write("AT+QIRD=0,1500\r\n");
  delay(300);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }
  MODULE_PORT.write("AT+QICLOSE=0\r\n");
  delay(300);
  while (MODULE_PORT.available()) {
    SerialUSB.write(MODULE_PORT.read());
  }
}

void ledOn() {
  //    SerialUSB.write("on");
  bolt = digitalRead(PIN_BOLT);
  if (bolt != previous_bolt) {
    previous_bolt = bolt;
    bolt == 0 ? SerialUSB.println("bolt 0") : SerialUSB.println("bolt 1");

    status_changed = 1;
  }

  cam = digitalRead(PIN_CAM);
  if (cam != preview_cam) {
    status_changed = 1;

    preview_cam = cam;
    cam == 0 ? SerialUSB.println("cam 0") : SerialUSB.println("cam 1");

    // open lock started, and the cam pulled down
    // then wait extra DELAY_AFTER_CAM_PULLED ms before stop the moto
    if (cam == 0 && open_lock_step == OPENLOCK_STAT_MOTO_STARTED) {
      open_lock_step = OPENLOCK_STAT_CAM_PULLED;
      time_delay_before_stop_moto_in_10ms = 0;
    }
  }

  if (open_lock_step == OPENLOCK_STAT_CAM_PULLED) {
    time_delay_before_stop_moto_in_10ms++;
  }
  moto_timeout_in_10ms++;

  // stop the moto in DELAY_AFTER_CAM_PULLED ms after the cam pulled down
  if (time_delay_before_stop_moto_in_10ms >= DELAY_AFTER_CAM_PULLED) {
    stop_moto();
  }

  // start the moto for 3s atmost
  if (moto_timeout_in_10ms >= 300) {
    stop_moto();
  }

  gsm_timer++;
	ping_timer++;
	last_pong++;
}
void ledOff() {
  //  SerialUSB.write("off");
}

void init_timer2() {
  // Reload in 0.04 seconds of initialization
  timer_pause(TIMER2);
  timer_init(TIMER2);
  timer_set_prescaler(TIMER2, 10000);
  timer_set_reload(TIMER2, 72 * 4);

  // Register interrupt handler to light LED after 1 second
  timer_set_mode(TIMER2, 1, TIMER_OUTPUT_COMPARE);
  timer_set_compare(TIMER2, 1, 72 * 1);
  timer_attach_interrupt(TIMER2, TIMER_CC1_INTERRUPT, ledOn);
  timer_enable_irq(TIMER2, 1);

  // Register interrupt handler to turn off the LED after 2 seconds
  timer_set_mode(TIMER2, 2, TIMER_OUTPUT_COMPARE);
  timer_set_compare(TIMER2, 2, 72 * 2);
  timer_attach_interrupt(TIMER2, TIMER_CC2_INTERRUPT, ledOff);
  timer_enable_irq(TIMER2, 2);

  // Interrupt execution
  TIMER2->regs.adv->CR1 |= TIMER_CR1_OPM; // repeat none specified
  timer_generate_update(TIMER2);
  timer_resume(TIMER2);
}
