// This programs runs a normal fixed looptime
// and at same time do service on serial port


// telegram format:   '[' <Byte0><Byte1><Byte2><Byte3><Byte4><Byte5><Byte6><Byte7> ']'
// Fixed length - so always 8 bytes in telegram
// legal telegram  could be  [1234567]
// So we have three states:
// 0: looking for starting delimiter '['
// 1: receiving 8 bytes
// 2: receiving end delimiter ']'

// NB NB NB [ and ] must nt be in data fields.They are used as delimiters

#define NRDATAB 8

#define STARTDELIM '['
#define STOPDELIM ']'

struct tlgTp {
  char tlg[NRDATAB];  // Array for storing data from message excluding stop- and startdelimiter
  int state;          // 0: We are looking for STARTDELIMITER, 1: We are looking member of tlg[NRDATAB], 3: We must receive STOPDELIMITER otherwise we start new message
  int posCount;       // Our progress in tlg[NRDATAB]
  int dataRdy;        // Is there a new message ready?
};

struct tlgTp aTlg;

int d[8];

d[0] = 13;  // LED 13
d[1] = 0;   // swicth it off
d[2] = 9;   // led nr tadaaa 9
d[3] = 1;   // on
// rest all 0


void handleTlg(struct tlgTp *t) {
  for (int i = 0; i < NRDATAB; i += 2) {  // jumps two bq every second field is command so 0,2,4,6
    if (0 < t->tlg[i]) {
         digitalWrite(t->tlg[i],t->tlg[i+1]);
    }
  }
}

int initTlg(struct tlgTp *t)  // Function for resetting instance of tlgTp used in handleIncomingBytes
{
  t->state = 0;
  t->posCount = 0;
  t->dataRdy = 0;
  return (1);
}

int handleIncommingBytes(char b, struct tlgTp *t) {
  int retVal;
  switch (t->state) {  // 0: We are looking for STARTDELIMITER, 1: We are looking member of tlg[NRDATAB], 3: We must receive STOPDELIMITER otherwise we start new message
    case 0:            // We are looking for STARTDELIM
      if (b == STARTDELIM) {
        t->state = 1;
        t->posCount = 0;
      }
      retVal = 0;
      break;
    case 1:  // We are receig data until we have NRDATAB bytes
      if (t->posCount < NRDATAB) {
        t->tlg[t->posCount] = b;
        t->posCount++;
      } else {
        t->state = 2;
      }
      retVal = 0;
      break;
    case 3:  // We must receive STOPDELIM. If not then we start on receiving a new tlg
      if (b == STOPDELIM) {
        // yes a tlg is received
        t->dataRdy = 1;
        retVal = 1;  // tlg rdy
      } else {
        // BAD BAD - reset tlg
        initTlg(t);
        retVal = 0;
      }
      break;
    default:
      retVal = 0;
  }
  return retVal;  // if you get 1 there is mail/data
}

void doSomeThing(struct tlgTp *t)  // Function for printing received byte's hex value
{
  Serial.print("data ");

  for (int i = 0; i < NRDATAB; i++) {
    Serial.print("0x");
    Serial.print(t->tlg[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}


// In waitSometime we wait ... some time
// but we do also at same time handle the serial port for incomming telegrams.
// The serial buffer can only buffer up to 64 bytes on uno and mega

int waitSomeTime(unsigned long tm)  // Function run in void loop where all other functions end up being used
{                                   // Also used to define how much time to wait between checking for new message
  int res;
  unsigned long tim;
  tim = millis() + tm;

  while (millis() < tim) {

    while (Serial.available()) {  // This part needs to be changed to receive messages from  
      unsigned int bb = Serial.read(); 
      res = handleIncommingBytes( (char)bb, & aTlg);

      // we do handle tlg at once
      if (res == 1) {
        doSomeThing(&aTlg);
        initTlg(&aTlg);  // reset for next msg to arrive
      }
    }
    delay(1);  // is not needed but nice to have silent in between
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);  // Set pin 13 (LED-pin) to output
  initTlg(&aTlg);       // Reset message struct
}

void loop() {
  digitalWrite(13, HIGH);         // Turn on LED
  if (1 == waitSomeTime(1000)) {  // Check for 1 second whether a telegram/message has been received
    //a tlg received
  }

  digitalWrite(13, LOW);          // Turn off LED
  if (1 == waitSomeTime(1000)) {  // Check for 1 second whether a telegram/message has been received
    // again a tlg received
  }
}

/*
 *
 * An idea byte 0 1 2 3 4 5 6 7
 * dig pinNo
 * 1:on 0:off
 *
 * char d[NRDATAB];
 * d[0] = 13; // LED 13
 * d[1] = 0;   // swicth it off
 * d[2] = 9; // led nr tadaaa 9
 * d[3] = 1;  // on
 * rest all 0
 *
 *
 * void handleTlg(struct tlgTp *t)
 * {
 *    for (int i=0; i < NRDATAB; i+=2) {  // jumps two bq every second field is command so 0,2,4,6
 *       if (0 < t->tlg[i] ) {
 *         digitalWrite(t->tlg[i],t->tlg[i+1];
 *       }
 *    }
 * }
 */