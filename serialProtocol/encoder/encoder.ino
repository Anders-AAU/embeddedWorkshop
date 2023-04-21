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
  char tlg[NRDATAB]; // Array for storing data from message excluding stop- and startdelimiter
  int state; // 0: We are looking for STARTDELIMITER, 1: We are looking member of tlg[NRDATAB], 3: We must receive STOPDELIMITER otherwise we start new message
  int posCount; // Our progress in tlg[NRDATAB]
  int dataRdy; // Is there a new message ready?
};

struct tlgTp aTlg;


int initTlg(struct tlgTp * t) // Function for resetting instance of tlgTp used in handleIncomingBytes
{
  t->state = 0;
  t->posCount = 0;
  t->dataRdy = 0;
  return (1);
}

int handleIncommingBytes(char b, struct tlgTp *t)
{
  int retVal;
  switch ( t->state) { // 0: We are looking for STARTDELIMITER, 1: We are looking member of tlg[NRDATAB], 3: We must receive STOPDELIMITER otherwise we start new message
    case 0: // We are looking for STARTDELIM
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
      }
      else {
        t->state = 2;
      }
      retVal = 0;
      break;
    case 3: // We must receive STOPDELIM. If not then we start on receiving a new tlg
      if (b == STOPDELIM) {
        // yes a tlg is received
        t->dataRdy = 1;
        retVal = 1; // tlg rdy
      }
      else {
        // BAD BAD - reset tlg
        initTlg(t);
        retVal = 0;
      }
      break;
    default:
      retVal = 0;
  }
  return retVal; // if you get 1 there is mail/data
}



void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT); // Set pin 13 (LED-pin) to output
  initTlg(&aTlg); // Reset message struct
}

void loop() {
  // Some function in loop
}





























