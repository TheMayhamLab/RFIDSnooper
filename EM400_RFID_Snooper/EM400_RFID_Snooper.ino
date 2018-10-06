// Arduino code to decode the RFID "envelope" from a 125Khz EM400 proximity card brought into the field of a 125Khz card reader.

byte buffer[1200];
int bufindex = 0;
int countofsameanswer = 0;
int countofwronganswers = 0;

// one bit is 40-92, two bits is 110-190
int minreadsforonebit = 16;
int maxreadsforonebit = 95;
int minreadsfortwobits = 96;
int maxreadsfortwobits = 172;
int toomanywronganswers = 20;

int gooddatagoing = 0;

byte last = 0;

byte thisread;

void setup() 
{
  DDRB = B11101111; //pinMode(12, INPUT);
  PORTB = B00000000;

  Serial.begin(115200);	// Debugging only
  Serial.println("Starting...\n");
  
}

void loop() 
{
  thisread = (PINB & B00010000);
  if (last ^ thisread)  // 
  {
    if ((countofsameanswer > minreadsforonebit) && (countofsameanswer < maxreadsforonebit))
    {
      // the count indicates that a single bit wide was read
      if (!(gooddatagoing)) // this is the first good bit
      {
        gooddatagoing = 1;
        nextvalidbit(B11111111); // start of good data, add 0xff to the buffer
      }
      nextvalidbit(last); // add whether the bit that just finished was high or low to the buffer
      countofsameanswer = 0;
      last = thisread;
    }
    else if ((countofsameanswer > minreadsfortwobits) && (countofsameanswer < maxreadsfortwobits)) 
    {
      // two bits read
      if (!(gooddatagoing))
      {
        gooddatagoing = 1;
        nextvalidbit(B11111111); // start of good data, add 0x0f to the buffer
      }
      nextvalidbit(last);
      nextvalidbit(last);
      countofsameanswer = 0;
      last = thisread;
    }
    else if ( (countofsameanswer > maxreadsfortwobits) || (countofsameanswer < minreadsfortwobits))  // the count was too big or too small
    {
      // bad data, more than two bits.. but we allow a few mistakes
      countofsameanswer = 0;
      countofwronganswers++;
      last = thisread;
      if (countofwronganswers > toomanywronganswers)
      {
        //Serial.println("Bad data, too many wrong answers");
        countofsameanswer = 0;
        countofwronganswers = 0;
        last = thisread;
        gooddatagoing = 0;
        bufindex = 0;
      }
    }   
  }
  else
  {
    countofsameanswer ++;
  }
}


void nextvalidbit (byte thisbit)
{
  buffer[bufindex] = thisbit;
  bufindex++;
  if ((bufindex > 1000) || (thisbit > B10000000))
  {
    // end of valid bits
    if (bufindex > 127)
    {
      // long enough to be a valid message, test it
      analyzebuffer(bufindex);
    }
    bufindex = 0;
  }
  countofwronganswers = 0; // we clear the wrong answer count when we read a valid bit.
}

void analyzebuffer(int buftotal)
{
  int trackerstart = 0;
  
  while ((buftotal - trackerstart) > 127)
  {
    //Serial.print("About to run Checkfor ValidEM400 starting at tracker ");
    // still long enough to be a valid message
    if ( CheckForValidEM400(buffer, trackerstart) )
    {
      //valid message, skip 128 bits
      trackerstart += 128;
      //Serial.println("Check for Valid EM 400 returned TRUE");
    }
    else
    {
      //invalid message, skip one and try again if still enough bytes left
      trackerstart++;
      //Serial.println("Check for Valid EM 400 returned false");
    }
  }
}  

boolean CheckForValidEM400(byte a[],int startat)
{
  //Serial.print("Checking buffer starting at pos ");
  //Serial.println (startat);
  int i;
  for (i = startat; i < (startat + 18); i = i + 2)
  {
    if ( a[i] || (!a[i+1]))  // anything but 10
    {
      //Serial.println("Too short for preamble, fail");
      return(false);
    }
  }
  if ( (!a[i])  || (a[i+1]))
  {
    //Serial.println("01 not found after nine 10 preamble, fail");
    return (false);  //anything but 01
  }
  // if we made it this far, we have pramble 1111111110
  // skip ten bits to the end of the version
  i = i + 20; 
  byte columnparity[4] = {0,0,0,0};
  byte databits[32];
  int databitpos = 0;
  byte paritybit = 0;
  
  for (int row = 0; row < 8; row++)
  {
    byte rowparity = 0;
    for (int column = 0; column < 4; column++)
    {
      if (a[i] && !a[i+1]) // 01
      {
        databits[databitpos] = 0;
        rowparity = rowparity ^ databits[databitpos];
        columnparity[column] = columnparity[column] ^ databits[databitpos]; 
        databitpos++;
     }
      else if (!a[i] && a[i+1]) // 10
      {
        databits[databitpos] = 1;
        rowparity = rowparity ^ databits[databitpos];
        columnparity[column] = columnparity[column] ^ databits[databitpos]; 
        databitpos++;
      }
      else
      {
        //not 10 or 01, error
        return(false);
      }
      i = i + 2;
    }
    // End of Row, check row parity
    if (a[i] && !a[i+1]) //01 parity bit
    {
      paritybit = 0;
    }
    else if (!a[i] && a[i+1]) //10 parity bit
    {
      paritybit = 1;
    }
    else 
    {
     //Serial.println("row parity bit not manchester, fail");
     return (false);
    }
    
    if (rowparity ^ paritybit)
    {
      //Serial.println("row parity mismatch, fail");
      return(false);
    }
    //row done, parity matches
    i = i + 2;
  }
  // End of all data, check column parity
  for (int j = 0; j < 4; j++)
  {
    if (! a[i+(2*j)] && a[i+(2*j)+1]) //01 parity bit
    {
      paritybit = 0;
    }
    else if (a[i+(2*j)] && !a[i+(2*j)+1]) //10 parity bit
    {
      paritybit = 1;
    }
    else 
    {
      //Serial.println("col parity bit not manchester, fail");
      return (false);
    }
  }
  Serial.println("Success");
  unsigned long tagid = 0;
  tagid = 0;
  for (int pos = 31; pos > -1; pos--)
  {
    tagid = tagid + (databits[31-pos] * twopow(pos));
  }
  Serial.println(tagid);
  
  return(true);
}

unsigned long twopow (int expon)
{
  unsigned long result = 1;
  for (int foo = 0; foo < expon; foo++)
  {
    result = result * 2;
  }
  return (result);
}

