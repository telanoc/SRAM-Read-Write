/*
 * SRAM Read/Write Test Program
 * SRAM Chip Model No.: UM61256AK-15
 * Chip Info: 32K X 8 High Speed CMOS RAM (32,768 bytes MAX)
 * 
 * This program tests the Arduino's ability to interface to older generation SRAM chips.
 * The memory testing algorithm is not meant to be a rigorous test, but rather to show
 * that the Read/Write functions of the program work.
 * 
 * Modified 10 Feb 2024 by Pete Cervasio to perform continuous loop testing using a
 * 32 bit XorShift PRNG (see Marsaglia).  The initial seed is 0xdeadbeef and one of
 * those four bytes is randomly set to a different value thanks to the Arduino random()
 * function.  The code also "spins the wheel" on the PRNG to get the seed to test with.
 * The PRNG is used to fill the RAM's memory, and then the PRNG is restored to the test
 * seed and each byte is validated against the PRNG values.  Any byte that doesn't match
 * is reported.  A total pass and total errors count is reported at the end of each pass
 * through the RAM.  One of the PRNG bytes is again changed on the next loop, as well as
 * a new spin of the wheel, making for a whole new set of values for the next pass.
 * 
 * Also changed: the delay for RAM reduced from 1ms to 2us.
 * 
 */

#define OE_LOW()    digitalWrite(51, 0)   //Sets OE LOW
#define CE_LOW()    digitalWrite(52, 0)   //Sets CE LOW
#define WE_LOW()    digitalWrite(53, 0)   //Sets WE LOW
#define WE_HIGH()   digitalWrite(53, 1)   //Sets WE HIGH

// zero means continuous
#define MAX_PASSES 0

// If defined, an error is forced on pass #4
// #define DO_FAKE_ERROR

// Data for xorshift32 PRNG
struct xorshift32_state {
   union {
      uint32_t a;
      uint8_t b[4];
   } q;
} seed;

/* The state word must be initialized to non-zero */
uint32_t xorshift32(struct xorshift32_state *state)
{   
   /* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
   uint32_t x = state->q.a;
   x ^= x << 13;
   x ^= x >> 17;
   x ^= x << 5;
   return state->q.a = x;
}

void setup()     
{

  Serial.begin(9600); // Initiate serial port data transmission at 9600 baud

  /*
   * Control Line Setup
   * WE = Write Enable is Pin 53
   * OE = Output Enable is Pin 52
   * CE = Chip Enable is Pin 51
   */

   DDRB = 0b00000111; //Pin 53, 52, 51 set to output

   seed.q.a = 0xdeadbeef;  // static non-zero seed initialization 

  /*
   * Address Bus Setup 
   * Maximum number of lines: 15
   */

   DDRA = 0b11111111; //Pins 22-29 Set as Output (Lower Byte of Address)
   DDRC = 0b01111111; //Pins 37-31 Set as Output (Upper Byte of Address)

   //Initialize random number generator using signal from pin AO
   randomSeed(analogRead(A0));

}

int pass = 0;     // How many passes we've done
long errors = 0;  // Count of errors encountered

void loop() 
{

   /*
    * Original Memory Testing Algorithm
    * 1.) Sequence through addresses from 0x0000 to 0x7FFF (0 to 32767)
    * 2.) Write random data to that location
    * 3.) Read back data and confirm it is the same
    * 3a.) If data read back is same, memory test is passed
    * 3b.) If data read back is different, memory test failed
    * 4.) Repeat
    * 
    * New memory test algorithm.  Loop forever, doing:
    * 1). Initialize PRNG with existing seed and one byte from
    *     Arduino random() function.  Save new seed for verify.
    * 2). Fill RAM from 0 to 0x7fff with data from the PRNG
    * 3). Restore PRNG seed
    * 4). Verify each byte of ram against PRNG values.  At the
    *     end of each 4k block, report status so far
    *     If any ram location fails, report it and increment the
    *     error total counter.
    * 5). At the end of the loop, report total passes and errors
    * 6). Go back to step 1.
    */

    uint16_t address;                               //Declare address variable
    uint8_t random_data;                            //Declare random data variable
    char print_str[90];                             //Declare char array used for formatted printing
    uint32_t save_seed;                             //PRNG seed storage for reading/writing
    int local_errors = 0;
    #ifdef DO_FAKE_ERROR
    int fake_flag;
    #endif
    
    // Modify the PRNG for this loop.  Randomly change one of the bytes.
    // Use range 1..255 to make sure we can't accidentally make the seed 
    // zero, which will break the PRNG (see Marsaglia on xorshift)
    seed.q.b[random(0,3)] = random(1, 0xff);

    // Also, randomly spin the wheel before picking a new seed
    for (int i = 0; i < random(10, 100); i++) {
      xorshift32(&seed);
    }

    save_seed = seed.q.a; // Save that for verify pass
    pass++;
    
    sprintf(print_str, "Seeding PRNG with 0x%8.8lx for test pass %d", save_seed, pass);
    Serial.println(print_str);

    // Fill RAM with random bytes
    Serial.print("Filling RAM memory");
    for (address = 0; address < 0x8000; address += 4) // Start at 0, sequence through all (2^15) - 1 addresses
    {
      xorshift32 (&seed);
      write_data(address+0, seed.q.b[0]);           // write PRNG data to address
      write_data(address+1, seed.q.b[1]);           // We're unrolling the inner loop for speed! Yeah!
      write_data(address+2, seed.q.b[2]);           // We're not just lazily cutting and pasting, no way!
      write_data(address+3, seed.q.b[3]);           //
      if ((address & 0xffc) == 0xffc) {
        Serial.print(".");                          // And the end of a 4k block
      }
    }
    Serial.println (" Done");
    Serial.println ("Validating RAM memory");
#ifdef DO_FAKE_ERROR
    fake_flag = (pass == 4);  // Cause error on pass 4, block 1
    if (fake_flag) {
      write_data(0x1234, 0x56);
      Serial.println(".. Faking error. Byte at 0x1234 set to 0x56");
    }
#endif   
    
    // Now verify those bytes.  Step 4 at a time because the PRNG
    // gives us 4 bytes and we cycle those inside this loop
    seed.q.a = save_seed;
    int byte_count = 0;   // good bytes counter
    for (address = 0; address < 0x8000; address += 4) {
      xorshift32 (&seed);   // get next PRNG value
      for (int i = 0; i < 4; i++) {
        uint8_t got = read_data(address + i);  // Find out what's in RAM's pocketses
        if( got == seed.q.b[i]) {              // Is it the Precious?
          byte_count++;
          if (((address & 0xffc) == 0xffc) && (i == 3)) {  // at the end of the 4k block
            sprintf(print_str, "Block: 0x%04X  Bytes verified: %d  Status: ", (address&0xf000), byte_count);
            Serial.print(print_str);
            Serial.println((local_errors == 0) ? "PASS" : "FAIL");  // Status of the block
            byte_count = local_errors = 0;        
#ifdef DO_FAKE_ERROR
            if ((fake_flag != 0) && (local_errors > 0)) {
              fake_flag = 0;
            }
#endif          
          }
        } 
        else {
          // If data read from address is NOT equal to random data generated, report it
          errors++;       // Total errors increase
          local_errors++; // Errors in this block increase
          sprintf(print_str, "Address: 0x%04X Expected: %02X Got: %02X Status: FAIL! Count=%ld", address+i,seed.q.b[i], got, errors);
          Serial.println(print_str);
          Serial.flush();
          delay(5000);    // Pause to let someone see it
        }
      }
    }

    Serial.println();
    sprintf(print_str, "Completed pass %d.  Total errors: %ld\n", pass, errors);
    Serial.println(print_str);
    
    delay(500);    //Delay 1500 milliseconds to ensure all data is properly displayed from Serial Monitor
#ifdef DO_FAKE_ERROR
    while (errors > 0) {
      delay(1);
    }
#endif

  if ((MAX_PASSES > 0) && (pass == MAX_PASSES)) {
    Serial.print("=== Max passes reached.  Stopping ===");
    while (1);
  }

}

/*
 * Write Data Function
 * Arguments: Address and Data to be written
 */

void write_data(uint16_t address, uint8_t data)
{
  /*
   * Write operation uses a WE (Write Enabled Controlled) Write Cycle.
   */
  OE_LOW();             //OE is continuously LOW
  CE_LOW();             //CE is continuously LOW
  WE_HIGH();            //WE starts off HIGH
  
  //delay(1);           //Delay just to ensure signals stay HIGH/LOW long enough
  delayMicroseconds(2); // Static ram is pretty fast
  
  set_addr(address);    //Address applied first
  WE_LOW();             //WE goes from HIGH to LOW
  data_op('w', data);   //Data applied to data bus
  WE_HIGH();            //WE goes from LOW to HIGH

}

/*
 * Read Data Function
 * Arguments: Address to be used to obtain data
 */

uint8_t read_data(uint16_t address)
{
  /*
   * Read Operation uses "Read Cycle 1", see PDF documentation linked
   */
  WE_HIGH();     //WE set to HIGH at all times for Type 1 Read
  OE_LOW();      //OE set to LOW at all times for Type 1 Read
  CE_LOW();      //CE set to LOW at all times for Type 1 Read
  
  set_addr(address);        //Address applied first
  return (data_op('r', NULL));  //Read data operation is committed, data obtained is returned
  
}

/*
 * Set Address Function, Outputs the address through two pin registers
 * Arguments: Address to be outputted
 */

void set_addr (uint16_t address)
{

  PORTA = address & 0xff;         //Takes the first half of the address and sets it to PORTA (Lower Address Byte)
  PORTC = (address >> 8) & 0xff;  //Takes the second half of the address and sets it to PORTC (Upper Address Byte)
  
}

/*
 * Data Operation Function, controls the data IO from the Arduino
 * Arguments: Read/Write option and data to be written
 */

uint8_t data_op (char rw, uint8_t data)
{
  if(rw == 'w') 
  {
    /*
     * If RW option is set to char 'w' for WRITE
     */
    DDRL = 0xff;   //Set Data Direction on L register pins to OUTPUT
    PORTL = data;  //Output Data through L pin data register
    return PORTL;  //Return the data outputted as confirmation
  }
  else if (rw == 'r')
  {
   /*
    * If RW option is set to char 'r' for READ, read data
    */
    PORTL = 0x00;  //Erase any data still being held from previous write operation in L pin data register
    DDRL = 0x00;   //Set L register pins to INPUT
    return PINL;   //Return values read in the pin register
  }

}
