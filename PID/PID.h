#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.0.0

class PID
{
  public:

  //Constants used in some of the functions below
  #define AUTOMATIC 1
  #define MANUAL    0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions ********************************************
    PID(uint16_t*, uint16_t*, uint16_t*, 
        uint16_t, uint16_t, uint16_t, uint8_t, int);     
    /* Constructor *****************************************************
     *   Intake vars:
     *      Input (current reading from the gauge)
     *      Output (value to be set on the controller)
     *      Set Point (target to achieve)
     *      KP (proportional: initial tuning parameter)
     *      KI (integrational: initial tuning parameter)
     *      KD (differential: initial tuning parameter)
     *      Multiplier (Multiplier for final output) 
     *      Direction (DIRECT for positive loop, and REVERSE for negative loop)
     */

    // * sets PID to either Manual (0) or Auto (non-0)
    void SetMode(int Mode);

    // * performs the PID calculation.  it should be called every
    //   time loop() cycles. ON/OFF and calculation frequency can
    //   be set using SetMode/SetSampleTime respectively.
    bool Compute();

    // it's likely the user will want to change this depending on
    // the application
    void SetOutputLimits(uint16_t, uint16_t);

    void SetMultiplier(uint8_t);

  //available but not commonly used functions **************************
    void SetTunings(uint16_t, uint16_t,       // * While most users will set the tunings once in the 
                    uint16_t);              //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetControllerDirection(int);     // * Sets the Direction, or "Action" of the controller. DIRECT
                                          //   means the output will increase when error is positive. REVERSE
                                          //   means the opposite.  it's very unlikely that this will be needed
                                          //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
                                          
                                          
                                          
  //Display functions ****************************************************************
    int32_t GetKp();                       // These functions query the pid for interal values.
    int32_t GetKi();                       //  they were created mainly for the pid front-end,
    int32_t GetKd();                       // where it's important to know what is actually 
    uint8_t GetMode();                        //  inside the PID.
    uint8_t GetDirection();                   //

  private:
    void Initialize();
    
    int16_t kp;                  // * (P)roportional Tuning Parameter
    int16_t ki;                  // * (I)ntegral Tuning Parameter
    int16_t kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    uint16_t *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    uint16_t *myOutput;             //   This creates a hard link between the variables and the 
    uint16_t *mySetpoint;           //   PID, freeing the user from having to constantly tell us
              
    unsigned long lastTime;
    double ITerm;
    uint16_t lastInput;

    unsigned long SampleTime;
    uint16_t outMin, outMax;
    bool inAuto;
    uint8_t multiplier;
};

#endif

