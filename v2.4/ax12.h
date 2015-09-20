/*
  ax12.h - arbotiX Library for AX-12 Servos
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.
  Modificada el 15/11/09 por Pablo Gindel.
  versión 2.0 - 10/02/10
  versión 2.1 - 27/06/10
  versión 2.2 - 19/10/10
	version 2.31 - 2/3/11
*/

// to do: defines para los errores

#ifndef _AX12_H
#define _AX12_H

#define AX12_MAX_SERVOS          18
#define AX12_BUFFER_SIZE         32

/** EEPROM AREA **/
#define MODEL_NUMBER             0
#define VERSION                  2
#define ID                       3
#define BAUD_RATE                4
#define RETURN_DELAY_TIME        5
#define CW_ANGLE_LIMIT           6
#define CCW_ANGLE_LIMIT          8
#define LIMIT_TEMPERATURE        11
#define DOWN_LIMIT_VOLTAGE       12
#define UP_LIMIT_VOLTAGE         13
#define MAX_TORQUE               14
#define STATUS_RETURN_LEVEL      16
#define ALARM_LED                17
#define ALARM_SHUTDOWN           18
#define DOWN_CALIBRATION         20
#define UP_CALIBRATION           22

/** RAM AREA **/
#define TORQUE_ENABLE            24
#define LED                      25
#define CW_COMPLIANCE_MARGIN     26
#define CCW_COMPLIANCE_MARGIN    27
#define CW_COMPLIANCE_SLOPE      28
#define CCW_COMPLIANCE_SLOPE     29
#define GOAL_POSITION            30
#define MOVING_SPEED             32
#define TORQUE_LIMIT             34
#define PRESENT_POSITION         36
#define PRESENT_SPEED            38
#define PRESENT_LOAD             40
#define PRESENT_VOLTAGE          42
#define PRESENT_TEMPERATURE      43
#define REGISTERED_INSTRUCTION   44
#define MOVING                   46
#define LOCK                     47
#define PUNCH                    48

/** Status Return Levels **/
#define RETURN_NONE              0
#define RETURN_READ              1
#define RETURN_ALL               2

/** Instruction Set **/
#define AX_PING                  1
#define READ_DATA                2
#define WRITE_DATA               3
#define REG_WRITE                4
#define ACTION                   5
#define RESET                    6
#define SYNC_WRITE               131

/** Special IDs **/
#define BROADCAST_ID             254


typedef unsigned char byte;

typedef struct {int error; byte *data;} AX12data;
typedef struct {int error; int value;} AX12info;

class AX12
{
  public:

    AX12 (long baud, byte motor_id, bool inv);
    AX12 (byte motor_id, bool inv);
    AX12 (long baud, byte motor_id);
    AX12 (byte motor_id);
    AX12 ();

    byte id;
    bool inverse;
    byte SRL;                                        // status return level

    // el buffer no puede ser privado porque debe ser visible desde la ISR
    static volatile byte ax_rx_buffer[AX12_BUFFER_SIZE];              // buffer de recepción
    static volatile byte ax_rx_Pointer;                           // making these volatile keeps the compiler from optimizing loops of available()

    static void init (long baud);
    static byte autoDetect (byte* list_motors, byte num_motors);
    static void syncWrite (byte start, byte length, byte targetlength, byte* targets, byte** valuess);
    static void syncInfo (byte registr, byte targetlength, byte* targets, int* values);
    static void setMultiPosVel (byte targetlength, byte* targets, int* posvalues, int* velvalues);

    int ping ();
    int reset ();
    AX12data readData (byte start, byte length);
    int writeData (byte start, byte length, byte* values, bool isReg = false);
    int action ();
    AX12info readInfo (byte registr);
    int writeInfo (byte registr, int value, bool isReg = false);
    void setEndlessTurnMode (bool endless);
    void endlessTurn (int velocidad);
    int presentPSL (int* PSL);
    void setSRL (byte _srl);
    byte changeID (byte newID);
    int setPosVel (int pos, int vel);
    
  private:

    static void setTX ();
    static void setRX ();
    static void setNone ();
    static byte writeByte (byte data);
    static void sendPacket (byte _id, byte datalength, byte instruction, byte* data);
    static byte readPacket ();
    AX12data returnData (byte _srl);
    void processValue (byte registr, int* value); 

};

// utils

bool sign2bin (int numero);
char bin2sign (bool var);

/**  Macros  **/

#define setPos(pos) writeInfo (GOAL_POSITION, pos)
#define regPos(pos) writeInfo (GOAL_POSITION, pos, true)
#define setVel(vel) writeInfo (MOVING_SPEED, vel)
#define setTorque(torque) writeInfo (TORQUE_LIMIT, torque)
#define setMultiPos(a, b, c) syncInfo (GOAL_POSITION, a, b, c)
#define setMultiVel(a, b, c) syncInfo (MOVING_SPEED, a, b, c)
#define setMultiTorque(a, b, c) syncInfo (TORQUE_LIMIT, a, b, c)
#define torqueOn writeInfo (TORQUE_ENABLE, 1)
#define torqueOff writeInfo (TORQUE_ENABLE, 0)
#define getPos() readInfo (PRESENT_POSITION).value
#define getSpeed() readInfo (PRESENT_SPEED).value
#define getLoad() readInfo (PRESENT_LOAD).value


#endif
