
#include <dynamixel/DynamixelAx.h>

dxlAx dxlCom(&hdSerial2);

int _id = 42;                // Default Dynamixel servo ID
bool c= 0;

void printServoId(String msg);
void printDxlResult();
void printDxlError(unsigned short dxlError);

void setup() {

    // Open serial communications and wait for port to open (PC communication)
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    Serial.println("Starting COM!");

    dxlCom.begin(9600);

    dxlCom.setId(254, 42);

    dxlCom.setStatusReturnLevel(42, 2);

    dxlCom.setReturnDelayTime(42, 250);

    dxlCom.setMovingSpeed(42, 512);

    dxlCom.setMaxTorque(42, 512);

    dxlCom.setGoalPosition(42, 512);

    Serial.println("Init ok");

}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    printServoId("Ping");
    c = !c;
    dxlCom.setLedEnable(_id,c);
    dxlCom.ping(42);
    printDxlResult();

}

/////////////////////////////////////////////////////////////////////////////////////

void printDxlResult()
{
    while(!dxlCom.dxlDataReady());        // waiting the answer of servo
    printDxlError(dxlCom.readDxlError());
    Serial.println(dxlCom.readDxlResult());
}

void printServoId(String msg)
{
    Serial.print(msg);
    Serial.print(" servo ID ");
    Serial.print(_id);
    Serial.print(" - ");
}

void printDxlError(unsigned short dxlError)
{
    // after any operation error can be retrieve using dx::readDxlResult() (i.e. after read or write operation)
    if(dxlError == DXL_ERR_SUCCESS)
        Serial.println("OK");
    else
    {
        if (dxlError & DXL_ERR_VOLTAGE)
            Serial.print("voltage out of range-");
        if (dxlError & DXL_ERR_ANGLE)
            Serial.print("angle out of range-");
        if (dxlError & DXL_ERR_OVERHEATING)
            Serial.print("overheating-");
        if (dxlError & DXL_ERR_RANGE)
            Serial.print("cmd out of range-");
        if (dxlError & DXL_ERR_TX_CHECKSUM)
            Serial.print("Tx CRC invalid-");
        if (dxlError & DXL_ERR_OVERLOAD )
            Serial.print("overload-");
        if (dxlError & DXL_ERR_INSTRUCTION )
            Serial.print("undefined instruction-");
        if (dxlError & DXL_ERR_TX_FAIL )
            Serial.print("Tx No header-");
        if (dxlError & DXL_ERR_RX_FAIL )
            Serial.print("Rx No header-");
        if (dxlError & DXL_ERR_TX_ERROR  )
            Serial.print("Tx error-");
        if (dxlError & DXL_ERR_RX_LENGTH   )
            Serial.print("Rx length invalid-");  // Not implemented yet
        if (dxlError & DXL_ERR_RX_TIMEOUT)
            Serial.print("timeout-");
        if (dxlError & DXL_ERR_RX_CORRUPT)
            Serial.print("Rx CRC invalid-");
        if (dxlError & DXL_ERR_ID )
            Serial.print("Wrong ID answered-"); // ?? Hardware issue
        Serial.println();
    }
}