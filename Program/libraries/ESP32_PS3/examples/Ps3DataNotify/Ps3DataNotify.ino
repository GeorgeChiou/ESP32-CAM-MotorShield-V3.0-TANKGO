#include <Ps3Controller.h>

void notify()
{
    if( Ps3.data.button.cross ){
        Serial.println("Pressing the cross button");
    }

    if( Ps3.data.button.square ){
        Serial.println("Pressing the square button");
    }

    if( Ps3.data.button.triangle ){
        Serial.println("Pressing the triangle button");
    }

    if( Ps3.data.button.circle ){
        Serial.println("Pressing the circle button");
    }
}

void onConnect(){
  Serial.println("Connected!.");
}

void setup()
{
    Serial.begin(115200);
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("01:02:03:04:05:06");
    Serial.println("Ready.");
}

void loop()
{
}
