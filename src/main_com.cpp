#include <Arduino.h>
#include <car/com/objects/text.h>
#include <car/com/mc/interface.h>
#include <car/com/mc/cycle_rate.h>

#define PRINT_FLOAT4(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*10000.)))
#define PRINT_FLOAT2(x) (x<0?"-":" "), ((int) abs(x)), ((int) (abs((x-(int) x)*100.)))

car::com::mc::Interface msg_tx;   				/// object to hande the serial communication
car::com::mc::Interface msg_rx;   				/// object to hande the serial communication
car::com::mc::CycleRate cycle_rate(1000); /// object for a constant cycle control
car::com::objects::Text text;                   /// object to send
car::com::objects::CmdRaw car_target;           /// control target falues
car::com::objects::State  car_state;            /// control target falues
unsigned int loop_count;

void setup() {
    Serial.begin ( 115200 );			/// init serial
    msg_rx.try_sync();   			    /// blocks until a sync message arrives
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

  if(cycle_rate.passed() > 0){
        msg_tx.reset();				/// removes all objects in message
        if(!text.empty()) {
        msg_tx.push_object ( car::com::objects::Object (text, car::com::objects::TYPE_TEXT ) );
        }
        msg_tx.push_object ( car::com::objects::Object (car_target, car::com::objects::TYPE_COMMAND_RAW ) );
        msg_tx.push_object ( car::com::objects::Object (car_state, car::com::objects::TYPE_STATE_RAW ) );
        msg_tx.send();				        /// sends the message
    }
    if ( msg_rx.receive() ) {			/// check for messages
        static car::com::objects::Object object;
        while ( msg_rx.pop_object ( object ).isValid() ) {
            switch ( object.type ) {
            case car::com::objects::TYPE_SYNC: 	/// case sync object
                car::com::objects::Time::compute_offset ( msg_rx.stamp ); /// set clock
                break;
            case car::com::objects::TYPE_COMMAND_RAW: {
                static car::com::objects::CmdRaw o;
                object.get ( car_target );  
                }
            case car::com::objects::TYPE_STATE_RAW: {
                static car::com::objects::CmdRaw o;
                object.get ( car_target );  
                }
                break;
            default:/// case unkown type
                text.write ( "Unknown type received" );
                continue;
            }
        }
    }

    delay ( 10 );
    loop_count++;
}