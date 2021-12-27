# Sport-IOT-Device-Prototype
Project made by Master 1 Industrial engineering students at ECAM. Feather M0 LORA board which reads SPO2, Heartrate, temperature and detects falls. All those data is sent via LORA to a TTN gateway and shown on a Node Red dashboard



# Node-Red Configuration 
The Node Red server allows us to display data transmitted from the sensors via a connection to the Mqtt broker. Configurated Nodes allow us to decode the data and retransmit it to a Dashboard where they are viewed in forms of a graph or text.
For the connection to the broker Mqtt, we use the node Mqtt-In, we set the server of the thing network and the subscription of topics. The recovered values are decoded from base 64 and they are returned in jason. The decoded values are extracted via nodes change and reset under new variables. These values are then transmitted to graphic and text nodes on the interface.  
