# Standard message for plugin/plugout request incoming from another vehicle

string rqst    # Request: specify the operation to be requested (plugin or plugout)
string[] type  # Type: specify the particular constraint to be requested (anticollision or proximity)
float64[] data # Data: specify the constraint values (minimum or maximum distances)
int16 id_dest 
Info_msg info  # Info: information about id, state and reference of the vehicle requesting the operation

