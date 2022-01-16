#include <string>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

/**
 * Retrieve a value from the limelight table.
 * 
 * @param variable The variable to retrieve.
 * @param default_value An optional parameter for specifying the defualt value.
 * @return The value of the variable.
 */
double limelight_get(std::string variable, double default_value = 0.0) {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber(variable, default_value);
}

/**
 * Set a value in the limelight table.
 * 
 * @param variable The variable to set.
 * @param value The value to set the variable to.
 */
void limelight_set(std::string variable, double value) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(variable, value);
}

/* Reference:

Config Panel:
http://limelight.local:5801/
http://10.29.73.11:5801

Camera Stream:
http://10.29.73.11:5800

*/