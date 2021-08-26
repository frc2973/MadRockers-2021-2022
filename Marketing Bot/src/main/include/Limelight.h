#include <string>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

/**
 * Custom class to simplify the Limelight interface.
 */
class Limelight {
    public:
        Limelight() {};

        /**
         * Retrieve the limelight table in case there is a command not implemented in this class.
         * 
         * @return The limelight table.
         */
        std::shared_ptr<NetworkTable> get_table() {
            return nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        }

        /**
         * Retrieve a table value.
         * 
         * @param variable The variable to retrieve.
         * @param default_value An optional parameter for specifying a default value.
         * @return The value of the variable.
         */
        double get(std::string variable, double default_value = 0) {
            return get_table()->GetNumber(variable, default_value);
        }

        /**
         * Set a table value.
         * 
         * @param variable The variable to set.
         * @param value The value to set the variable to.
         */
        void set(std::string variable, int value) {
            get_table()->PutNumber(variable, value);
        }
};