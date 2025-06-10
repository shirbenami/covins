/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

// Include the header file where the static inline functions are declared
#include <covins/covins_base/config_comm.hpp>
// Also include ros/ros.h if it's not already brought in by config_comm.hpp
// For completeness, explicitly include it here if the header doesn't guarantee it.
#include <ros/ros.h>
#include <iostream> // For std::cout/cerr if not covered by COUTINFO/ERROR macros

namespace covins { // Note: The functions are declared within covins::covins_params namespace in the header.

// The implementations of static inline functions from config_comm.hpp.
// These functions are already implemented in the header itself (as `static inline`).
// Therefore, no separate .cpp definitions are typically required for them,
// as the compiler will generate them in every translation unit that includes the header.
//
// This .cpp file is now largely empty if all functions are static inline and defined in the header.
// If there were non-inline functions declared in the header, their definitions would go here.

// However, if you have a `ShowParamsComm()` that relies on non-static members
// or if you prefer to have a .cpp for consistency, here's how it would look,
// ensuring it calls the functions defined in the header to get values.

// No need to redefine GetPort, GetServerIP, GetCommProtocolType, GetCommSerializationFormat
// here if they are truly `static inline` in the header and retrieve from ROS params.

// Only provide definition for ShowParamsComm if it's not static inline in header,
// or if it's explicitly needed here.
// Based on the immersive `config_comm_h`, ShowParamsComm is also static inline.
// So, this .cpp file would theoretically become empty.

// However, if the intent was for these functions to *not* be inline and have a single definition,
// the `static inline` keywords would be removed from the header, and their definitions would be here.
// Given the current errors, the issue is that `frontend_wrapper.cpp` *expects* the functions
// from the immersive `config_comm_h`.

// To ensure all required functions are available, we'll confirm that `config_comm.hpp`
// contains all the `static inline` definitions as shown in the immersive.
// The provided `config_comm.cpp` uses `covins_params::sys::port` which indicates
// a different setup. This `config_comm.cpp` needs to be replaced to align.

// If `ShowParamsComm` was to be defined in .cpp (not static inline in header):
/*
namespace covins_params { // This is how it's defined in the user's provided .cpp

void ShowParamsComm() {
    // This implementation needs to use the functions defined in the config_comm.hpp header
    // to get the parameter values, which in turn use ros::param::get().
    COUTINFO << "++++++++++ System ++++++++++" << std::endl;
    COUTINFO << "server_ip: " << GetServerIP() << std::endl; // Calls the function from header
    COUTINFO << "port: " << GetPort() << std::endl;           // Calls the function from header
    COUTINFO << std::endl;
    COUTINFO << "++++++++++ Communication ++++++++++" << std::endl;
    // Assuming these parameters exist in ROS param server or are handled by GetCommProtocolType/SerializationFormat
    COUTINFO << "protocol_type: " << GetCommProtocolType() << std::endl;
    COUTINFO << "serialization_format: " << GetCommSerializationFormat() << std::endl;
    // Removed old params like send_updates, data_to_client etc. as they are not
    // part of the GetX methods in the new config_comm.hpp.
    // If these parameters are still needed, they should be added to the config_comm.hpp.
    COUTINFO << std::endl;
}

} // namespace covins_params
*/

// Since all functions in the immersive `config_comm_h` are `static inline`,
// this `config_comm.cpp` can effectively be empty, or entirely removed if CMake
// allows. However, to explicitly define `covins_params::ShowParamsComm`
// for clearer logging and to ensure the function actually gets linked if
// it's not `static inline` in the header, we will provide its definition here.
// But first, let's explicitly include the header it corresponds to.

// Let's assume ShowParamsComm will be defined here, and not static inline in header.
// This requires `config_comm.hpp` to *declare* it without `inline`.

// Given that the immersive `config_comm_h` defines `ShowParamsComm` as `static inline`,
// this `.cpp` file for `config_comm` should actually be empty, as the compiler
// handles the `static inline` definitions.

// If you encountered linker errors (undefined reference to `ShowParamsComm`),
// it would mean `static inline` was not sufficient or not properly applied.
// For now, based on the `config_comm_h` immersive, this file is not strictly necessary.

// I will keep it empty as the `static inline` declarations in the header mean no
// further definition is needed here. If the build fails with linker errors for these,
// then the `static inline` needs to be removed from the header and the definitions
// moved here.

// For now, we will assume `config_comm.hpp` provides all definitions as static inline.

} // namespace covins
