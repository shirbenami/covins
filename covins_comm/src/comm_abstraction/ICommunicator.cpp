//
// Created by user1 on 09/06/25.
//
#include <covins/comm_abstraction/ICommunicator.hpp>

namespace covins {

    // Define the virtual destructor.
    // Even if it's pure virtual in the header (e.g., virtual ~ICommunicator() = 0;),
    // you still need to provide an implementation for it.
    ICommunicator::~ICommunicator() = default;

} // namespace covins
