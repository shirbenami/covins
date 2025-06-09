//
// Created by user1 on 09/06/25.
//
#include <covins/comm_abstraction/ISerializer.hpp>

namespace covins {

    // Define the virtual destructor for ISerializer.
    ISerializer::~ISerializer() = default;

    // Define the virtual destructor for IDeserializer (if applicable).
    // If IDeserializer is a separate class in the same header and has a virtual destructor.
    IDeserializer::~IDeserializer() = default;

} // namespace covins
