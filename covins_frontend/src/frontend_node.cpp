
#include "frontend_wrapper.hpp"


int main(int argc, char* argv[]) {
    covins::FrontendWrapper frontend;
    frontend.Init("config.yaml");  // optional init function
    // call frontend.ProcessFrame(...) externally
    return 0;
}
