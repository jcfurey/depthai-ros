#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
int main() {
    depthai_bridge::DisparityConverter converter("optical", 100, 10, 10, 1000);
    return converter.getBaseline() > 0 ? 0 : 1;
}
