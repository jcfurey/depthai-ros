#pragma once

// message_filters ships .hpp headers from jazzy on and drops the .h
// compatibility headers in lyrical; humble has only the .h variants.
#if __has_include("message_filters/subscriber.hpp")
    #include "message_filters/subscriber.hpp"
    #include "message_filters/sync_policies/approximate_time.hpp"
    #include "message_filters/synchronizer.hpp"
#else
    #include "message_filters/subscriber.h"
    #include "message_filters/sync_policies/approximate_time.h"
    #include "message_filters/synchronizer.h"
#endif
