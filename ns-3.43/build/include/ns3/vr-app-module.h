#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_VR_APP
    // Module headers: 
    #include <ns3/burst-generator.h>
    #include <ns3/burst-sink.h>
    #include <ns3/bursty-application.h>
    #include <ns3/my-random-variable-stream.h>
    #include <ns3/seq-ts-size-frag-header.h>
    #include <ns3/simple-burst-generator.h>
    #include <ns3/trace-file-burst-generator.h>
    #include <ns3/vr-burst-generator.h>
    #include <ns3/bursty-helper.h>
    #include <ns3/burst-sink-helper.h>
    #include <ns3/bursty-app-stats-calculator.h>
#endif 