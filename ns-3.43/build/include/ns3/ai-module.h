#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_AI
    // Module headers: 
    #include <ns3/ns3-ai-msg-interface.h>
    #include <ns3/ns3-ai-gym-interface.h>
    #include <ns3/ns3-ai-gym-env.h>
    #include <ns3/container.h>
    #include <ns3/spaces.h>
#endif 