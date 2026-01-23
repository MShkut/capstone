#ifdef USE_DEV_CONFIG
    #include "custom/dev_config.h"
#endif

// Add myrobot here
#ifdef USE_MYROBOT_CONFIG
    #include "custom/myrobot_config.h"
#endif

// this should be the last one
#ifndef LINO_BASE
    #include "lino_base_config.h"
#endif
