#ifndef __HM_WEBIF__
#define __HM_WEBIF__

#include <WebServer.h>

class HM_WEBIF {
    private:
        WebServer apserver;

    public:
        HM_WEBIF();
        void handleRoot();
        void handleSetWiFi();
        void handleReSetWiFi();
        void setupAPMode();
        void setupWebIF();
        void handleExit();
};

#endif
