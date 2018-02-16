#include <xamla_sysmon_watch.h>

XamlaSysmonWatch* instance = 0;

extern "C" void start_watch() {
    if (!instance) {
        instance = new XamlaSysmonWatch();
        instance->start();
    }
}

extern "C" void shutdown_watch() {
    if (instance) {
        instance->shutdown();
        delete instance;
        instance = 0;
    }
}