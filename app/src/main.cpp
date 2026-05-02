#include <QCoreApplication>
#include "scan_tracking/app/console_runtime.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/common/config_manager.h"

#ifdef _WIN32
#include <windows.h>
#endif

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    QCoreApplication app(argc, argv);
    app.setApplicationName("Scan Tracking");
    app.setOrganizationName("ScanTracking");

    scan_tracking::common::Logger::initialize();
    scan_tracking::common::ConfigManager::initialize();

    scan_tracking::app::ConsoleRuntime runtime(app);
    const int exit_code = runtime.run();

    scan_tracking::common::ConfigManager::cleanup();
    scan_tracking::common::Logger::cleanup();
    return exit_code;
}
