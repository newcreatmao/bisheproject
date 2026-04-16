#include "sys.hpp"

#include <csignal>
#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace {

LogDashboardServer* g_server = nullptr;

std::string envOrDefault(const char* key, const std::string& fallback) {
    if (const char* value = std::getenv(key)) {
        if (*value != '\0') {
            return value;
        }
    }
    return fallback;
}

int envPortOrDefault(const char* key, int fallback) {
    if (const char* value = std::getenv(key)) {
        try {
            return std::stoi(value);
        } catch (...) {
            return fallback;
        }
    }
    return fallback;
}

void onSignal(int) {
    if (g_server != nullptr) {
        g_server->stop();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

}  // namespace

int main(int argc, char ** argv) {
    logger_set_info_enabled(false);
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    LogDashboardServer server;
    g_server = &server;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    const std::string host = envOrDefault("PROJECT_HOST", "0.0.0.0");
    const int port = envPortOrDefault("PROJECT_PORT", 8080);

    const bool ok = server.listen(host, port);
    server.stop();
    g_server = nullptr;
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return ok ? 0 : 1;
}
