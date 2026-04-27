#include "sys.hpp"

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace {

LogDashboardServer* g_server = nullptr;
constexpr const char* kProjectWebLockFileName = "project_web.lock";

class SingleInstanceLock {
public:
    explicit SingleInstanceLock(const std::string& lock_path)
        : lock_path_(lock_path) {
        const int fd = ::open(lock_path_.c_str(), O_CREAT | O_RDWR, 0644);
        if (fd < 0) {
            throw std::runtime_error(
                "failed to open lock file " + lock_path_ + ": " + std::strerror(errno));
        }

        if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            const std::string message =
                "another project web instance is already running (lock: " + lock_path_ + ")";
            ::close(fd);
            throw std::runtime_error(message);
        }

        fd_ = fd;
        const std::string pid_text = std::to_string(::getpid()) + "\n";
        ::ftruncate(fd_, 0);
        ::lseek(fd_, 0, SEEK_SET);
        const ssize_t ignored = ::write(fd_, pid_text.c_str(), pid_text.size());
        (void)ignored;
    }

    ~SingleInstanceLock() {
        if (fd_ >= 0) {
            ::flock(fd_, LOCK_UN);
            ::close(fd_);
        }
    }

    SingleInstanceLock(const SingleInstanceLock&) = delete;
    SingleInstanceLock& operator=(const SingleInstanceLock&) = delete;

private:
    std::string lock_path_;
    int fd_{-1};
};

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

std::filesystem::path executablePath() {
    std::array<char, 4096> buffer{};
    const ssize_t size = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if (size <= 0) {
        return {};
    }
    buffer[static_cast<std::size_t>(size)] = '\0';
    return std::filesystem::path(buffer.data());
}

std::filesystem::path resolveProjectRoot() {
    if (const char* env_root = std::getenv("PROJECT_ROOT")) {
        if (*env_root != '\0') {
            return std::filesystem::path(env_root);
        }
    }

    std::filesystem::path current = executablePath();
    if (current.empty()) {
        return {};
    }

    current = current.parent_path();
    while (!current.empty()) {
        if (std::filesystem::exists(current / "package.xml") &&
            std::filesystem::exists(current / "CMakeLists.txt")) {
            return current;
        }
        const auto parent = current.parent_path();
        if (parent == current) {
            break;
        }
        current = parent;
    }

    return {};
}

std::string resolveProjectWebLockPath() {
    std::error_code ec;
    const std::filesystem::path project_root = resolveProjectRoot();
    if (!project_root.empty()) {
        const auto lock_dir = project_root / "log" / "runtime" / "locks";
        std::filesystem::create_directories(lock_dir, ec);
        if (!ec) {
            return (lock_dir / kProjectWebLockFileName).string();
        }
    }
    return (std::filesystem::temp_directory_path() / kProjectWebLockFileName).string();
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

    std::unique_ptr<SingleInstanceLock> web_instance_lock;
    try {
        web_instance_lock = std::make_unique<SingleInstanceLock>(resolveProjectWebLockPath());
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }

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
