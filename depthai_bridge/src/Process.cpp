#include "depthai_bridge/Process.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

extern char** environ;
namespace depthai_bridge {
std::vector<std::string> splitArguments(const std::string& input) {
    std::vector<std::string> result;
    std::string token;
    char quote = 0;
    bool escaped = false, started = false;
    for(char c : input) {
        if(escaped) {
            token += c;
            escaped = false;
            started = true;
        } else if(c == '\\' && quote != '\'') {
            escaped = true;
            started = true;
        } else if(quote) {
            if(c == quote)
                quote = 0;
            else
                token += c;
        } else if(c == '\'' || c == '"') {
            quote = c;
            started = true;
        } else if(std::isspace(static_cast<unsigned char>(c))) {
            if(started) {
                result.push_back(token);
                token.clear();
                started = false;
            }
        } else {
            token += c;
            started = true;
        }
    }
    if(escaped || quote) throw std::invalid_argument("Unterminated quote or escape in xacro arguments");
    if(started) result.push_back(token);
    return result;
}
std::string runProcess(const std::vector<std::string>& arguments) {
    if(arguments.empty()) throw std::invalid_argument("Empty process command");
    std::vector<char*> argv;
    for(const auto& arg : arguments) {
        if(arg.find('\0') != std::string::npos) throw std::invalid_argument("NUL in process argument");
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    int descriptors[2];
    if(pipe2(descriptors, O_CLOEXEC) != 0) throw std::runtime_error("Cannot create process pipe");
    posix_spawn_file_actions_t actions;
    int error = posix_spawn_file_actions_init(&actions);
    if(error) {
        close(descriptors[0]);
        close(descriptors[1]);
        throw std::runtime_error(std::strerror(error));
    }
    error = posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
    if(!error) error = posix_spawn_file_actions_addclose(&actions, descriptors[0]);
    if(!error) error = posix_spawn_file_actions_addclose(&actions, descriptors[1]);
    pid_t child = -1;
    if(!error) error = posix_spawnp(&child, argv.front(), &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(descriptors[1]);
    if(error) {
        close(descriptors[0]);
        throw std::runtime_error("Cannot run " + arguments.front() + ": " + std::strerror(error));
    }
    std::string output;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    try {
        char buffer[4096];
        while(true) {
            if(std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Process timed out: " + arguments.front());
            pollfd descriptor{descriptors[0], POLLIN, 0};
            const int ready = poll(&descriptor, 1, 50);
            if(ready < 0 && errno == EINTR) continue;
            if(ready < 0) throw std::runtime_error("Process output poll failed");
            if(ready == 0) continue;
            const auto count = read(descriptors[0], buffer, sizeof(buffer));
            if(count < 0 && errno == EINTR) continue;
            if(count < 0) throw std::runtime_error("Process output read failed");
            if(count == 0) break;
            output.append(buffer, count);
            if(output.size() > 16 * 1024 * 1024) throw std::runtime_error("Process output exceeded 16 MiB");
        }
        int status = 0;
        while(true) {
            const auto result = waitpid(child, &status, WNOHANG);
            if(result == child) break;
            if(result < 0 && errno != EINTR) throw std::runtime_error("Cannot wait for process");
            if(std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Process exit timed out");
            poll(nullptr, 0, 10);
        }
        child = -1;
        close(descriptors[0]);
        descriptors[0] = -1;
        if(!WIFEXITED(status) || WEXITSTATUS(status) != 0) throw std::runtime_error("Process failed: " + arguments.front());
        return output;
    } catch(...) {
        if(descriptors[0] >= 0) close(descriptors[0]);
        if(child > 0) {
            kill(child, SIGKILL);
            while(waitpid(child, nullptr, 0) < 0 && errno == EINTR) {
            }
        }
        throw;
    }
}
}  // namespace depthai_bridge
