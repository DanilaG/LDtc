#include "Log.h"

#include <ctime>
#include <iomanip>
#include <iostream>

namespace ld {
namespace det {
std::ostream* Log::out_ = &std::cout;
std::mutex Log::lock_;

void Log::write(const std::string& s) {
    auto time = std::time(nullptr);
    auto local_time = *std::localtime(&time);
    lock_.lock();
    (*out_) << std::put_time(&local_time, "%Y-%m-%d_%H:%M:%S: ") << s
            << std::endl;
    lock_.unlock();
}

void Log::setOut(std::ostream& out) { out_ = &out; }

}  // namespace det
}  // namespace ld
