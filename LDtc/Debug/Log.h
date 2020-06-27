#ifndef LDTC_LOG_H
#define LDTC_LOG_H

#include <fstream>
#include <mutex>
#include <string>

namespace ld {
namespace det {
class Log {
   public:
    static void write(const std::string& s);
    static void setOut(std::ostream& out);

   private:
    Log() {}
    ~Log() {}

    Log(const Log&) = delete;
    Log& operator=(const Log&) = delete;

    static std::ostream* out_;
    static std::mutex lock_;
};

}  // namespace det
}  // namespace ld

#endif  // LDTC_LOG_H
