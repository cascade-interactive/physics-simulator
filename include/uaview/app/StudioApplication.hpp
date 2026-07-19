#pragma once

#include <filesystem>
#include <memory>

namespace uaview::app {

struct StudioApplicationConfig {
    std::filesystem::path executablePath;
    bool smokeTest{false};
};

class StudioApplication {
public:
    explicit StudioApplication(StudioApplicationConfig config);
    ~StudioApplication();

    StudioApplication(const StudioApplication&) = delete;
    StudioApplication& operator=(const StudioApplication&) = delete;

    int run();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uaview::app
