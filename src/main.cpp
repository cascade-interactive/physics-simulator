#include <uaview/app/StudioApplication.hpp>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace {

int runUAViewStudio(const std::filesystem::path& executablePath, bool smokeTest) {
    try {
        uaview::app::StudioApplicationConfig config{};
        config.executablePath = executablePath;
        config.smokeTest = smokeTest;
        uaview::app::StudioApplication application(std::move(config));
        return application.run();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "UAView Studio failed: %s\n", error.what());
#if defined(_WIN32)
        OutputDebugStringA("UAView Studio failed: ");
        OutputDebugStringA(error.what());
        OutputDebugStringA("\n");
        if (!smokeTest) {
            MessageBoxA(
                nullptr,
                error.what(),
                "UAView Studio",
                MB_OK | MB_ICONERROR | MB_TASKMODAL
            );
        }
#endif
        return 1;
    } catch (...) {
        constexpr const char* message = "UAView Studio failed with an unknown error.";
        std::fprintf(stderr, "%s\n", message);
#if defined(_WIN32)
        OutputDebugStringA(message);
        OutputDebugStringA("\n");
        if (!smokeTest) {
            MessageBoxA(
                nullptr,
                message,
                "UAView Studio",
                MB_OK | MB_ICONERROR | MB_TASKMODAL
            );
        }
#endif
        return 1;
    }
}

#if defined(_WIN32)

std::filesystem::path currentExecutablePath() {
    std::wstring buffer(512U, L'\0');
    for (;;) {
        const DWORD length = GetModuleFileNameW(
            nullptr,
            buffer.data(),
            static_cast<DWORD>(buffer.size())
        );
        if (length == 0U) {
            return {};
        }
        if (length < buffer.size() - 1U) {
            buffer.resize(length);
            return std::filesystem::path(buffer);
        }
        buffer.resize(buffer.size() * 2U);
    }
}

bool commandLineHasSmokeTest(std::string_view commandLine) {
    constexpr std::string_view option = "--smoke-test";
    std::size_t position = commandLine.find(option);
    while (position != std::string_view::npos) {
        const std::size_t after = position + option.size();
        const bool startsAtBoundary =
            position == 0U ||
            commandLine[position - 1U] == ' ' ||
            commandLine[position - 1U] == '\t' ||
            commandLine[position - 1U] == '"';
        const bool endsAtBoundary =
            after == commandLine.size() ||
            commandLine[after] == ' ' ||
            commandLine[after] == '\t' ||
            commandLine[after] == '"';
        if (startsAtBoundary && endsAtBoundary) {
            return true;
        }
        position = commandLine.find(option, after);
    }
    return false;
}

#else

std::filesystem::path resolveExecutablePath(const char* argumentZero) {
    if (argumentZero == nullptr || *argumentZero == '\0') {
        return {};
    }

    std::error_code error;
    const std::filesystem::path argumentPath(argumentZero);
    if (argumentPath.has_parent_path()) {
        const std::filesystem::path absolute =
            std::filesystem::absolute(argumentPath, error);
        return error ? argumentPath : absolute;
    }

    const char* pathEnvironment = std::getenv("PATH");
    if (pathEnvironment != nullptr) {
        std::stringstream paths(pathEnvironment);
        std::string directory;
        while (std::getline(paths, directory, ':')) {
            const std::filesystem::path candidate =
                std::filesystem::path(directory) / argumentPath;
            if (std::filesystem::is_regular_file(candidate, error)) {
                const std::filesystem::path absolute =
                    std::filesystem::absolute(candidate, error);
                return error ? candidate : absolute;
            }
            error.clear();
        }
    }

    const std::filesystem::path absolute =
        std::filesystem::absolute(argumentPath, error);
    return error ? argumentPath : absolute;
}

#endif

} // namespace

#if defined(_WIN32)

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR commandLine, int) {
    const std::string_view arguments =
        commandLine != nullptr ? std::string_view(commandLine) : std::string_view{};
    return runUAViewStudio(
        currentExecutablePath(),
        commandLineHasSmokeTest(arguments)
    );
}

#else

int main(int argumentCount, char** arguments) {
    bool smokeTest = false;
    for (int index = 1; index < argumentCount; ++index) {
        if (std::string_view(arguments[index]) == "--smoke-test") {
            smokeTest = true;
        }
    }
    const std::filesystem::path executablePath =
        argumentCount > 0 ? resolveExecutablePath(arguments[0]) : std::filesystem::path{};
    return runUAViewStudio(executablePath, smokeTest);
}

#endif
