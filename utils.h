#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <filesystem>
#include <optional>

#define UTL_SEPARATOR "\\\\"

typedef enum {
    DebugLevelDevice,
    DebugLevelRxRaw,
    DebugLevelRx,
    DebugLevelTx,
    DebugLevelTemperature,
    DebugLevelDigitallOffsetCompensation,
    DebugLevelMaxSpeed,
    DebugLevelsNum
} DebugLevels_t;

inline void createDebugFile(FILE * &fid, std::string fileName) {
    const char * home = std::getenv("USERPROFILE");
    std::filesystem::path filePath = std::filesystem::path(home) / (fileName + ".txt");
    fid = fopen(filePath.string().c_str(), "wb");
}

inline bool debugLevelEnabled(DebugLevels_t level) {
    static std::vector <std::optional <bool>> cachedResult(DebugLevelsNum);
    if (!cachedResult[level].has_value()) {
#ifdef _WIN32
        const char * home = std::getenv("USERPROFILE");
#elif __linux__
        const char * home = std::getenv("HOME");
#endif
        std::string filename;
        switch (level) {
        case DebugLevelDevice:
            filename = "er4_DEMO.pls";
            break;

        case DebugLevelRxRaw:
            filename = "er4_RX_RAW.pls";
            break;

        case DebugLevelRx:
            filename = "er4_RX.pls";
            break;

        case DebugLevelTx:
            filename = "er4_TX.pls";
            break;

        case DebugLevelTemperature:
            filename = "er4_TEMP.pls";
            break;

        case DebugLevelDigitallOffsetCompensation:
            filename = "er4_DOC.pls";
            break;

        default:
            return false;
        }
        std::filesystem::path filePath = std::filesystem::path(home) / filename;
        cachedResult[level] = std::filesystem::exists(filePath) && std::filesystem::is_regular_file(filePath);
    }
    return * cachedResult[level];
}

#endif // UTILS_H
