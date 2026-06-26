#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace motor_control::utils {

class CsvLogger {
public:
    CsvLogger(const std::filesystem::path& filePath, const std::vector<std::string>& headers) {
        const auto parentPath = filePath.parent_path();
        if (!parentPath.empty()) {
            std::filesystem::create_directories(parentPath);
        }

        _stream.open(filePath);
        if (!_stream.is_open()) {
            throw std::runtime_error("Failed to open CSV file: " + filePath.string());
        }

        writeHeader(headers);
    }

    void writeRow(std::initializer_list<double> values) {
        bool isFirst = true;
        for (const double value : values) {
            if (!isFirst) {
                _stream << ',';
            }
            _stream << std::setprecision(12) << value;
            isFirst = false;
        }
        _stream << '\n';
    }

private:
    void writeHeader(const std::vector<std::string>& headers) {
        for (std::size_t index = 0; index < headers.size(); ++index) {
            if (index > 0) {
                _stream << ',';
            }
            _stream << headers[index];
        }
        _stream << '\n';
    }

    std::ofstream _stream;
};

} // namespace motor_control::utils
