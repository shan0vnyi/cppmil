#include <string>
#define ENABLE_DEBUG  0

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

#include "telemetry.hpp"

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

long parse_long(const char* text) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);

    if (end == text) {
        throw std::invalid_argument(std::string("invalid long value: ") + text);
    }

    return value;
}

int parse_int(const char* text) {
    return static_cast<int>(parse_long(text));
}

double parse_double(const char* text) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);

    if (end == text) {
        throw std::invalid_argument(std::string("invalid double value: ") + text);
    }

    return value;
}

Frame parse_frame(char line[]) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

    if (field_count != EXPECTED_FIELD_COUNT) {
        throw std::runtime_error("expected " + std::to_string(EXPECTED_FIELD_COUNT)+ " fields");
    }

    DEBUG("f0 [" << fields[0] << "]; f1 ["<< fields[1]
        << "]; f2 [" << fields[2] << "]; f3 [" << fields[3]
        << "]; f4 [" << fields[4] << "]; f5 [" << fields[5]
        << "]; f6 [" << fields[6] << "]"
    );
    Frame frame{};
    frame.timestamp_ms = parse_long(fields[0]);
    frame.seq = parse_int(fields[1]);
    frame.voltage_v = parse_double(fields[2]);
    frame.current_a = parse_double(fields[3]);
    frame.temperature_c = parse_double(fields[4]);
    frame.gps_fix = parse_int(fields[5]);
    frame.satellites = parse_int(fields[6]);
    frame.isSet = true;

    return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count) {
    double rate = 0.0;
    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

    if (elapsed_ms == 0) {
        throw std::runtime_error("invalid frames period");
    } else {
        rate = static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
    }

    return rate;
}

int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error(std::string("failed to open input file: ") + path);
    }

    int frame_count = 0;
    char line[MAX_LINE_LENGTH];

    while (input.getline(line, MAX_LINE_LENGTH)) {
        if (line[0] == '\0') {
            continue;
        }

        if (frame_count < max_frames) {
            try {
                frames[frame_count] = parse_frame(line);

                if (frame_count > 0) {
                    if (frames[frame_count - 1].timestamp_ms >= frames[frame_count].timestamp_ms) {
                        throw std::runtime_error("invalid timestamp " + std::to_string(frames[frame_count].timestamp_ms));
                    }
                }

                ++frame_count;
            } catch (const std::exception& ex) {
                throw std::runtime_error("invalid frame at line " + std::to_string(frame_count + 1) + ": " + ex.what());
            }
        }
    }

    if (frame_count == 0) {
        throw std::runtime_error("empty frames");
    }

    return frame_count;
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;
    int frames_valid = 0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].isEmpty()) {
            DEBUG("Skip empty frame [" << i << "]");
            continue;
        }

        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }

        ++frames_valid;
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    summary.frames_valid = frames_valid;

    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
