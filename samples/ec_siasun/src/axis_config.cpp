#include "axis_config.h"

#include <cstdio>
#include <fstream>
#include <sys/stat.h>

#include "tinyxml2.h"

namespace siasun {
namespace {

bool is_directory(const std::string &path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string join_path(const std::string &directory,
                      const std::string &filename) {
    if (directory.empty() || directory.back() == '/' ||
        directory.back() == '\\') {
        return directory + filename;
    }
    return directory + "/" + filename;
}

bool parse_servo_parameter(const tinyxml2::XMLElement *element,
                           ServoParameter &parameter) {
    const tinyxml2::XMLElement *id_node = element->FirstChildElement("Id");
    const tinyxml2::XMLElement *name_node = element->FirstChildElement("Name");
    const tinyxml2::XMLElement *value_node = element->FirstChildElement("Value");
    const tinyxml2::XMLElement *qfmt_node = element->FirstChildElement("qFmt");
    if (!id_node || !value_node || !qfmt_node) {
        return false;
    }

    parameter.id = id_node->IntText(-1);
    if (parameter.id < 0) {
        return false;
    }
    if (name_node && name_node->GetText()) {
        parameter.name = name_node->GetText();
    }
    parameter.value = value_node->DoubleText(0.0);
    parameter.qfmt = qfmt_node->IntText(0);
    return true;
}

int load_axis_file(const std::string &path, AxisParameters &parameters) {
    std::printf("[XML] loading axis parameter file: %s\n", path.c_str());

    tinyxml2::XMLDocument doc;
    const tinyxml2::XMLError load_result = doc.LoadFile(path.c_str());
    if (load_result != tinyxml2::XML_SUCCESS) {
        std::fprintf(stderr, "failed to load %s: %s\n", path.c_str(),
                     doc.ErrorStr());
        return -1;
    }

    const tinyxml2::XMLElement *root = doc.FirstChildElement("dataentry");
    if (!root) {
        std::fprintf(stderr, "missing dataentry root in %s\n", path.c_str());
        return -1;
    }

    std::size_t total_count = 0;
    std::size_t skipped_count = 0;
    for (const tinyxml2::XMLElement *element =
             root->FirstChildElement("ServoParameters");
         element;
         element = element->NextSiblingElement("ServoParameters")) {
        ++total_count;
        ServoParameter parameter;
        if (parse_servo_parameter(element, parameter)) {
            parameters.push_back(parameter);
            if (kLogAxisXmlParameterDetails) {
                std::printf("[XML]   valid id=%d name=%s value=%.10g qFmt=%d\n",
                            parameter.id,
                            parameter.name.c_str(),
                            parameter.value,
                            parameter.qfmt);
            }
        } else {
            ++skipped_count;
        }
    }

    if (parameters.empty()) {
        std::fprintf(stderr, "no valid ServoParameters in %s\n", path.c_str());
        return -1;
    }

    std::printf("[XML] loaded %zu valid parameters from %s "
                "(nodes=%zu skipped=%zu)\n",
                parameters.size(),
                path.c_str(),
                total_count,
                skipped_count);
    return 0;
}

}  // namespace

int resolve_axis_config_directory(const std::string &requested_directory,
                                  std::string &selected_directory) {
    std::vector<std::string> candidates;
    if (!requested_directory.empty()) {
        candidates.push_back(requested_directory);
    } else {
        candidates.push_back("samples/ec_siasun/doc/gcr10_1300");
        candidates.push_back("work-soem-200/samples/ec_siasun/doc/gcr10_1300");
    }

    for (const auto &candidate : candidates) {
        const std::string axis1_path = join_path(candidate, "Axis1.xml");
        std::printf("[XML] checking Axis*.xml directory: %s\n",
                    candidate.c_str());
        if (!is_directory(candidate)) {
            continue;
        }

        std::ifstream input(axis1_path);
        if (input.good()) {
            selected_directory = candidate;
            std::printf("[XML] selected Axis*.xml directory: %s\n",
                        selected_directory.c_str());
            return 0;
        }
    }

    std::fprintf(stderr, "failed to resolve Axis*.xml directory\n");
    return -1;
}

int load_axis_parameter_set(const std::string &directory,
                            AxisParameterSet &parameters) {
    for (std::size_t axis = 0; axis < kServoCount; ++axis) {
        const std::string filename = "Axis" + std::to_string(axis + 1) + ".xml";
        if (load_axis_file(join_path(directory, filename), parameters[axis])) {
            return -1;
        }
    }

    std::printf("[XML] all Axis*.xml files loaded successfully from %s\n",
                directory.c_str());
    return 0;
}

}  // namespace siasun
