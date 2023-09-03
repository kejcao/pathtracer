#ifndef WAVEFRONT_H
#define WAVEFRONT_H

#include <fstream>
#include <ranges>
#include <map>
#include <filesystem>
#include "objects.h"
#include "materials.h"

namespace fs = std::filesystem;

std::vector<Object *> parseobj(const std::string &filename) {
    std::ifstream ifs(filename);
    if (!ifs) assert(false);

    auto split = [](
        const std::string &s,
        const std::string &delim=" "
    ) {
        std::vector<std::string> toks = {};
        size_t i = 0, pi = 0;
        while ((i = s.find(delim, pi)) != std::string::npos) {
            // skip empty strings
            // if (i-pi != 0) {
                toks.push_back(s.substr(pi, i-pi));
            // }
            pi = i + delim.size();
        }
        toks.push_back(s.substr(pi));
        return toks;
    };

    std::map<std::string, Material *> materials;
    auto parsemtl = [&](const std::string &filename) {
        std::ifstream ifs(filename);
        if (!ifs) assert(false);

        Material *m;
        std::string line;
        while (std::getline(ifs, line)) {
            std::vector<std::string> toks = split(line);

            if (toks[0] == "newmtl") {
                assert(toks.size() == 2);
                m = new Material();
                materials[toks[1]] = m;
            } else if (toks[0] == "Kd") {
                assert(toks.size() == 4);
                m->diffuse = vec(
                    std::stof(toks[1]),
                    std::stof(toks[2]),
                    std::stof(toks[3])
                );
            } else if (toks[0] == "Ks") {
                assert(toks.size() == 4);
                m->specular = vec(
                    std::stof(toks[1]),
                    std::stof(toks[2]),
                    std::stof(toks[3])
                );
            } else if (toks[0] == "Ns") {
                assert(toks.size() == 2);
                m->shininess = std::stof(toks[1]);
            }
        }
    };

    std::vector<Object *> objects;
    int offset = 0;
    Polygon *p = new Polygon();
    std::string line;
    while (std::getline(ifs, line)) {
        std::vector<std::string> toks = split(line);

        // TODO!!!! crashes if not prefixed with "o"
        if (toks[0] == "o") {
            objects.push_back(p);
            offset += p->vertices.size();
            p = new Polygon();
            continue;
        } else if (toks[0] == "s") {
            assert(toks.size() == 2);
            p->smooth = std::stoi(toks[1]) == 1;
        } else if (toks[0] == "v") {
            assert(toks.size() == 4);
            p->vertices.push_back(vec(
                std::stof(toks[1]),
                std::stof(toks[2]),
                std::stof(toks[3])
            ));
        } else if (toks[0] == "vn") {
            assert(toks.size() == 4);
            p->vnormals.push_back(vec(
                std::stof(toks[1]),
                std::stof(toks[2]),
                std::stof(toks[3])
            ));
        } else if(toks[0] == "f") {
            std::vector<std::array<unsigned int, 2>> vs;
            for (const auto &vertex : toks | std::views::drop(1)) {
                std::vector<std::string> toks = split(vertex, "/");
                assert(toks.size() >= 2 && toks.size() <= 4);
                vs.push_back({
                    (unsigned int)std::stoi(toks[0])-1 - offset,
                    toks.size() >= 3 ? (unsigned int)std::stoi(toks[2])-1 - offset : 0
                });
            }
            p->addfaces({vs});
        } else if(toks[0] == "mtllib") {
            assert(toks.size() == 2);
            parsemtl(fs::path(filename).parent_path() / toks[1]);
        } else if(toks[0] == "usemtl") {
            assert(toks.size() == 2);
            p->mat = materials[toks[1]];
        }
    }
    if (p != nullptr) {
        objects.push_back(p);
    }
    return objects;
}

#endif