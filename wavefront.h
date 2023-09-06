#ifndef WAVEFRONT_H
#define WAVEFRONT_H

#include <fstream>
#include <ranges>
#include <map>
#include <filesystem>
#include <stdexcept>
#include "objects.h"
#include "materials.h"

namespace fs = std::filesystem;

std::vector<Object *> parseobj(const std::string &filename) {
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

    auto parsemtl = [&split](const std::string &filename, auto &materials) {
        int lineno = 0;
        try {
            std::ifstream ifs(filename);
            if (!ifs) assert(false);

            Material *m;
            std::string line;
            while (std::getline(ifs, line)) {
                ++lineno;
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
                } else if (toks[0] == "Ni") {
                    assert(toks.size() == 2);
                    m->index_of_refraction = std::stof(toks[1]);
                }
            }
        } catch(std::invalid_argument &e) {
            throw std::runtime_error(filename + ": " + std::to_string(lineno) + ": parse error");
        }
    };

    int lineno = 0;
    try {
        std::ifstream ifs(filename);
        if (!ifs) assert(false);

        std::map<std::string, Material *> materials;
        std::vector<Object *> objects;
        int offset = 0;
        Polygon *p = nullptr;
        std::string line;
        while (std::getline(ifs, line)) {
            ++lineno;
            std::vector<std::string> toks = split(line);

            if (toks[0] == "o") {
                if (p != nullptr) {
                    p->init();
                    objects.push_back(p);
                    offset += p->vertices.size();
                }
                p = new Polygon();
                continue;
            } else if (toks[0] == "s") {
                if (p == nullptr) p = new Polygon();
                assert(toks.size() == 2);
                if (toks[1] == "on") {
                    p->smooth = true;
                    continue;
                } else if (toks[1] == "off") {
                    p->smooth = false;
                    continue;
                }
                p->smooth = std::stoi(toks[1]) == 1;
            } else if (toks[0] == "v") {
                if (p == nullptr) p = new Polygon();
                assert(toks.size() == 4);
                p->vertices.push_back(vec(
                    std::stof(toks[1]),
                    std::stof(toks[2]),
                    std::stof(toks[3])
                ));
            } else if (toks[0] == "vn") {
                if (p == nullptr) p = new Polygon();
                assert(toks.size() == 4);
                p->vnormals.push_back(vec(
                    std::stof(toks[1]),
                    std::stof(toks[2]),
                    std::stof(toks[3])
                ));
            } else if(toks[0] == "f") {
                if (p == nullptr) p = new Polygon();
                std::vector<std::array<unsigned int, 2>> vs;
                for (const auto &vertex : toks | std::views::drop(1)) {
                    std::vector<std::string> toks = split(vertex, "/");
                    assert(toks.size() >= 2 && toks.size() <= 4);
                    vs.push_back({
                        (unsigned int)std::stoi(toks[0])-1 - offset,
                        toks.size() >= 3 ? (unsigned int)std::stoi(toks[2])-1 - offset : 0
                    });
                }
                p->faces.push_back({vs});
            } else if(toks[0] == "mtllib") {
                assert(toks.size() == 2);
                parsemtl(fs::path(filename).parent_path() / toks[1], materials);
            } else if(toks[0] == "usemtl") {
                if (p == nullptr) p = new Polygon();
                assert(toks.size() == 2);
                p->mat = materials[toks[1]];
            }
        }
        if (p != nullptr) {
            p->init();
            objects.push_back(p);
        }
        return objects;
    } catch(std::invalid_argument &e) {
        throw std::runtime_error(filename + ": " + std::to_string(lineno) + ": parse error");
    }
}

#endif