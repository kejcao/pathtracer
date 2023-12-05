#ifndef SCENE_H
#define SCENE_H

#include <memory>
#include <vector>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include "lights.h"
#include "objects.h"

class Scene {
public:
    BVH objects;
    std::vector<PointLight *> pointlights;
    std::vector<AreaLight *> arealights;

    Scene(const std::string &filename) {
        Assimp::Importer importer;

        const aiScene* scene = importer.ReadFile(
            filename,
                aiProcess_CalcTangentSpace |
                aiProcess_Triangulate |
                aiProcess_JoinIdenticalVertices |
                aiProcess_SortByPType
        );

        if (!scene) {
            throw std::runtime_error(importer.GetErrorString());
        }

        std::vector<Material *> materials;
        for (int i = 0; i < scene->mNumMaterials; ++i) {
            auto m = scene->mMaterials[i];
            materials.push_back(new Material());

            for (int j = 0; j < m->mNumProperties; ++j) {
                auto p = m->mProperties[j];

                // this is bad. should use length variable because of certain utfs
                auto k = std::string(p->mKey.data);

                if (k == "$clr.diffuse") {
                    assert(p->mType == 1 && p->mDataLength == sizeof(float) * 3);
                    float *a = (float *)p->mData;
                    materials.back()->diffuse = vec(a[0], a[1], a[2]);
                }

                if (k == "$clr.specular") {
                    assert(p->mType == 1 && p->mDataLength == sizeof(float) * 3);
                    float *a = (float *)p->mData;
                    materials.back()->specular = vec(a[0], a[1], a[2]);
                }

                if (k == "$clr.emissive") {
                    assert(p->mType == 1 && p->mDataLength == sizeof(float) * 3);
                    float *a = (float *)p->mData;
                    materials.back()->emission = vec(a[0], a[1], a[2]);
                }
            }

            std::cout << std::endl;
        }

        // TODO: scene->mCameras;

        std::vector<std::unique_ptr<Object>> objs;

        auto **p = scene->mMeshes;
        for (size_t i = 0; i < scene->mNumMeshes; ++i) {
            objs.push_back(std::make_unique<Polygon>(p[i], materials[p[i]->mMaterialIndex]));
        }

        objects = BVH(std::move(objs));
    }

    // Scene(
    //     const std::vector<Object *> &objects,
    //     const std::vector<LightSource *> &lights
    // ) : objects{objects} {
    //     for (auto *l : lights) {
    //         if (dynamic_cast<PointLight *>(l) != nullptr) {
    //             pointlights.push_back((PointLight *)l);
    //         } else if (dynamic_cast<AreaLight *>(l) != nullptr) {
    //             arealights.push_back((AreaLight *)l);
    //         }
    //     }
    // }

    ~Scene() {
        for (auto &&light : pointlights) delete light;
        for (auto &&light : arealights) delete light;
    }

    // AreaLight *random_light() const {
    //     return arealights[rand() % arealights.size()];
    // }

    HitData castray(vec origin, vec direction) const {
        return objects.intersect(origin, direction);
    }

private:
};

#endif